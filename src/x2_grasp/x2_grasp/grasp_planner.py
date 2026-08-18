"""Pure grasp geometry checks and staged IK planning."""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np

from x2_arm.config import ArmSide

from .grasp_constants import (
    GRASP_AXIS_WORLD,
    GRASP_X_REACHABLE,
    RIGHT_ARM_Y_MAX,
    SOURCE_LABELS,
    TARGET_X_RANGE,
    TARGET_Y_RANGE,
    TARGET_Z_RANGE,
)
from .grasp_errors import GraspCancelled


def _check_canceled(cancel_requested) -> None:
    if cancel_requested():
        raise GraspCancelled("grasp goal canceled during IK planning")


def validate_target(xyz, frame_id, source_label):
    if frame_id and frame_id != "base_link":
        raise RuntimeError(
            f"{source_label}目标向量坐标系为 {frame_id}，本节点只接受 base_link"
        )
    if len(xyz) != 3 or not all(math.isfinite(value) for value in xyz):
        raise RuntimeError(f"{source_label}目标向量必须包含三个有限值：{xyz}")
    for value, (low, high), axis in zip(
        xyz, (TARGET_X_RANGE, TARGET_Y_RANGE, TARGET_Z_RANGE), "XYZ"
    ):
        if not low <= value <= high:
            raise RuntimeError(
                f"{source_label}目标 {axis}={value:.3f} m "
                f"超出安全范围 [{low}, {high}]"
            )


def check_reachable(grasp_xyz, target_xyz, args, source_label):
    grasp_x, grasp_y = grasp_xyz[0], grasp_xyz[1]
    low_x, high_x = GRASP_X_REACHABLE
    problems = []
    if grasp_y > RIGHT_ARM_Y_MAX:
        problems.append(
            f"抓取点 Y={grasp_y:.3f} 比右臂可达上限 "
            f"{RIGHT_ARM_Y_MAX:.2f} 更靠中线"
        )
    if grasp_x > high_x:
        problems.append(f"抓取点 X={grasp_x:.3f} 超出右臂前伸极限 {high_x:.2f}")
    if grasp_x < low_x:
        problems.append(f"抓取点 X={grasp_x:.3f} 低于右臂近身极限 {low_x:.2f}")
    if problems:
        raise RuntimeError(
            f"{source_label}目标不可达（抓取点={grasp_xyz}，视觉目标={target_xyz}）："
            + "；".join(problems)
        )


def ik_seed_candidates(node, primary, retract, perturbation):
    candidates = [list(primary)]
    if any(abs(a - b) > 1e-9 for a, b in zip(primary, retract)):
        candidates.append(list(retract))
    for index in (9, 10, 13):
        for direction in (-1.0, 1.0):
            candidate = list(primary)
            candidate[index] += direction * perturbation
            candidates.append(node.solver.clip_arm_pos(candidate))
    return candidates


def solve_grasp_axis(
    node,
    target_xyz,
    current_arm_pos,
    retract_arm_pos,
    args,
    description,
    approximate_position_tolerance=None,
    approximate_axis_tolerance=None,
    cancel_requested=lambda: False,
):
    results = []
    for seed_index, seed in enumerate(
        ik_seed_candidates(
            node, current_arm_pos, retract_arm_pos, args.ik_seed_perturbation
        ),
        1,
    ):
        _check_canceled(cancel_requested)
        result = node.solver.solve_axis(
            side=ArmSide.RIGHT,
            target_xyz=target_xyz,
            target_axis=GRASP_AXIS_WORLD,
            current_arm_pos=seed,
            orientation_weight=args.grasp_axis_orientation_weight,
            orientation_eps=args.grasp_axis_orientation_eps,
        )
        results.append(result)
        if result.success:
            joint_travel = float(
                np.linalg.norm(
                    np.asarray(result.arm_pos[7:])
                    - np.asarray(current_arm_pos[7:])
                )
            )
            result = SimpleNamespace(
                **vars(result), joint_travel=joint_travel, seed_index=seed_index
            )
            node.node.get_logger().info(
                f"{description} IK success; seed={seed_index}; "
                f"position_error={result.position_error_norm}; "
                f"axis_error={result.orientation_error_norm}"
            )
            return result

    if (
        approximate_position_tolerance is not None
        and approximate_axis_tolerance is not None
    ):
        acceptable = [
            (index, result)
            for index, result in enumerate(results, 1)
            if result.position_error_norm <= approximate_position_tolerance
            and result.orientation_error_norm <= approximate_axis_tolerance
        ]
        if acceptable:
            seed_index, best = min(
                acceptable,
                key=lambda item: (
                    item[1].position_error_norm / approximate_position_tolerance
                    + item[1].orientation_error_norm / approximate_axis_tolerance
                ),
            )
            return SimpleNamespace(
                **vars(best),
                seed_index=seed_index,
                accepted_approximate=True,
            )

    best = min(
        results,
        key=lambda result: (
            result.position_error_norm,
            result.orientation_error_norm,
        ),
    )
    raise RuntimeError(
        f"{description} IK failed after {len(results)} seeds: {best.message}; "
        f"position_error={best.position_error_norm}; "
        f"axis_error={best.orientation_error_norm}"
    )


def solve_lift_steps(
    node, grasp_xyz, grasp_result, retract_arm_pos, args, cancel_requested
):
    results = []
    achieved = 0.0
    while achieved + 1e-9 < args.post_grasp_lift:
        _check_canceled(cancel_requested)
        next_height = min(args.post_grasp_lift, achieved + args.lift_step)
        target_xyz = [grasp_xyz[0], grasp_xyz[1], grasp_xyz[2] + next_height]
        try:
            result = solve_grasp_axis(
                node,
                target_xyz,
                results[-1].arm_pos if results else grasp_result.arm_pos,
                retract_arm_pos,
                args,
                f"抓取后分步抬高到{next_height * 100:.0f}cm",
                cancel_requested=cancel_requested,
            )
        except GraspCancelled:
            raise
        except RuntimeError:
            if achieved + 1e-9 < args.min_post_grasp_lift:
                raise
            node.node.get_logger().warning(
                f"期望抬高未完全可解，降级到 {achieved * 100:.0f}cm"
            )
            break
        results.append(result)
        achieved = next_height
    if not results:
        raise RuntimeError("抓取后没有生成任何抬高 IK 结果")
    return results, achieved


def solve_cartesian_segment(
    node,
    start_xyz,
    target_xyz,
    start_result,
    retract_arm_pos,
    args,
    description,
    cancel_requested=lambda: False,
):
    start = np.asarray(start_xyz, dtype=float)
    target = np.asarray(target_xyz, dtype=float)
    distance = float(np.linalg.norm(target - start))
    steps = max(1, int(math.ceil(distance / args.cartesian_step)))
    results = []
    seed = start_result.arm_pos
    for step in range(1, steps + 1):
        _check_canceled(cancel_requested)
        waypoint = (start + step / steps * (target - start)).tolist()
        result = solve_grasp_axis(
            node,
            waypoint,
            seed,
            retract_arm_pos,
            args,
            f"{description} {step}/{steps}",
            cancel_requested=cancel_requested,
        )
        results.append(result)
        seed = result.arm_pos
    return results


def solve_high_retract_segment(
    node,
    start_xyz,
    target_xyz,
    start_result,
    retract_arm_pos,
    args,
    description,
    cancel_requested=lambda: False,
):
    start = np.asarray(start_xyz, dtype=float)
    target = np.asarray(target_xyz, dtype=float)
    distance = float(np.linalg.norm(target - start))
    steps = max(1, int(math.ceil(distance / args.cartesian_step)))
    results = []
    seed = start_result.arm_pos
    for step in range(1, steps + 1):
        _check_canceled(cancel_requested)
        waypoint = (start + step / steps * (target - start)).tolist()
        try:
            result = solve_grasp_axis(
                node,
                waypoint,
                seed,
                retract_arm_pos,
                args,
                f"{description} {step}/{steps}",
                approximate_position_tolerance=(
                    args.high_retract_position_tolerance
                ),
                approximate_axis_tolerance=args.high_retract_axis_tolerance,
                cancel_requested=cancel_requested,
            )
        except GraspCancelled:
            raise
        except RuntimeError as error:
            node.node.get_logger().warning(f"高位后撤提前停止：{error}")
            break
        results.append(result)
        seed = result.arm_pos
    return results


def plan_grasp(
    node,
    current_arm_pos,
    target_xyz,
    args,
    source,
    cancel_requested=lambda: False,
    feedback=lambda _stage, _detail="": None,
):
    feedback("planning", "solving staged grasp IK")
    _check_canceled(cancel_requested)
    current_xyz = node.solver.fk_xyz(ArmSide.RIGHT, current_arm_pos)
    retract_xyz = [
        current_xyz[0] - args.backward,
        current_xyz[1],
        current_xyz[2] + args.initial_upward,
    ]
    retract_result = solve_grasp_axis(
        node,
        retract_xyz,
        current_arm_pos,
        current_arm_pos,
        args,
        "准备段后撤抬高",
        cancel_requested=cancel_requested,
    )
    retract_arm_pos = retract_result.arm_pos
    depth_offset = args.tag_depth if source == "apriltag" else 0.0
    object_center_x = target_xyz[0] + depth_offset
    grasp_xyz = [
        object_center_x - args.gripper_reach + args.grasp_x_offset,
        target_xyz[1],
        args.grasp_plane_z,
    ]
    source_label = SOURCE_LABELS[source]
    check_reachable(grasp_xyz, target_xyz, args, source_label)

    standoff = args.standoff
    min_pre_x = GRASP_X_REACHABLE[0]
    if grasp_xyz[0] - standoff < min_pre_x - 1e-6:
        standoff = max(0.0, grasp_xyz[0] - min_pre_x)
        if standoff < 0.01:
            raise RuntimeError("抓取点太靠近躯干，无法保留安全接近距离")
    pre_grasp_xyz = [
        grasp_xyz[0] - standoff,
        grasp_xyz[1],
        args.grasp_plane_z,
    ]
    pre_grasp_result = solve_grasp_axis(
        node,
        pre_grasp_xyz,
        retract_arm_pos,
        retract_arm_pos,
        args,
        "预抓取点",
        cancel_requested=cancel_requested,
    )
    approach_results = solve_cartesian_segment(
        node,
        pre_grasp_xyz,
        grasp_xyz,
        pre_grasp_result,
        retract_arm_pos,
        args,
        "水平接近",
        cancel_requested,
    )
    grasp_result = approach_results[-1]
    lift_results, achieved_lift = solve_lift_steps(
        node,
        grasp_xyz,
        grasp_result,
        retract_arm_pos,
        args,
        cancel_requested,
    )
    post_grasp_xyz = [
        grasp_xyz[0],
        grasp_xyz[1],
        args.grasp_plane_z + achieved_lift,
    ]
    high_retract_target = [
        pre_grasp_xyz[0],
        grasp_xyz[1],
        post_grasp_xyz[2],
    ]
    high_retract_results = solve_high_retract_segment(
        node,
        post_grasp_xyz,
        high_retract_target,
        lift_results[-1],
        retract_arm_pos,
        args,
        "高位后撤",
        cancel_requested,
    )
    return SimpleNamespace(
        retract=retract_result,
        retract_arm_pos=retract_arm_pos,
        pre_grasp=pre_grasp_result,
        approach_steps=approach_results,
        grasp=grasp_result,
        lift_steps=lift_results,
        post_grasp=lift_results[-1],
        high_retract_steps=high_retract_results,
        achieved_lift=achieved_lift,
    )


def grip_close_position_for(args, target):
    catalog = getattr(args, "target_catalog", None)
    if catalog is not None:
        return catalog.spec(target).grip_close_position
    return getattr(args, f"{target}_grip_close_position")
