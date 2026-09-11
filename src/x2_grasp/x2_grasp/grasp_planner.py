"""Pure grasp geometry checks and staged IK planning."""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np

from x2_arm.config import ArmSide

from .grasp_constants import (
    ARM_CENTERLINE_CLEARANCE,
    GRASP_AXIS_WORLD,
    GRASP_X_REACHABLE,
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


def check_reachable(grasp_xyz, target_xyz, args, source_label, side):
    grasp_x, grasp_y = grasp_xyz[0], grasp_xyz[1]
    low_x, high_x = GRASP_X_REACHABLE
    problems = []
    if side == ArmSide.RIGHT and grasp_y > -ARM_CENTERLINE_CLEARANCE:
        problems.append(
            f"抓取点 Y={grasp_y:.3f} 比右臂可达上限 "
            f"{-ARM_CENTERLINE_CLEARANCE:.2f} 更靠中线"
        )
    if side == ArmSide.LEFT and grasp_y < ARM_CENTERLINE_CLEARANCE:
        problems.append(
            f"抓取点 Y={grasp_y:.3f} 比左臂可达下限 "
            f"{ARM_CENTERLINE_CLEARANCE:.2f} 更靠中线"
        )
    if grasp_x > high_x:
        problems.append(
            f"抓取点 X={grasp_x:.3f} 超出{side.value}臂前伸极限 {high_x:.2f}"
        )
    if grasp_x < low_x:
        problems.append(
            f"抓取点 X={grasp_x:.3f} 低于{side.value}臂近身极限 {low_x:.2f}"
        )
    if problems:
        raise RuntimeError(
            f"{source_label}目标不可达（抓取点={grasp_xyz}，视觉目标={target_xyz}）："
            + "；".join(problems)
        )


def ik_seed_candidates(node, primary, retract, perturbation, side=ArmSide.RIGHT):
    yield list(primary)
    if any(abs(a - b) > 1e-9 for a, b in zip(primary, retract)):
        yield list(retract)
    offset = 0 if side == ArmSide.LEFT else 7
    for index in (offset + 2, offset + 3, offset + 6):
        for direction in (-1.0, 1.0):
            candidate = list(primary)
            candidate[index] += direction * perturbation
            yield node.solver.clip_arm_pos(candidate)


def solve_grasp_axis(
    node,
    target_xyz,
    current_arm_pos,
    retract_arm_pos,
    args,
    description,
    side=ArmSide.RIGHT,
    approximate_position_tolerance=None,
    approximate_axis_tolerance=None,
    cancel_requested=lambda: False,
):
    results = []
    for seed_index, seed in enumerate(
        ik_seed_candidates(
            node,
            current_arm_pos,
            retract_arm_pos,
            args.ik_seed_perturbation,
            side,
        ),
        1,
    ):
        _check_canceled(cancel_requested)
        result = node.solver.solve_axis(
            side=side,
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
                    np.asarray(result.arm_pos[_arm_slice(side)])
                    - np.asarray(current_arm_pos[_arm_slice(side)])
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
    node, grasp_xyz, grasp_result, retract_arm_pos, args, cancel_requested, side
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
                side=side,
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
    side=ArmSide.RIGHT,
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
            side=side,
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
    side=ArmSide.RIGHT,
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
                side=side,
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


def _arm_slice(side):
    return slice(0, 7) if side == ArmSide.LEFT else slice(7, 14)


def plan_holding_return(
    node, start_result, initial_arm_pos, retract_arm_pos, args, side,
    cancel_requested=lambda: False,
):
    """Plan raise, horizontal return, descent, then exact posture restoration."""
    initial_xyz = node.solver.fk_xyz(side, initial_arm_pos)
    start_xyz = node.solver.fk_xyz(side, start_result.arm_pos)
    height = max(start_xyz[2], initial_xyz[2] + args.initial_upward)
    targets = [
        ("return_raising", [start_xyz[0], start_xyz[1], height]),
        ("returning_high", [initial_xyz[0], initial_xyz[1], height]),
        ("return_lowering", list(initial_xyz)),
    ]
    segments = []
    seed = start_result
    for stage, target in targets:
        results = solve_cartesian_segment(
            node, start_xyz, target, seed, retract_arm_pos, args, stage,
            cancel_requested, side,
        )
        # Joint interpolation can bow away from Cartesian waypoints. Check
        # the connecting curves as well, including the actual IK endpoints.
        previous = seed.arm_pos
        line_start = np.asarray(start_xyz, dtype=float)
        line_delta = np.asarray(target, dtype=float) - line_start
        length_squared = float(np.dot(line_delta, line_delta))
        for result in results:
            previous_array = np.asarray(previous, dtype=float)
            delta = np.asarray(result.arm_pos, dtype=float) - previous_array
            samples = max(20, int(math.ceil(float(np.max(np.abs(delta))) / 0.005)))
            for fraction in np.linspace(0.0, 1.0, samples + 1):
                _check_canceled(cancel_requested)
                joints = (previous_array + fraction * delta).tolist()
                xyz = np.asarray(node.solver.fk_xyz(side, joints), dtype=float)
                progress = (
                    float(np.dot(xyz - line_start, line_delta)) / length_squared
                    if length_squared > 1e-12 else 0.0
                )
                nearest = line_start + np.clip(progress, 0.0, 1.0) * line_delta
                if (
                    not np.all(np.isfinite(xyz))
                    or np.linalg.norm(xyz - nearest) > args.high_retract_position_tolerance
                ):
                    raise RuntimeError(
                        f"{stage}: interpolated return path exceeds Cartesian tolerance"
                    )
            previous = result.arm_pos
        segments.append((stage, results))
        seed = results[-1]
        start_xyz = node.solver.fk_xyz(side, seed.arm_pos)
    return segments


def _plan_joint_travel(plan, current_arm_pos):
    states = [
        current_arm_pos,
        plan.retract_arm_pos,
        plan.pre_grasp.arm_pos,
        *(result.arm_pos for result in plan.approach_steps),
        *(result.arm_pos for result in plan.lift_steps),
        *(result.arm_pos for result in plan.high_retract_steps),
        *(result.arm_pos for _stage, results in plan.return_segments for result in results),
        current_arm_pos,
    ]
    active = _arm_slice(plan.side)
    return sum(
        float(
            np.linalg.norm(
                np.asarray(after[active], dtype=float)
                - np.asarray(before[active], dtype=float)
            )
        )
        for before, after in zip(states, states[1:])
    )


def _plan_grasp_for_side(
    node,
    current_arm_pos,
    target_xyz,
    args,
    source,
    side,
    cancel_requested=lambda: False,
    feedback=lambda _stage, _detail="": None,
):
    feedback("planning", "solving staged grasp IK")
    _check_canceled(cancel_requested)
    depth_offset = args.tag_depth if source == "apriltag" else 0.0
    object_center_x = target_xyz[0] + depth_offset
    grasp_xyz = [
        object_center_x - args.gripper_reach + args.grasp_x_offset,
        target_xyz[1],
        args.grasp_plane_z,
    ]
    source_label = SOURCE_LABELS[source]
    check_reachable(grasp_xyz, target_xyz, args, source_label, side)

    current_xyz = node.solver.fk_xyz(side, current_arm_pos)
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
        side=side,
        cancel_requested=cancel_requested,
    )
    retract_arm_pos = retract_result.arm_pos
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
        side=side,
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
        side,
    )
    grasp_result = approach_results[-1]
    lift_results, achieved_lift = solve_lift_steps(
        node,
        grasp_xyz,
        grasp_result,
        retract_arm_pos,
        args,
        cancel_requested,
        side,
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
        side,
    )
    return_segments = plan_holding_return(
        node, high_retract_results[-1] if high_retract_results else lift_results[-1],
        list(current_arm_pos), retract_arm_pos, args, side, cancel_requested,
    )
    return SimpleNamespace(
        side=side,
        retract=retract_result,
        retract_arm_pos=retract_arm_pos,
        pre_grasp=pre_grasp_result,
        approach_steps=approach_results,
        grasp=grasp_result,
        lift_steps=lift_results,
        post_grasp=lift_results[-1],
        high_retract_steps=high_retract_results,
        return_segments=return_segments,
        achieved_lift=achieved_lift,
    )


def plan_grasp(
    node,
    current_arm_pos,
    target_xyz,
    args,
    source,
    cancel_requested=lambda: False,
    feedback=lambda _stage, _detail="": None,
):
    requested_side = getattr(args, "arm_side", "auto")
    if requested_side != "auto":
        return _plan_grasp_for_side(
            node,
            current_arm_pos,
            target_xyz,
            args,
            source,
            ArmSide(requested_side),
            cancel_requested,
            feedback,
        )

    plans = []
    failures = []
    preferred = (
        (ArmSide.LEFT, ArmSide.RIGHT)
        if target_xyz[1] >= 0.0
        else (ArmSide.RIGHT, ArmSide.LEFT)
    )
    for side in preferred:
        _check_canceled(cancel_requested)
        feedback("planning", f"evaluating {side.value} arm")
        try:
            plan = _plan_grasp_for_side(
                node,
                current_arm_pos,
                target_xyz,
                args,
                source,
                side,
                cancel_requested,
                feedback,
            )
        except GraspCancelled:
            raise
        except RuntimeError as error:
            failures.append(f"{side.value}: {error}")
            node.node.get_logger().warning(
                f"{side.value} arm grasp plan rejected: {error}"
            )
            continue
        plan.selection_cost = _plan_joint_travel(plan, current_arm_pos)
        plans.append(plan)

    if not plans:
        raise RuntimeError("左右臂均无法完成抓取规划：" + "；".join(failures))
    selected = min(plans, key=lambda plan: plan.selection_cost)
    node.node.get_logger().info(
        f"selected {selected.side.value} arm; "
        f"joint_travel={selected.selection_cost:.3f}"
    )
    feedback(
        "planning",
        f"selected {selected.side.value} arm "
        f"(joint travel {selected.selection_cost:.3f} rad)",
    )
    return selected


def grip_close_position_for(args, target):
    catalog = getattr(args, "target_catalog", None)
    if catalog is not None:
        return catalog.spec(target).grip_close_position
    return getattr(args, f"{target}_grip_close_position")
