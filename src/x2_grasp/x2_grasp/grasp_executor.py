"""Execute an already validated grasp plan on X2 hardware."""

from __future__ import annotations

from types import SimpleNamespace
import threading
import time

from x2_common import RetryExhaustedError, run_with_retry

from .grasp_errors import GraspCancelled
from .grasp_planner import grip_close_position_for


def make_hardware_args(args):
    return SimpleNamespace(
        urdf=None,
        side="right",
        target=None,
        target_6d=None,
        target_rpy=None,
        keep_current_rpy=False,
        orientation_weight=1.0,
        orientation_eps=1e-3,
        dry_run=not args.execute,
        duration=args.duration,
        command_topic="/mc/upper_body_command",
        joint_state_service="/aimdk_5Fmsgs/srv/GetAllJointState",
        action_service="/aimdk_5Fmsgs/srv/SetMcAction",
        source="x2_arm",
        frame_id="mc_upper_body",
        hand_command_topic="/aima/hal/joint/hand/command",
        hand_publish_hz=50.0,
    )


def request_action_with_retry(
    node, action_desc, attempts=3, cancel_requested=lambda: False
):
    stop_requested = threading.Event()

    def request(attempt):
        if cancel_requested():
            stop_requested.set()
            raise RuntimeError("grasp goal canceled")
        node.node.get_logger().info(
            f"请求模式 {action_desc}（第 {attempt}/{attempts} 次）"
        )
        node.switch_action(action_desc)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if cancel_requested():
                stop_requested.set()
                raise RuntimeError("grasp goal canceled")
            time.sleep(min(0.1, deadline - time.monotonic()))

    try:
        run_with_retry(
            request,
            stop_requested,
            max_attempts=attempts,
            initial_delay_seconds=1.0,
            max_delay_seconds=1.0,
        )
    except RetryExhaustedError as error:
        raise RuntimeError(
            f"模式请求最终失败：{action_desc}: {error.__cause__}"
        ) from error
    if stop_requested.is_set() or cancel_requested():
        raise GraspCancelled("grasp goal canceled while switching control mode")


def _check_canceled(cancel_requested) -> None:
    if cancel_requested():
        raise GraspCancelled("grasp goal canceled during hardware execution")


def _publish_trajectory(node, start, goal, duration, cancel_requested) -> None:
    _check_canceled(cancel_requested)
    try:
        node.publish_trajectory(
            start,
            goal,
            duration,
            cancel_requested=cancel_requested,
        )
    except InterruptedError as error:
        raise GraspCancelled(str(error)) from error


def execute_grasp(
    node,
    current_arm_pos,
    plan,
    args,
    target,
    cancel_requested=lambda: False,
    feedback=lambda _stage, _detail="": None,
):
    grip_position = grip_close_position_for(args, target)
    side = plan.side.value

    feedback("preparing", "closing gripper and moving to clearance pose")
    _check_canceled(cancel_requested)
    node.close_gripper(side, seconds=args.initial_close_seconds)
    _publish_trajectory(
        node,
        current_arm_pos,
        plan.retract_arm_pos,
        args.duration,
        cancel_requested,
    )

    feedback("approaching", "opening gripper and moving to pre-grasp pose")
    _check_canceled(cancel_requested)
    node.open_gripper(side, seconds=args.open_seconds)
    _publish_trajectory(
        node,
        plan.retract_arm_pos,
        plan.pre_grasp.arm_pos,
        args.duration,
        cancel_requested,
    )

    approach_start = plan.pre_grasp.arm_pos
    approach_duration = args.approach_duration / len(plan.approach_steps)
    for index, approach_result in enumerate(plan.approach_steps, 1):
        feedback("approaching", f"cartesian step {index}/{len(plan.approach_steps)}")
        _publish_trajectory(
            node,
            approach_start,
            approach_result.arm_pos,
            approach_duration,
            cancel_requested,
        )
        approach_start = approach_result.arm_pos

    feedback("gripping", f"setting {side} gripper to {grip_position:.3f}")
    _check_canceled(cancel_requested)
    node.set_gripper_position(
        side, grip_position, seconds=args.grip_close_seconds
    )

    lift_start = plan.grasp.arm_pos
    lift_duration = args.duration * args.lift_step / max(
        plan.achieved_lift, args.lift_step
    )
    for index, lift_result in enumerate(plan.lift_steps, 1):
        feedback("lifting", f"lift step {index}/{len(plan.lift_steps)}")
        _publish_trajectory(
            node,
            lift_start,
            lift_result.arm_pos,
            lift_duration,
            cancel_requested,
        )
        lift_start = lift_result.arm_pos

    high_retract_start = plan.post_grasp.arm_pos
    if plan.high_retract_steps:
        retract_duration = args.duration / len(plan.high_retract_steps)
        for index, result in enumerate(plan.high_retract_steps, 1):
            feedback(
                "retracting",
                f"high retract step {index}/{len(plan.high_retract_steps)}",
            )
            _publish_trajectory(
                node,
                high_retract_start,
                result.arm_pos,
                retract_duration,
                cancel_requested,
            )
            high_retract_start = result.arm_pos

    feedback("completed", "physical grasp sequence completed")
