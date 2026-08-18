"""ROS 2 Action-based orchestrator for the X2 visual grasp workflow."""

from __future__ import annotations

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from x2_grasp.msg import GroundingCommand

from x2_arm.hardware_node import X2HardwareNode, _require_ros2_imports

from .audio_player import GraspAudioPlayer
from .grasp_action_server import GraspActionBridge
from .grasp_errors import GraspCancelled
from .grasp_executor import (
    execute_grasp,
    make_hardware_args,
    request_action_with_retry,
)
from .grasp_parameters import (
    build_parser,
    parse_ros_params_into_args,
    validate_parameters,
)
from .grasp_planner import (
    SOURCE_LABELS,
    grip_close_position_for,
    ik_seed_candidates,
    plan_grasp,
    solve_cartesian_segment,
    solve_high_retract_segment,
    validate_target,
)
from .perception_coordinator import (
    GroundingResultListener,
    LocalizerStatusListener,
    TargetListener,
    acquire_grounding_target_with_retry,
)


def _wait_for_goal(action_bridge, node, rclpy_mod):
    while rclpy_mod.ok():
        work = action_bridge.next_goal(timeout=0.05)
        if work is not None:
            return work
        rclpy_mod.spin_once(node, timeout_sec=0.05)
    return None


def _acquire_target(
    work,
    node,
    grounding_command_publisher,
    grounding_listener,
    apriltag_listener,
    result_listener,
    localizer_listener,
    rclpy_mod,
    args,
):
    target = None
    if args.source == "auto":
        work.publish_feedback("arbitrating", "checking for a fresh AprilTag target")
        target = apriltag_listener.wait_for_fresh(
            rclpy_mod,
            args.arbitration_probe_timeout,
            args.arbitration_tag_freshness,
            work.is_cancel_requested,
        )
    if target is None and args.source != "apriltag":
        target = acquire_grounding_target_with_retry(
            node,
            work.target,
            grounding_command_publisher,
            grounding_listener,
            result_listener,
            localizer_listener,
            rclpy_mod,
            args,
            work.is_cancel_requested,
            work.publish_feedback,
        )
    elif target is None:
        work.publish_feedback("waiting_for_apriltag", "waiting for a stable target")
        target = apriltag_listener.wait_for_target(
            rclpy_mod,
            args.timeout,
            work.is_cancel_requested,
        )
    return target


def _process_goal(
    work,
    node,
    grounding_command_publisher,
    grounding_listener,
    apriltag_listener,
    result_listener,
    localizer_listener,
    audio_player,
    rclpy_mod,
    args,
    mode_ready,
):
    work.publish_feedback("accepted", f"target={work.target}")
    source, target_xyz, frame_id, stamp = _acquire_target(
        work,
        node.node,
        grounding_command_publisher,
        grounding_listener,
        apriltag_listener,
        result_listener,
        localizer_listener,
        rclpy_mod,
        args,
    )
    source_label = SOURCE_LABELS[source]
    node.node.get_logger().info(
        f"goal={work.request_id} source={source_label} xyz={target_xyz} stamp={stamp}"
    )
    validate_target(target_xyz, frame_id, source_label)
    if work.is_cancel_requested():
        raise GraspCancelled("grasp goal canceled before IK planning")

    work.publish_feedback("reading_joint_state", "reading current arm position")
    current = node.read_current_arm_pos()
    plan = plan_grasp(
        node,
        current,
        target_xyz,
        args,
        source,
        work.is_cancel_requested,
        work.publish_feedback,
    )
    if not args.execute:
        work.publish_feedback("dry_run_complete", "IK plan succeeded; no motion sent")
        return mode_ready

    if not args.skip_mode_switch and not mode_ready:
        work.publish_feedback("switching_mode", "enabling upper-body remote control")
        request_action_with_retry(
            node, "STAND_DEFAULT", cancel_requested=work.is_cancel_requested
        )
        request_action_with_retry(
            node,
            "UPPERBODY_REMOTE_SPLIT",
            cancel_requested=work.is_cancel_requested,
        )
        mode_ready = True
    execute_grasp(
        node,
        current,
        plan,
        args,
        work.target,
        work.is_cancel_requested,
        work.publish_feedback,
    )
    try:
        work.publish_feedback("announcing", "playing completion audio")
        audio_player.play(rclpy_mod, work.target)
    except RuntimeError as error:
        node.node.get_logger().error(f"抓取完成，但音频播报失败：{error}")
    return mode_ready


def run(args):
    validate_parameters(args)
    rclpy_mod, upper_body_msg, joint_srv, action_srv = _require_ros2_imports()
    if not rclpy_mod.ok():
        rclpy_mod.init()

    node = X2HardwareNode(
        rclpy_mod,
        upper_body_msg,
        joint_srv,
        action_srv,
        make_hardware_args(args),
    )
    grounding_listener = (
        TargetListener(node.node, "grounding", args.vector_topic)
        if args.source in {"grounding", "auto"}
        else None
    )
    apriltag_listener = (
        TargetListener(node.node, "apriltag", args.apriltag_topic)
        if args.source in {"apriltag", "auto"}
        else None
    )
    grounding_command_publisher = node.node.create_publisher(
        GroundingCommand, args.grounding_command_topic, 10
    )
    result_listener = (
        GroundingResultListener(node.node, args.grounding_result_topic)
        if args.source in {"grounding", "auto"}
        else None
    )
    localizer_listener = (
        LocalizerStatusListener(node.node, args.localizer_status_topic)
        if args.source in {"grounding", "auto"}
        else None
    )
    audio_player = GraspAudioPlayer(node.node, args)
    if args.execute:
        audio_player.recover_stale_focus(rclpy_mod)
    action_bridge = GraspActionBridge(
        rclpy_mod, args.action_name, args.target_catalog
    )
    node.node.get_logger().info(
        f"Grasp Action ready: {args.action_name}; mode={args.source}; "
        f"execute={args.execute}; targets={','.join(args.target_catalog.names)}"
    )

    mode_ready = False
    try:
        while rclpy_mod.ok():
            work = _wait_for_goal(action_bridge, node.node, rclpy_mod)
            if work is None:
                break
            try:
                mode_ready = _process_goal(
                    work,
                    node,
                    grounding_command_publisher,
                    grounding_listener,
                    apriltag_listener,
                    result_listener,
                    localizer_listener,
                    audio_player,
                    rclpy_mod,
                    args,
                    mode_ready,
                )
                if work.is_cancel_requested():
                    raise GraspCancelled("grasp goal canceled before completion")
                work.finish(success=True)
            except GraspCancelled as error:
                node.node.get_logger().warning(str(error))
                work.finish(success=False, error=str(error), canceled=True)
            except (RuntimeError, ValueError) as error:
                node.node.get_logger().error(
                    f"goal={work.request_id} target={work.target} failed: {error}"
                )
                work.finish(success=False, error=str(error))
            except Exception as error:  # Keep the Action server available.
                node.node.get_logger().error(
                    f"goal={work.request_id} unexpected failure: "
                    f"{type(error).__name__}: {error}"
                )
                work.finish(
                    success=False,
                    error=f"internal error: {type(error).__name__}: {error}",
                )
            finally:
                action_bridge.goal_done()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        action_bridge.shutdown()
        if args.execute and rclpy_mod.ok():
            audio_player.release(rclpy_mod, raise_on_failure=False)
        node.shutdown()
        if rclpy_mod.ok():
            rclpy_mod.shutdown()


def main(argv=None):
    raw = sys.argv[1:] if argv is None else argv
    if "--ros-args" in raw:
        rclpy.init()
        probe = rclpy.create_node("x2_grasp_param_probe")
        try:
            args = parse_ros_params_into_args(probe)
        finally:
            probe.destroy_node()
    else:
        args = build_parser().parse_args(raw)
    run(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
