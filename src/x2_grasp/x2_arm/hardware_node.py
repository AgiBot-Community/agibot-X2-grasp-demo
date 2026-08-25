from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Iterable

from .config import ArmSide, X2IKConfig
from .command_client import CppCommandClient, native_command_publisher_installed
from .standalone_hand_api import StandaloneHandAPI
from .native_solver import create_ik_solver
from .trajectory import interpolate_arm_pos


def _require_ros2_imports():
    try:
        import rclpy
        from aimdk_msgs.msg import UpperBodyCommandArray
        from aimdk_msgs.srv import GetAllJointState, SetMcAction
    except ImportError as exc:
        raise SystemExit(
            "ROS2 adapter dependencies are not importable.\n"
            "Confirm that rclpy, aimdk_msgs, and the new UpperBodyCommandArray message "
            "are available in the X2 SDK environment.\n"
            f"Original error: {exc}"
        ) from exc
    return rclpy, UpperBodyCommandArray, GetAllJointState, SetMcAction


def joint_states_to_arm_pos(joint_states: Iterable[object]) -> list[float]:
    by_name = {
        state.name: state.position
        for state in joint_states
        if hasattr(state, "name") and hasattr(state, "position")
    }
    from .config import ARM_POS_ORDER

    missing = [name for name in ARM_POS_ORDER if name not in by_name]
    if missing:
        raise ValueError(f"joint state response is missing arm joints: {missing}")
    ordered = [float(by_name[name]) for name in ARM_POS_ORDER]
    if not all(math.isfinite(value) for value in ordered):
        raise ValueError("joint state response contains non-finite arm positions")
    return ordered


class X2HardwareNode:
    def __init__(self, rclpy, UpperBodyCommandArray, GetAllJointState, SetMcAction, args):
        self.rclpy = rclpy
        self.UpperBodyCommandArray = UpperBodyCommandArray
        self.GetAllJointState = GetAllJointState
        self.SetMcAction = SetMcAction

        from rclpy.node import Node

        class _Node(Node):
            pass

        self.node = _Node("x2_upper_body_ik_node")
        default_cfg = X2IKConfig.default_omnipicker()
        cfg = X2IKConfig(
            urdf_path=args.urdf if args.urdf else default_cfg.urdf_path,
            joint_margin=getattr(args, "joint_margin", default_cfg.joint_margin),
        )
        self.solver = create_ik_solver(cfg, getattr(args, "ik_backend", "auto"))
        self.command_backend = getattr(args, "command_backend", "auto")
        self.command_client: CppCommandClient | None = None
        native_installed = native_command_publisher_installed()
        if self.command_backend == "native" and not native_installed:
            raise RuntimeError(
                "native command backend requested, but x2_command_publisher was not built"
            )
        if self.command_backend != "python" and native_installed:
            candidate = CppCommandClient(self.node)
            if candidate.wait_for_server(timeout_sec=5.0):
                self.command_client = candidate
                self.command_backend = "native"
            else:
                candidate.shutdown()
                if self.command_backend == "native":
                    raise RuntimeError("C++ command publisher Action is not available")
        if self.command_client is None:
            self.command_backend = "python"
        self.node.get_logger().info(
            f"50 Hz command backend: {self.command_backend}"
        )
        self.publisher = (
            None
            if self.command_client is not None
            else self.node.create_publisher(UpperBodyCommandArray, args.command_topic, 10)
        )
        self.joint_client = self.node.create_client(GetAllJointState, args.joint_state_service)
        self.action_client = self.node.create_client(SetMcAction, args.action_service)
        self.source = args.source
        self.frame_id = args.frame_id
        self.hand_command_topic = getattr(args, "hand_command_topic", None)
        self.hand_publish_hz = getattr(args, "hand_publish_hz", 50.0)
        self.hand_api: StandaloneHandAPI | None = None
        self.sequence = 0

    def _ensure_hand_api(self) -> StandaloneHandAPI:
        if self.hand_api is None:
            kwargs = {"publish_hz": self.hand_publish_hz}
            if self.hand_command_topic is not None:
                kwargs["command_topic"] = self.hand_command_topic
            self.hand_api = StandaloneHandAPI(self.node, **kwargs)
        return self.hand_api

    def set_gripper_position(
        self,
        hand: str,
        position: float,
        seconds: float = 2.0,
        *,
        cancel_requested=lambda: False,
    ) -> dict:
        """Set a left, right, or both grippers to a 0.0-1.0 position."""

        if self.command_client is not None:
            return self.command_client.execute_hand(
                hand,
                position if hand in {"left", "both"} else None,
                position if hand in {"right", "both"} else None,
                seconds,
                cancel_requested,
            )
        return self._ensure_hand_api().set_position(
            hand, position, seconds=seconds, cancel_requested=cancel_requested
        )

    def open_gripper(
        self, hand: str = "both", seconds: float = 2.0, *, cancel_requested=lambda: False
    ) -> dict:
        """Open one gripper or both grippers."""

        return self.set_gripper_position(
            hand, 1.0, seconds=seconds, cancel_requested=cancel_requested
        )

    def close_gripper(
        self, hand: str = "both", seconds: float = 2.0, *, cancel_requested=lambda: False
    ) -> dict:
        """Close one gripper or both grippers."""

        return self.set_gripper_position(
            hand, 0.0, seconds=seconds, cancel_requested=cancel_requested
        )

    def switch_action(self, action_desc: str) -> None:
        if not self.action_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("SetMcAction service not available")
        req = self.SetMcAction.Request()
        req.source = self.source
        req.command.action_desc = action_desc
        future = self.action_client.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if future.result() is None:
            raise RuntimeError(f"SetMcAction failed: {action_desc}")
        self.node.get_logger().info(f"SetMcAction requested: {action_desc}")

    def read_current_arm_pos(self) -> list[float]:
        if not self.joint_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("GetAllJointState service not available")
        req = self.GetAllJointState.Request()
        future = self.joint_client.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if future.result() is None:
            raise RuntimeError("GetAllJointState request failed")
        return joint_states_to_arm_pos(future.result().arm_joints)

    def make_msg(self, arm_pos: list[float], hand_open: tuple[float, float]):
        msg = self.UpperBodyCommandArray()
        now = self.node.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.header.sequence = self.sequence
        msg.source = self.source
        msg.hand_sub_mode = 1
        msg.head_pos = [0.0, 0.0]
        msg.arm_pos = [float(v) for v in arm_pos]
        msg.hand_pos = [float(hand_open[0]), float(hand_open[1])]
        self.sequence += 1
        return msg

    def publish_trajectory(
        self,
        start: list[float],
        goal: list[float],
        duration: float,
        *,
        cancel_requested=lambda: False,
    ) -> None:
        safe_start = self.solver.clip_arm_pos(start)
        safe_goal = self.solver.clip_arm_pos(goal)
        if any(abs(before - after) > 1e-9 for before, after in zip(start, safe_start)):
            self.node.get_logger().warning(
                "Current arm state was outside the operational IK limits; "
                "trajectory start was clamped before publishing"
            )
        if any(abs(before - after) > 1e-9 for before, after in zip(goal, safe_goal)):
            self.node.get_logger().warning(
                "IK goal was outside the operational IK limits; "
                "trajectory goal was clamped before publishing"
            )
        if self.command_client is not None:
            metrics = self.command_client.execute_arm(
                safe_start, safe_goal, duration, cancel_requested
            )
            self.node.get_logger().info(
                "C++ command stream: "
                f"frames={metrics['frames_published']}/{metrics['frames_requested']} "
                f"misses={metrics['deadline_misses']} "
                f"max_lateness_ms={metrics['max_lateness_ms']:.3f}"
            )
            return
        waypoints = interpolate_arm_pos(
            safe_start, safe_goal, duration=duration, rate_hz=50.0
        )
        for index, wp in enumerate(waypoints):
            if cancel_requested():
                raise InterruptedError("trajectory canceled before completion")
            self.publisher.publish(self.make_msg(wp.arm_pos, (1.0, 1.0)))
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
            # Use wall-clock sleep here; create_rate() can stall in this
            # single-threaded publish loop before additional waypoints are sent.
            if index + 1 < len(waypoints):
                time.sleep(wp.duration)

    def run_once(
        self,
        side: ArmSide,
        target_xyz: list[float],
        dry_run: bool,
        duration: float,
        target_rpy: list[float] | None,
        keep_current_rpy: bool,
        orientation_weight: float,
        orientation_eps: float,
    ) -> None:
        current = self.read_current_arm_pos()
        if target_rpy is not None and keep_current_rpy:
            raise ValueError("Use either target_rpy or keep_current_rpy, not both")
        if keep_current_rpy:
            target_rpy = self.solver.fk_rpy(side, current)

        if target_rpy is None:
            result = self.solver.solve_position(
                side=side,
                target_xyz=target_xyz,
                current_arm_pos=current,
            )
        else:
            result = self.solver.solve_pose(
                side=side,
                target_xyz=target_xyz,
                target_rpy=target_rpy,
                current_arm_pos=current,
                orientation_weight=orientation_weight,
                orientation_eps=orientation_eps,
            )
        self.node.get_logger().info(
            f"IK success={result.success} error={result.error_norm:.6f} arm_pos={result.arm_pos}"
        )
        if dry_run:
            return
        if not result.success:
            raise RuntimeError(f"IK failed: {result.message}, err={result.error_norm}")
        self.publish_trajectory(current, result.arm_pos, duration)

    def run_6d(
        self,
        side: ArmSide,
        target_6d: list[float],
        dry_run: bool,
        duration: float,
        orientation_weight: float,
        orientation_eps: float,
    ) -> None:
        """Solve and optionally publish a six-dimensional pose target."""

        current = self.read_current_arm_pos()
        result = self.solver.solve_6d(
            side=side,
            target_6d=target_6d,
            current_arm_pos=current,
            orientation_weight=orientation_weight,
            orientation_eps=orientation_eps,
        )
        self.node.get_logger().info(
            f"6D IK success={result.success} error={result.error_norm:.6f} "
            f"arm_pos={result.arm_pos}"
        )
        if dry_run:
            return
        if not result.success:
            raise RuntimeError(f"IK failed: {result.message}, err={result.error_norm}")
        self.publish_trajectory(current, result.arm_pos, duration)

    def shutdown(self) -> None:
        if self.command_client is not None:
            self.command_client.shutdown()
        if self.hand_api is not None:
            self.hand_api.shutdown()
        self.node.destroy_node()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="X2 upper-body IK hardware adapter")
    parser.add_argument("--urdf", type=Path)
    parser.add_argument("--side", choices=[side.value for side in ArmSide], default="right")
    parser.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument(
        "--target-6d",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Target [x, y, z, roll, pitch, yaw]; position is meters and RPY is radians",
    )
    parser.add_argument("--target-rpy", nargs=3, type=float, metavar=("ROLL", "PITCH", "YAW"))
    parser.add_argument("--keep-current-rpy", action="store_true")
    parser.add_argument("--orientation-weight", type=float, default=1.0)
    parser.add_argument("--orientation-eps", type=float, default=1e-3)
    parser.add_argument(
        "--joint-margin",
        type=float,
        default=0.02,
        help="Keep arm joints this far (rad) inside URDF mechanical limits",
    )
    parser.add_argument(
        "--ik-backend",
        choices=("auto", "native", "python"),
        default="auto",
        help="IK implementation (auto prefers the native extension)",
    )
    parser.add_argument(
        "--command-backend",
        choices=("auto", "native", "python"),
        default="auto",
        help="50 Hz command publisher (auto prefers the C++ node)",
    )
    parser.add_argument("--dry-run", action="store_true", help="Compute IK but do not publish motion")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--command-topic", default="/mc/upper_body_command")
    parser.add_argument("--joint-state-service", default="/aimdk_5Fmsgs/srv/GetAllJointState")
    parser.add_argument("--action-service", default="/aimdk_5Fmsgs/srv/SetMcAction")
    parser.add_argument("--source", default="x2_arm")
    parser.add_argument("--frame-id", default="mc_upper_body")
    parser.add_argument("--hand-command-topic", default=None)
    parser.add_argument("--hand-publish-hz", type=float, default=50.0)
    parser.add_argument(
        "--gripper",
        choices=["none", "open", "close"],
        default="none",
        help="Open or close a gripper without changing the IK target",
    )
    parser.add_argument(
        "--gripper-position",
        type=float,
        metavar="POSITION",
        help="Set gripper position from 0.0 (closed) to 1.0 (open)",
    )
    parser.add_argument(
        "--gripper-hand",
        choices=["left", "right", "both"],
        default="both",
    )
    parser.add_argument("--gripper-seconds", type=float, default=2.0)
    parser.add_argument("--skip-mode-switch", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    rclpy, UpperBodyCommandArray, GetAllJointState, SetMcAction = _require_ros2_imports()
    rclpy.init()
    node = X2HardwareNode(rclpy, UpperBodyCommandArray, GetAllJointState, SetMcAction, args)
    try:
        has_arm_target = args.target is not None or args.target_6d is not None
        has_gripper_target = args.gripper != "none" or args.gripper_position is not None
        if args.target is not None and args.target_6d is not None:
            raise SystemExit("Use either --target or --target-6d, not both")
        if args.target_6d is not None and (
            args.target_rpy is not None or args.keep_current_rpy
        ):
            raise SystemExit("--target-6d already includes orientation")
        if args.gripper != "none" and args.gripper_position is not None:
            raise SystemExit("Use either --gripper or --gripper-position, not both")
        if not has_arm_target and not has_gripper_target:
            raise SystemExit(
                "Provide --target, --target-6d, --gripper, or --gripper-position"
            )

        if not args.skip_mode_switch and not args.dry_run and (
            has_arm_target or has_gripper_target
        ):
            node.switch_action("STAND_DEFAULT")
            node.switch_action("UPPERBODY_REMOTE_SPLIT")
        if args.target_6d is not None:
            node.run_6d(
                ArmSide(args.side),
                [float(v) for v in args.target_6d],
                args.dry_run,
                args.duration,
                args.orientation_weight,
                args.orientation_eps,
            )
        elif args.target is not None:
            node.run_once(
                ArmSide(args.side),
                [float(v) for v in args.target],
                args.dry_run,
                args.duration,
                args.target_rpy,
                args.keep_current_rpy,
                args.orientation_weight,
                args.orientation_eps,
            )

        if has_gripper_target:
            if args.dry_run:
                node.node.get_logger().info("dry-run: gripper command skipped")
            elif args.gripper == "open":
                node.open_gripper(args.gripper_hand, seconds=args.gripper_seconds)
            elif args.gripper == "close":
                node.close_gripper(args.gripper_hand, seconds=args.gripper_seconds)
            else:
                node.set_gripper_position(
                    args.gripper_hand,
                    args.gripper_position,
                    seconds=args.gripper_seconds,
                )
    finally:
        node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
