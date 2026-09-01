"""Client boundary for the optional target-only C++ command publisher."""

from __future__ import annotations

import math
from pathlib import Path


COMMAND_ACTION = "/x2_grasp/execute_command"


def native_command_publisher_installed() -> bool:
    try:
        from ament_index_python.packages import get_package_prefix

        executable = (
            Path(get_package_prefix("x2_grasp"))
            / "lib"
            / "x2_grasp"
            / "x2_command_publisher"
        )
        return executable.is_file()
    except (ImportError, LookupError):
        return False


class CppCommandClient:
    def __init__(self, node, action_name: str = COMMAND_ACTION):
        from rclpy.action import ActionClient
        from x2_grasp.action import ExecuteCommand

        self.node = node
        self.action_type = ExecuteCommand
        self.client = ActionClient(node, ExecuteCommand, action_name)
        self.action_name = action_name

    def wait_for_server(self, timeout_sec: float) -> bool:
        return bool(self.client.wait_for_server(timeout_sec=timeout_sec))

    def execute_arm(
        self,
        start,
        goal,
        duration,
        gripper_positions,
        cancel_requested=lambda: False,
    ):
        message = self.action_type.Goal()
        message.kind = self.action_type.Goal.ARM_TRAJECTORY
        message.start_arm_pos = [float(value) for value in start]
        message.goal_arm_pos = [float(value) for value in goal]
        message.duration = float(duration)
        message.left_hand_position = float(gripper_positions[0])
        message.right_hand_position = float(gripper_positions[1])
        return self._execute(message, cancel_requested)

    def execute_hand(
        self,
        hand,
        left_position,
        right_position,
        duration,
        cancel_requested=lambda: False,
    ):
        message = self.action_type.Goal()
        message.kind = self.action_type.Goal.HAND_COMMAND
        message.hand = str(hand)
        message.left_hand_position = float(left_position or 0.0)
        message.right_hand_position = float(right_position or 0.0)
        message.duration = float(duration)
        return self._execute(message, cancel_requested)

    def _execute(self, message, cancel_requested):
        import rclpy

        send_future = self.client.send_goal_async(message)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.01)
        if not send_future.done():
            raise RuntimeError("ROS stopped before C++ command goal was accepted")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("C++ command publisher rejected the command stream")

        result_future = goal_handle.get_result_async()
        cancel_sent = False
        while rclpy.ok() and not result_future.done():
            if cancel_requested() and not cancel_sent:
                goal_handle.cancel_goal_async()
                cancel_sent = True
            rclpy.spin_once(self.node, timeout_sec=0.01)
        if not result_future.done():
            raise RuntimeError("ROS stopped during C++ command execution")

        result = result_future.result().result
        metrics = {
            "frames_requested": int(result.frames_requested),
            "frames_published": int(result.frames_published),
            "deadline_misses": int(result.deadline_misses),
            "max_lateness_ms": float(result.max_lateness_ns) / 1_000_000.0,
            "elapsed_seconds": float(result.elapsed_seconds),
            "hold_published": bool(result.hold_published),
        }
        if bool(result.canceled) or cancel_sent:
            raise InterruptedError(result.error or "command stream canceled")
        if not bool(result.success):
            raise RuntimeError(result.error or "C++ command stream failed")
        if not all(
            math.isfinite(value)
            for value in (metrics["max_lateness_ms"], metrics["elapsed_seconds"])
        ):
            raise RuntimeError("C++ command publisher returned non-finite metrics")
        return metrics

    def shutdown(self):
        self.client.destroy()
