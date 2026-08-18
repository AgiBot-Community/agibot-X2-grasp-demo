#!/usr/bin/env python3
"""Minimal ROS 2 Action client for the X2 grasp demo."""

from __future__ import annotations

import argparse
import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from x2_grasp.action import Grasp


STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send an X2 grasp goal")
    parser.add_argument("target", help="target name configured on the Action server")
    parser.add_argument("--action-name", default="/x2_grasp/grasp")
    parser.add_argument(
        "--cancel-after",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="cancel the accepted goal after this delay",
    )
    args = parser.parse_args(remove_ros_args(args=argv)[1:])
    if args.cancel_after < 0.0:
        parser.error("--cancel-after must be non-negative")
    return args


def main() -> int:
    args = parse_args(sys.argv)
    rclpy.init(args=sys.argv)
    node = Node("x2_grasp_example_client")
    client = ActionClient(node, Grasp, args.action_name)
    goal_handle = None

    def feedback_callback(message) -> None:
        feedback = message.feedback
        print(f"[feedback] {feedback.stage}: {feedback.detail}")

    try:
        print(f"Waiting for {args.action_name} ...")
        if not client.wait_for_server(timeout_sec=10.0):
            print("Action server was not available within 10 seconds.")
            return 1

        goal = Grasp.Goal()
        goal.target = args.target
        send_future = client.send_goal_async(
            goal,
            feedback_callback=feedback_callback,
        )
        rclpy.spin_until_future_complete(node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("Goal rejected. Check the target or an already active goal.")
            return 2
        print(f"Goal accepted: target={args.target}")

        if args.cancel_after > 0.0:
            timer = None

            def request_cancel() -> None:
                timer.cancel()
                print("Requesting goal cancellation ...")
                goal_handle.cancel_goal_async()

            timer = node.create_timer(args.cancel_after, request_cancel)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            print("No result was returned.")
            return 1

        result = wrapped_result.result
        status = STATUS_NAMES.get(wrapped_result.status, str(wrapped_result.status))
        print(
            f"[result] status={status} success={result.success} "
            f"target={result.target} error={result.error!r}"
        )
        return 0 if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED else 1
    except KeyboardInterrupt:
        print("Interrupted; requesting cancellation ...")
        if goal_handle is not None and goal_handle.accepted:
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=2.0)
        return 130
    finally:
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
