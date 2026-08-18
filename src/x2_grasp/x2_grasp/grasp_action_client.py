"""Command-line client for submitting and optionally canceling a grasp goal."""

from __future__ import annotations

import argparse
import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from x2_grasp.action import Grasp


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Send an X2 grasp Action goal")
    parser.add_argument("target", help="target name configured on the Action server")
    parser.add_argument("--action-name", default="/x2_grasp/grasp")
    parser.add_argument(
        "--cancel-after",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="request cancellation after this many seconds; zero disables it",
    )
    return parser


def main(argv=None) -> int:
    raw = sys.argv if argv is None else [sys.argv[0], *argv]
    args = _parser().parse_args(remove_ros_args(args=raw)[1:])
    if args.cancel_after < 0.0:
        _parser().error("--cancel-after must be non-negative")

    rclpy.init(args=raw)
    node = Node("x2_grasp_action_client")
    client = ActionClient(node, Grasp, args.action_name)
    goal_handle = None
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error(f"Action server unavailable: {args.action_name}")
            return 1

        goal = Grasp.Goal()
        goal.target = args.target

        def on_feedback(message) -> None:
            feedback = message.feedback
            print(f"feedback: stage={feedback.stage} detail={feedback.detail}")

        send_future = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("goal rejected")
            return 2
        print("goal accepted")

        if args.cancel_after > 0.0:
            timer = None

            def cancel_goal() -> None:
                timer.cancel()
                print("requesting cancellation")
                goal_handle.cancel_goal_async()

            timer = node.create_timer(args.cancel_after, cancel_goal)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        response = result_future.result()
        if response is None:
            print("result unavailable")
            return 1
        result = response.result
        print(
            f"result: status={response.status} success={result.success} "
            f"target={result.target} error={result.error}"
        )
        if response.status == GoalStatus.STATUS_CANCELED:
            return 3
        return 0 if response.status == GoalStatus.STATUS_SUCCEEDED else 1
    except KeyboardInterrupt:
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
