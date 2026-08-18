from __future__ import annotations

import math
from typing import Iterable

from .config import ARM_POS_ORDER, ArmSide, X2IKConfig, default_urdf_path
from .solver import IKResult, X2ArmIKSolver


DEFAULT_TOPICS = {
    ArmSide.LEFT: {
        "target": "/x2_ik/left/target_pose",
        "target_6d": "/x2_arm/left/target_6d",
        "solution": "/x2_ik/left/solution",
    },
    ArmSide.RIGHT: {
        "target": "/x2_ik/right/target_pose",
        "target_6d": "/x2_arm/right/target_6d",
        "solution": "/x2_ik/right/solution",
    },
}
DEFAULT_JOINT_STATES_TOPIC = "/joint_states"
DEFAULT_STATUS_TOPIC = "/x2_ik/status"


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> list[float]:
    """Convert a normalized or non-normalized quaternion to roll/pitch/yaw."""
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise ValueError("target orientation quaternion must contain finite values")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("target orientation quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [roll, pitch, yaw]


def ordered_arm_pos(
    names: Iterable[str], positions: Iterable[float]
) -> list[float] | None:
    """Return the SDK's 14-joint order, or None when state is incomplete."""
    by_name = {name: float(position) for name, position in zip(names, positions)}
    if any(name not in by_name for name in ARM_POS_ORDER):
        return None
    ordered = [by_name[name] for name in ARM_POS_ORDER]
    return ordered if all(math.isfinite(value) for value in ordered) else None


def _require_ros2_imports():
    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
    except ImportError as exc:
        raise SystemExit(
            "ROS 2 topic node dependencies are not importable. "
            "Source the ROS 2 workspace before running this node.\n"
            f"Original error: {exc}"
        ) from exc
    return (
        rclpy,
        Node,
        PoseStamped,
        JointState,
        Float64MultiArray,
        DiagnosticArray,
        DiagnosticStatus,
        KeyValue,
    )


def create_node():
    (
        _rclpy,
        Node,
        PoseStamped,
        JointState,
        Float64MultiArray,
        DiagnosticArray,
        DiagnosticStatus,
        KeyValue,
    ) = _require_ros2_imports()

    class X2IKSolverNode(Node):
        def __init__(self) -> None:
            super().__init__("x2_ik_solver_node")
            self.declare_parameter("base_frame", "base_link")
            self.declare_parameter("use_target_orientation", False)
            self.declare_parameter("require_current_state", True)
            self.declare_parameter("orientation_weight", 1.0)
            self.declare_parameter("orientation_eps", 1e-3)
            self.declare_parameter("joint_margin", 0.02)
            self.declare_parameter("joint_states_topic", DEFAULT_JOINT_STATES_TOPIC)
            self.declare_parameter("status_topic", DEFAULT_STATUS_TOPIC)
            for side in ArmSide:
                self.declare_parameter(
                    f"{side.value}_target_topic", DEFAULT_TOPICS[side]["target"]
                )
                self.declare_parameter(
                    f"{side.value}_target_6d_topic", DEFAULT_TOPICS[side]["target_6d"]
                )
                self.declare_parameter(
                    f"{side.value}_solution_topic", DEFAULT_TOPICS[side]["solution"]
                )

            self.solver = X2ArmIKSolver(
                X2IKConfig(
                    urdf_path=default_urdf_path(),
                    joint_margin=float(self._parameter("joint_margin")),
                )
            )
            self.current_arm_pos: list[float] | None = None
            self.status_publisher = self.create_publisher(
                DiagnosticArray, self._parameter("status_topic"), 10
            )
            self.solution_publishers = {}
            self.target_subscriptions = []
            self.target_6d_subscriptions = []
            for side in ArmSide:
                self.solution_publishers[side] = self.create_publisher(
                    JointState, self._parameter(f"{side.value}_solution_topic"), 10
                )
                self.target_subscriptions.append(
                    self.create_subscription(
                        PoseStamped,
                        self._parameter(f"{side.value}_target_topic"),
                        lambda msg, selected_side=side: self._on_target(
                            selected_side, msg
                        ),
                        10,
                    )
                )
                self.target_6d_subscriptions.append(
                    self.create_subscription(
                        Float64MultiArray,
                        self._parameter(f"{side.value}_target_6d_topic"),
                        lambda msg, selected_side=side: self._on_target_6d(
                            selected_side, msg
                        ),
                        10,
                    )
                )
            self.joint_state_subscription = self.create_subscription(
                JointState,
                self._parameter("joint_states_topic"),
                self._on_joint_state,
                10,
            )
            self.get_logger().info(
                "X2 IK pose and 6D-vector topics ready; solutions are published only "
                "and do not command hardware"
            )

        def _parameter(self, name: str):
            return self.get_parameter(name).value

        def _on_joint_state(self, msg) -> None:
            arm_pos = ordered_arm_pos(msg.name, msg.position)
            if arm_pos is None:
                self.get_logger().warning(
                    "JointState does not contain all 14 X2 arm joints; state ignored"
                )
                return
            self.current_arm_pos = arm_pos

        def _on_target(self, side: ArmSide, msg) -> None:
            base_frame = str(self._parameter("base_frame"))
            if msg.header.frame_id != base_frame:
                self._publish_status(
                    side,
                    None,
                    f"target frame must be {base_frame!r}; TF conversion is not performed",
                )
                return
            current = self.current_arm_pos
            if current is None:
                if bool(self._parameter("require_current_state")):
                    self._publish_status(side, None, "waiting for complete current joint state")
                    return
                current = self.solver.ready_arm_pos()

            target_xyz = [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ]
            try:
                if bool(self._parameter("use_target_orientation")):
                    q = msg.pose.orientation
                    target_rpy = quaternion_to_rpy(q.x, q.y, q.z, q.w)
                    result = self.solver.solve_pose(
                        side,
                        target_xyz,
                        target_rpy,
                        current,
                        orientation_weight=float(self._parameter("orientation_weight")),
                        orientation_eps=float(self._parameter("orientation_eps")),
                    )
                else:
                    result = self.solver.solve_position(side, target_xyz, current)
            except (TypeError, ValueError) as exc:
                self._publish_status(side, None, str(exc))
                return

            self._publish_status(side, result, result.message)
            if not result.success:
                return
            self._publish_solution(side, result, base_frame, JointState)

        def _on_target_6d(self, side: ArmSide, msg) -> None:
            """Solve ``[x, y, z, roll, pitch, yaw]`` from a Float64MultiArray."""

            base_frame = str(self._parameter("base_frame"))
            current = self.current_arm_pos
            if current is None:
                if bool(self._parameter("require_current_state")):
                    self._publish_status(side, None, "waiting for complete current joint state")
                    return
                current = self.solver.ready_arm_pos()

            try:
                result = self.solver.solve_6d(
                    side,
                    [float(value) for value in msg.data],
                    current,
                    orientation_weight=float(self._parameter("orientation_weight")),
                    orientation_eps=float(self._parameter("orientation_eps")),
                )
            except (TypeError, ValueError) as exc:
                self._publish_status(side, None, str(exc))
                return

            self._publish_status(side, result, result.message)
            if result.success:
                self._publish_solution(side, result, base_frame, JointState)

        def _publish_solution(self, side, result, base_frame, joint_state_type) -> None:
            solution = joint_state_type()
            solution.header.stamp = self.get_clock().now().to_msg()
            solution.header.frame_id = base_frame
            solution.name = list(ARM_POS_ORDER)
            solution.position = [float(value) for value in result.arm_pos]
            self.solution_publishers[side].publish(solution)

        def _publish_status(
            self, side: ArmSide, result: IKResult | None, message: str
        ) -> None:
            status = DiagnosticStatus()
            status.name = f"x2_ik/{side.value}"
            status.hardware_id = "x2_arm"
            status.level = (
                DiagnosticStatus.OK
                if result is not None and result.success
                else DiagnosticStatus.WARN
            )
            status.message = message
            values = [("side", side.value)]
            if result is not None:
                values.extend(
                    [
                        ("success", str(result.success).lower()),
                        ("error_norm", f"{result.error_norm:.9g}"),
                        ("iterations", str(result.iterations)),
                        ("ee_frame", result.ee_frame),
                    ]
                )
            status.values = [KeyValue(key=key, value=value) for key, value in values]
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            array.status = [status]
            self.status_publisher.publish(array)

    return X2IKSolverNode()


def main() -> None:
    rclpy, *_ = _require_ros2_imports()
    rclpy.init()
    node = create_node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
