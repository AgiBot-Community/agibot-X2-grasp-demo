"""Coordinate AprilTag and grounding results for one grasp goal."""

from __future__ import annotations

from collections import deque
import time
import uuid

from geometry_msgs.msg import Vector3Stamped
from x2_grasp.msg import GroundingCommand, GroundingResult, PerceptionStatus

from x2_common import LatestValue

from .grasp_constants import APRILTAG_VECTOR_TOPIC, GROUNDING_VECTOR_TOPIC
from .grasp_errors import GraspCancelled


class TargetListener:
    """Listen to one selected coordinate source."""

    SOURCE_LABELS = {"grounding": "API画框", "apriltag": "ARTag"}

    def __init__(self, node, source, topic):
        self.node = node
        self.source = source
        self.topic = topic
        self._latest = LatestValue()
        self.subscription = node.create_subscription(
            Vector3Stamped, topic, self._on_vector, 10
        )

    def _on_vector(self, message):
        self._latest.set(message)

    def clear(self):
        self._latest.clear()

    def _format_message(self, message):
        if message is None:
            return None
        xyz = [
            float(message.vector.x),
            float(message.vector.y),
            float(message.vector.z),
        ]
        stamp = f"{message.header.stamp.sec}.{message.header.stamp.nanosec:09d}"
        return self.source, xyz, message.header.frame_id, stamp

    def take_fresh(self, max_age_sec):
        return self._format_message(
            self._latest.take(max_age_seconds=max_age_sec)
        )

    def wait_for_fresh(
        self, rclpy_mod, timeout_sec, max_age_sec, cancel_requested=lambda: False
    ):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if cancel_requested():
                raise GraspCancelled("grasp goal canceled during AprilTag arbitration")
            target = self.take_fresh(max_age_sec)
            if target is not None:
                return target
            rclpy_mod.spin_once(self.node, timeout_sec=0.05)
        return self.take_fresh(max_age_sec)

    def wait_for_target(
        self, rclpy_mod, timeout_sec, cancel_requested=lambda: False
    ):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if cancel_requested():
                raise GraspCancelled("grasp goal canceled while waiting for AprilTag")
            target = self._format_message(self._latest.take())
            if target is not None:
                return target
            rclpy_mod.spin_once(self.node, timeout_sec=0.1)
        raise RuntimeError(
            f"在 {timeout_sec:.1f}s 内没有收到{self.SOURCE_LABELS[self.source]}坐标"
            f"（监听：{self.topic}）"
        )

    def take_for_stamp(self, expected_stamp):
        message = self._latest.take()
        if message is None:
            return None
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stamp != expected_stamp:
            return None
        return self._format_message(message)


class GroundingResultListener:
    def __init__(self, node, topic):
        self.pending_results = deque(maxlen=32)
        self.subscription = node.create_subscription(
            GroundingResult, topic, self._on_result, 10
        )

    def _on_result(self, message):
        payload = {
            "request_id": message.request_id,
            "success": message.success,
            "error": message.error,
            "boxes": [None] * int(message.box_count),
            "image_stamp": {
                "sec": message.image_stamp.sec,
                "nanosec": message.image_stamp.nanosec,
            },
        }
        if not payload["success"]:
            payload["error"] = f"API 画框失败：{payload['error'] or '未知错误'}"
        elif not payload["boxes"]:
            payload["success"] = False
            payload["error"] = "API 调用成功，但没有识别到目标框"
        self.pending_results.append(payload)

    def clear(self):
        self.pending_results.clear()

    def pop_result(self):
        return self.pending_results.popleft() if self.pending_results else None


class LocalizerStatusListener:
    def __init__(self, node, topic):
        self.pending_statuses = deque(maxlen=32)
        self.subscription = node.create_subscription(
            PerceptionStatus, topic, self._on_status, 10
        )

    def _on_status(self, message):
        self.pending_statuses.append(
            {
                "success": message.success,
                "error": message.error,
                "image_stamp": {
                    "sec": message.image_stamp.sec,
                    "nanosec": message.image_stamp.nanosec,
                },
            }
        )

    def clear(self):
        self.pending_statuses.clear()

    def pop_status(self):
        return self.pending_statuses.popleft() if self.pending_statuses else None


def _payload_stamp(payload):
    stamp = payload.get("image_stamp") or payload.get("stamp") or {}
    try:
        return int(stamp["sec"]), int(stamp["nanosec"])
    except (KeyError, TypeError, ValueError):
        return None


def acquire_grounding_target_with_retry(
    node,
    command,
    publisher,
    target_listener,
    result_listener,
    localizer_listener,
    rclpy_mod,
    args,
    cancel_requested=lambda: False,
    feedback=lambda _stage, _detail="": None,
):
    """Acquire one valid 3D target, retrying only the perception pipeline."""
    failures = []
    for attempt in range(1, args.grounding_retry_attempts + 1):
        if cancel_requested():
            raise GraspCancelled("grasp goal canceled before visual grounding")
        target_listener.clear()
        result_listener.clear()
        localizer_listener.clear()

        message = GroundingCommand()
        request_id = uuid.uuid4().hex
        message.created_at = node.get_clock().now().to_msg()
        message.target = command
        message.request_id = request_id
        publisher.publish(message)
        detail = f"attempt {attempt}/{args.grounding_retry_attempts}: {command}"
        feedback("grounding", detail)
        node.get_logger().info(f"视觉定位{detail}")

        deadline = time.monotonic() + args.grounding_attempt_timeout
        expected_stamp = None
        api_succeeded_at = None
        pending_statuses = []
        failure = None
        while time.monotonic() < deadline:
            if cancel_requested():
                raise GraspCancelled("grasp goal canceled during visual grounding")
            rclpy_mod.spin_once(node, timeout_sec=0.1)

            while True:
                result = result_listener.pop_result()
                if result is None:
                    break
                if result.get("request_id") != request_id:
                    node.get_logger().warning("忽略上一轮视觉请求的迟到结果")
                    continue
                if not result.get("success", False):
                    failure = result.get("error") or "API 画框失败"
                    break
                expected_stamp = _payload_stamp(result)
                if expected_stamp is None:
                    failure = "API 结果缺少有效的图像时间戳"
                    break
                api_succeeded_at = time.monotonic()
            if failure is not None:
                break

            while True:
                status = localizer_listener.pop_status()
                if status is None:
                    break
                pending_statuses.append(status)

            if expected_stamp is not None:
                for status in pending_statuses:
                    if (
                        _payload_stamp(status) == expected_stamp
                        and not status.get("success", False)
                    ):
                        failure = "RGB-D 定位失败：" + str(
                            status.get("error") or "未知深度/同步错误"
                        )
                        break
                if failure is not None:
                    break
                target = target_listener.take_for_stamp(expected_stamp)
                if target is not None:
                    return target
                if (
                    api_succeeded_at is not None
                    and time.monotonic() - api_succeeded_at
                    >= args.localization_result_timeout
                ):
                    failure = (
                        f"API 已返回检测框，但 {args.localization_result_timeout:.1f}s "
                        "内没有得到有效深度坐标"
                    )
                    break

        if failure is None:
            failure = f"视觉定位尝试超过 {args.grounding_attempt_timeout:.1f}s"
        failures.append(f"第 {attempt} 次：{failure}")
        node.get_logger().warning(failures[-1])
        if attempt < args.grounding_retry_attempts:
            retry_deadline = time.monotonic() + args.grounding_retry_delay
            while time.monotonic() < retry_deadline:
                if cancel_requested():
                    raise GraspCancelled("grasp goal canceled before perception retry")
                rclpy_mod.spin_once(
                    node,
                    timeout_sec=min(0.1, retry_deadline - time.monotonic()),
                )

    raise RuntimeError("视觉定位失败：" + "；".join(failures))
