#!/usr/bin/env python3
"""ARTag 检测节点：AprilTag 36h11 -> solvePnP -> TF -> base_link 目标向量。

链路::

    rgb_image + rgb_camera_info
        -> 36h11 detectMarkers
        -> solvePnP(SOLVEPNP_IPPE_SQUARE)  # 用原始内参与原始像素，不做任何旋转
        -> TF camera_optical -> base_link
        -> 稳定性判据
        -> /x2_apriltag/target_vector  (Vector3Stamped, base_link, 米)

稳定判据（三个条件同时满足才算稳定，全部可配）：

  1. 连续 ``stable_frames`` 帧都检出目标 ID；
  2. 这些帧的 base_link 坐标最大离散度 ``max_spread_m`` 以内（逐轴 max-min）；
  3. 每帧的 solvePnP 重投影误差 ≤ ``max_reproj_error_px``。

一旦稳定，就按 ``publish_mode`` 决定行为：

  * ``once``（默认）：只发一次，然后进入冷却，直到标签消失 ``reset_after_lost_s``
    秒或坐标跳变超过 ``retrigger_move_m`` 才允许再次触发。适合"放一个物体抓一次"。
  * ``continuous``：持续按 ``publish_rate_hz`` 发布，适合调试观察。

任何一帧不满足条件都会清空累计窗口，重新从 0 开始数。

单独运行::

    source /opt/ros/humble/setup.bash && source ~/x2_grasp_ws/install/setup.bash
    ros2 run x2_grasp apriltag_detector --ros-args -p tag_size_m:=0.080

调试（只看不发）::

    ros2 run x2_grasp apriltag_detector --ros-args -p dry_run:=true
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Any, Callable, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from x2_grasp.msg import PerceptionStatus

DEFAULT_RGB_TOPIC = "/aima/hal/sensor/rgbd_head_front/rgb_image"
DEFAULT_RGB_INFO_TOPIC = "/aima/hal/sensor/rgbd_head_front/rgb_camera_info"
DEFAULT_APRILTAG_TOPIC = "/x2_apriltag/target_vector"
DEFAULT_STATUS_TOPIC = "/x2_apriltag/status"


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert a finite, non-zero quaternion to a rotation matrix."""
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise ValueError("TF quaternion contains non-finite values")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("TF quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def make_detector_36h11() -> Callable[[np.ndarray], tuple[Any, Any]]:
    """构造 AprilTag 36h11 检测器，兼容 OpenCV 4.6 之前/之后的两套 aruco API。"""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
    else:  # OpenCV < 4.7
        params = cv2.aruco.DetectorParameters_create()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return lambda gray: detector.detectMarkers(gray)[:2]
    return lambda gray: cv2.aruco.detectMarkers(
        gray, dictionary, parameters=params)[:2]


class AprilTagDetectorNode(Node):
    """检测 AprilTag，稳定后把 base_link 坐标发布出去。"""

    def __init__(self) -> None:
        super().__init__("x2_apriltag_detector")

        self.declare_parameter("rgb_topic", DEFAULT_RGB_TOPIC)
        self.declare_parameter("rgb_info_topic", DEFAULT_RGB_INFO_TOPIC)
        self.declare_parameter("target_topic", DEFAULT_APRILTAG_TOPIC)
        self.declare_parameter("status_topic", DEFAULT_STATUS_TOPIC)
        self.declare_parameter("tag_id", 0)
        self.declare_parameter("tag_size_m", 0.080)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("min_side_pixels", 40.0)
        self.declare_parameter("max_reproj_error_px", 3.0)
        self.declare_parameter("stable_frames", 8)
        self.declare_parameter("max_spread_m", 0.010)
        self.declare_parameter("publish_mode", "once")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("reset_after_lost_s", 1.5)
        self.declare_parameter("retrigger_move_m", 0.05)
        self.declare_parameter("detect_rate_hz", 10.0)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("log_every_frame", False)

        get = self.get_parameter
        self.rgb_topic = get("rgb_topic").value
        self.rgb_info_topic = get("rgb_info_topic").value
        self.target_topic = get("target_topic").value
        self.tag_id = int(get("tag_id").value)
        self.tag_size = float(get("tag_size_m").value)
        self.base_frame = get("base_frame").value
        self.camera_frame = get("camera_frame").value or None
        self.min_side_pixels = float(get("min_side_pixels").value)
        self.max_reproj_error_px = float(get("max_reproj_error_px").value)
        self.stable_frames = int(get("stable_frames").value)
        self.max_spread_m = float(get("max_spread_m").value)
        self.publish_mode = str(get("publish_mode").value).lower()
        publish_rate_hz = float(get("publish_rate_hz").value)
        self.reset_after_lost_s = float(get("reset_after_lost_s").value)
        self.retrigger_move_m = float(get("retrigger_move_m").value)
        detect_rate_hz = float(get("detect_rate_hz").value)
        self.dry_run = bool(get("dry_run").value)
        self.log_every_frame = bool(get("log_every_frame").value)

        if self.publish_mode not in ("once", "continuous"):
            raise ValueError("publish_mode 只能是 once 或 continuous")
        positive_parameters = {
            "tag_size_m": self.tag_size,
            "min_side_pixels": self.min_side_pixels,
            "max_reproj_error_px": self.max_reproj_error_px,
            "max_spread_m": self.max_spread_m,
            "publish_rate_hz": publish_rate_hz,
            "detect_rate_hz": detect_rate_hz,
        }
        for name, value in positive_parameters.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} 必须是有限正数")
        for name, value in (
            ("reset_after_lost_s", self.reset_after_lost_s),
            ("retrigger_move_m", self.retrigger_move_m),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} 必须是有限非负数")
        if self.stable_frames < 1:
            raise ValueError("stable_frames 必须至少为 1")
        if self.tag_id < 0:
            raise ValueError("tag_id 不能为负数")
        if not str(self.base_frame).strip():
            raise ValueError("base_frame 不能为空")
        self.publish_period = 1.0 / publish_rate_hz
        self.detect_period = 1.0 / detect_rate_hz

        self.bridge = CvBridge()
        self.detect = make_detector_36h11()
        self.camera_info: Optional[CameraInfo] = None
        self.window: deque[np.ndarray] = deque(maxlen=self.stable_frames)
        self.last_detect_time = 0.0
        self.last_seen_time: Optional[float] = None
        self.last_publish_time = 0.0
        self.published_xyz: Optional[np.ndarray] = None
        self.publish_count = 0
        self.current_image_stamp = None
        self.armed = True  # once 模式下：True 表示允许触发下一次发布

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.target_pub = self.create_publisher(
            Vector3Stamped, self.target_topic, QoSProfile(depth=10))
        self.status_pub = self.create_publisher(
            PerceptionStatus, get("status_topic").value, QoSProfile(depth=10))
        self.create_subscription(
            CameraInfo, self.rgb_info_topic, self.on_camera_info, sensor_qos)
        self.create_subscription(
            Image, self.rgb_topic, self.on_image, sensor_qos)

        self.get_logger().info(
            f"ARTag 检测启动：tag_id={self.tag_id} tag_size={self.tag_size:.3f}m "
            f"stable_frames={self.stable_frames} max_spread={self.max_spread_m * 1000:.0f}mm "
            f"mode={self.publish_mode} dry_run={self.dry_run}")
        self.get_logger().info(
            f"订阅 {self.rgb_topic}；稳定后发布到 {self.target_topic}")

    # ------------------------------------------------------------------
    def on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def publish_status(self, result: str, **extra: Any) -> None:
        message = PerceptionStatus()
        if self.current_image_stamp is not None:
            message.image_stamp = self.current_image_stamp
        message.source = "apriltag"
        message.stage = result
        message.success = result in {
            "PUBLISHED",
            "STABLE_DRY_RUN",
            "STABLE_ALREADY_PUBLISHED",
        }
        message.detected_ids = [int(value) for value in extra.get("detected", [])]
        message.sample_count = len(self.window)
        message.required_samples = self.stable_frames
        message.spread_m = float(extra.get("spread_mm", 0.0)) / 1000.0
        message.reprojection_error_px = float(extra.get("reproj_px", 0.0))
        xyz = extra.get("xyz")
        if isinstance(xyz, (list, tuple)) and len(xyz) == 3:
            message.target.x = float(xyz[0])
            message.target.y = float(xyz[1])
            message.target.z = float(xyz[2])
        message.frame_id = self.base_frame
        detail_items = [
            f"{key}={value}"
            for key, value in extra.items()
            if key not in {"detected", "spread_mm", "reproj_px", "xyz"}
        ]
        message.detail = " ".join(detail_items)
        if result in {
            "PNP_FAILED",
            "REPROJ_ERROR_TOO_LARGE",
            "TF_LOOKUP_FAILED",
            "NON_FINITE_BASE_XYZ",
        }:
            message.error = message.detail or result
        self.status_pub.publish(message)
        if self.log_every_frame:
            self.get_logger().info(
                f"stage={result} progress={len(self.window)}/{self.stable_frames} "
                f"{message.detail}"
            )

    def reset_window(self, reason: str) -> None:
        if self.window:
            self.window.clear()
            if self.log_every_frame:
                self.get_logger().info(f"稳定窗口清零：{reason}")

    # ------------------------------------------------------------------
    def lookup_base_tf(self, image_frame_id: str):
        """返回 (R, t, used_frame)，把相机光学系下的点变换到 base_frame。"""
        candidates: list[str] = []
        for name in (self.camera_frame, image_frame_id,
                     "rgbd_head_front",
                     "rgbd_head_front_color_optical_frame",
                     "rgbd_head_front_rgb_optical_frame"):
            if name and name not in candidates:
                candidates.append(name)
        last_error = "NO_CANDIDATE_FRAME"
        for frame in candidates:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, frame, rclpy.time.Time())
            except Exception as error:  # noqa: BLE001 - TF 尚未就绪属常态
                last_error = f"{type(error).__name__}[{frame}]"
                continue
            q = transform.transform.rotation
            v = transform.transform.translation
            try:
                rotation = quat_to_matrix(q.x, q.y, q.z, q.w)
            except ValueError as error:
                last_error = f"INVALID_TF[{frame}]: {error}"
                continue
            return (
                rotation,
                np.array([v.x, v.y, v.z], dtype=np.float64),
                frame,
                "OK",
            )
        return None, None, None, last_error

    # ------------------------------------------------------------------
    def on_image(self, message: Image) -> None:
        self.current_image_stamp = message.header.stamp
        now = time.monotonic()
        if now - self.last_detect_time < self.detect_period:
            return
        self.last_detect_time = now

        # once 模式：标签消失够久就重新武装，允许下一次触发。
        if (not self.armed and self.last_seen_time is not None
                and now - self.last_seen_time > self.reset_after_lost_s):
            self.armed = True
            self.published_xyz = None
            self.get_logger().info(
                f"标签已消失 {self.reset_after_lost_s:.1f}s，重新武装，等待下一次稳定识别")

        if self.camera_info is None:
            self.publish_status("WAITING_FOR_CAMERA_INFO")
            return

        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"图像转换失败：{error}")
            return

        # solvePnP 必须用相机原始内参与原始像素。相机是倒装的，但三维计算完全
        # 不受影响 —— 任何 180° 旋转都只是显示层的事，这里绝不做旋转。
        k = np.asarray(self.camera_info.k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(self.camera_info.d, dtype=np.float64)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect(gray)
        id_list = [] if ids is None else [int(v) for v in ids.reshape(-1)]

        if self.tag_id not in id_list:
            self.reset_window("未检出目标 ID")
            self.publish_status("NO_TAG_DETECTED", detected=id_list)
            return

        self.last_seen_time = now
        quad = corners[id_list.index(self.tag_id)].reshape(4, 2).astype(np.float64)
        side_px = float(np.mean([
            np.linalg.norm(quad[i] - quad[(i + 1) % 4]) for i in range(4)
        ]))
        if side_px < self.min_side_pixels:
            self.reset_window("标签在画面里太小")
            self.publish_status("TAG_TOO_SMALL", side_px=round(side_px, 1))
            return

        half = self.tag_size / 2.0
        object_points = np.array([
            [-half, half, 0.0], [half, half, 0.0],
            [half, -half, 0.0], [-half, -half, 0.0],
        ], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            object_points, quad, k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            self.reset_window("solvePnP 失败")
            self.publish_status("PNP_FAILED")
            return

        reprojected, _ = cv2.projectPoints(object_points, rvec, tvec, k, d)
        error_px = float(np.mean(np.linalg.norm(
            reprojected.reshape(4, 2) - quad, axis=1)))
        if not math.isfinite(error_px) or error_px > self.max_reproj_error_px:
            self.reset_window(f"重投影误差过大 {error_px:.2f}px")
            self.publish_status("REPROJ_ERROR_TOO_LARGE",
                                reproj_px=round(error_px, 2))
            return

        rotation, translation, used_frame, tf_status = self.lookup_base_tf(
            message.header.frame_id)
        if rotation is None:
            self.reset_window("TF 查询失败")
            self.publish_status("TF_LOOKUP_FAILED", tf=tf_status)
            return

        base_xyz = rotation.dot(tvec.reshape(3)) + translation
        if not np.all(np.isfinite(base_xyz)):
            self.reset_window("base_link 坐标非有限")
            self.publish_status("NON_FINITE_BASE_XYZ")
            return

        self.window.append(base_xyz)
        spread = 0.0
        if len(self.window) > 1:
            stack = np.stack(self.window)
            spread = float(np.max(stack.max(axis=0) - stack.min(axis=0)))
            if spread > self.max_spread_m:
                # 抖太大：保留当前帧重新开始计数，而不是彻底清空，
                # 这样物体刚放稳的那一刻能立刻开始累计。
                self.window.clear()
                self.window.append(base_xyz)
                self.publish_status("UNSTABLE",
                                    spread_mm=round(spread * 1000.0, 1))
                return

        if len(self.window) < self.stable_frames:
            self.publish_status("ACCUMULATING",
                                spread_mm=round(spread * 1000.0, 1),
                                xyz=np.round(base_xyz, 4).tolist())
            return

        # ---- 到这里说明已经稳定 ----
        stable_xyz = np.stack(self.window).mean(axis=0)
        self.on_stable(stable_xyz, spread, error_px, used_frame, now)

    # ------------------------------------------------------------------
    def on_stable(self, xyz: np.ndarray, spread: float,
                  error_px: float, used_frame: str, now: float) -> None:
        if self.publish_mode == "once":
            if not self.armed:
                # 已经发过了。除非坐标明显挪动，否则不重复触发。
                if (self.published_xyz is not None
                        and float(np.max(np.abs(xyz - self.published_xyz)))
                        > self.retrigger_move_m):
                    self.get_logger().info(
                        "标签位置发生明显移动，重新武装并再次发布")
                    self.armed = True
                else:
                    self.publish_status("STABLE_ALREADY_PUBLISHED",
                                        xyz=np.round(xyz, 4).tolist())
                    return
        else:
            if now - self.last_publish_time < self.publish_period:
                return

        if self.dry_run:
            self.get_logger().info(
                f"[dry-run] 稳定目标 base_link={np.round(xyz, 4).tolist()} "
                f"spread={spread * 1000:.1f}mm reproj={error_px:.2f}px "
                f"tf_frame={used_frame}（未发布）")
            self.publish_status("STABLE_DRY_RUN", xyz=np.round(xyz, 4).tolist())
            if self.publish_mode == "once":
                self.armed = False
                self.published_xyz = xyz.copy()
            self.last_publish_time = now
            return

        message = Vector3Stamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.vector.x = float(xyz[0])
        message.vector.y = float(xyz[1])
        message.vector.z = float(xyz[2])
        self.target_pub.publish(message)
        self.publish_count += 1
        self.last_publish_time = now
        self.published_xyz = xyz.copy()
        if self.publish_mode == "once":
            self.armed = False

        self.get_logger().info(
            f"[{self.publish_count}] 稳定识别，已发布 base_link="
            f"{np.round(xyz, 4).tolist()} m -> {self.target_topic}"
            f"（spread={spread * 1000:.1f}mm reproj={error_px:.2f}px "
            f"tf={used_frame}）")
        self.publish_status("PUBLISHED", xyz=np.round(xyz, 4).tolist(),
                            count=self.publish_count)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = AprilTagDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
