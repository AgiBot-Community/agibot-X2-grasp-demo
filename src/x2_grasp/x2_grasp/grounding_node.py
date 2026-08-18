"""ROS 2 node that invokes Volcengine visual grounding on target commands."""

from __future__ import annotations

import base64
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
import time
from typing import Any
import uuid

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, PolygonStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from x2_grasp.msg import GroundingCommand, GroundingResult

from x2_common import package_file, put_latest, resolve_secret

from .ark_http import ArkHttpClient, completion_content
from .grounding import (
    NormalizedBox,
    build_prompt,
    detect_mime_type,
    image_dimensions,
    parse_bboxes,
    safe_artifact_component,
    select_box,
)
from .target_catalog import (
    DEFAULT_TARGET_ALIASES,
    DEFAULT_TARGET_DESCRIPTIONS,
    DEFAULT_TARGET_NAMES,
    build_target_catalog,
)


@dataclass(frozen=True)
class PendingRequest:
    request_id: str
    target: str
    target_zh: str
    image_data: bytes
    image_format: str
    frame_id: str
    stamp_sec: int
    stamp_nanosec: int
    enqueued_at: float


class GroundingNode(Node):
    def __init__(self) -> None:
        super().__init__("x2_grounding")

        self.declare_parameter(
            "image_topic",
            "/aima/hal/sensor/rgbd_head_front/rgb_image",
        )
        self.declare_parameter("target_topic", "/x2_grasp/grounding_target")
        self.declare_parameter("result_topic", "/x2_grasp/grounding_result")
        self.declare_parameter(
            "bbox_topic", "/x2_rgbd_localizer/detection_bbox"
        )
        self.declare_parameter(
            "hold_frame_topic", "/x2_grasp/grounding_image_stamp"
        )
        self.declare_parameter("bbox_selection", "largest")
        self.declare_parameter("model", "doubao-seed-2-1-pro-260628")
        self.declare_parameter(
            "base_url", "https://ark.cn-beijing.volces.com/api/v3"
        )
        self.declare_parameter("api_key_env", "ARK_API_KEY")
        self.declare_parameter("api_key", "")
        self.declare_parameter("api_key_file", "")
        self.declare_parameter("request_timeout_seconds", 120.0)
        self.declare_parameter("max_queue_age_seconds", 5.0)
        self.declare_parameter("annotated_image_directory", "")
        self.declare_parameter("target_names", DEFAULT_TARGET_NAMES)
        self.declare_parameter("target_descriptions", DEFAULT_TARGET_DESCRIPTIONS)
        self.declare_parameter("target_aliases", DEFAULT_TARGET_ALIASES)

        self._target_catalog = build_target_catalog(
            self.get_parameter("target_names").value,
            self.get_parameter("target_descriptions").value,
            self.get_parameter("target_aliases").value,
        )

        self._model = self.get_parameter("model").value
        self._base_url = self.get_parameter("base_url").value
        self._annotated_image_directory = str(
            self.get_parameter("annotated_image_directory").value
        ).strip()
        self._max_queue_age_seconds = float(
            self.get_parameter("max_queue_age_seconds").value
        )
        if self._max_queue_age_seconds <= 0.0:
            raise RuntimeError("max_queue_age_seconds 必须为正数")
        self._bbox_selection = str(
            self.get_parameter("bbox_selection").value
        ).strip().lower()
        if self._bbox_selection not in {"largest", "first"}:
            raise RuntimeError("bbox_selection 只支持 largest 或 first")
        api_key_env = self.get_parameter("api_key_env").value
        configured_api_key = str(self.get_parameter("api_key").value).strip()
        api_key_file = self._resolve_api_key_file(
            str(self.get_parameter("api_key_file").value).strip()
        )
        self._api_key = resolve_secret(
            environment_variable=str(api_key_env),
            configured_value=configured_api_key,
            file_path=api_key_file,
            keys=("api_key",),
        )
        if not self._api_key:
            raise RuntimeError(
                f"API Key 为空；请设置环境变量 {api_key_env} 或配置 api_key_file"
            )

        self._client = ArkHttpClient(
            api_key=self._api_key,
            base_url=self._base_url,
            timeout=float(self.get_parameter("request_timeout_seconds").value),
        )
        self._bridge = CvBridge()
        self._latest_image: Image | None = None
        self._image_lock = Lock()
        self._stop_requested = Event()
        self._requests: Queue[PendingRequest | None] = Queue(maxsize=1)
        self._worker = Thread(
            target=self._worker_loop,
            name="ark-grounding-worker",
            daemon=True,
        )

        image_topic = self.get_parameter("image_topic").value
        target_topic = self.get_parameter("target_topic").value
        result_topic = self.get_parameter("result_topic").value
        self._result_publisher = self.create_publisher(
            GroundingResult, result_topic, 10
        )
        self._bbox_publisher = self.create_publisher(
            PolygonStamped, self.get_parameter("bbox_topic").value, 10
        )
        self._hold_frame_publisher = self.create_publisher(
            Header, self.get_parameter("hold_frame_topic").value, 10
        )
        self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(GroundingCommand, target_topic, self._on_target, 10)
        self._worker.start()

        self.get_logger().info(
            f"等待图像 {image_topic} 和按需识别指令 {target_topic}；"
            f"可用指令: {', '.join(self._target_catalog.names)}"
        )

    @staticmethod
    def _resolve_api_key_file(path_value: str) -> str:
        if not path_value:
            return ""
        return str(
            package_file(
                "x2_grasp",
                Path("config") / Path(path_value).expanduser(),
                source_root=Path(__file__).resolve().parents[1],
            )
        )

    def _on_image(self, message: Image) -> None:
        with self._image_lock:
            self._latest_image = message

    def _on_target(self, message: GroundingCommand) -> None:
        if self._stop_requested.is_set():
            self.get_logger().warning("节点正在停止，忽略新的识别请求")
            return
        request_id = message.request_id.strip() or uuid.uuid4().hex
        target_text = message.target
        target = self._target_catalog.normalize(target_text)
        if target is None:
            self._publish_error(
                request_id,
                target_text.strip(),
                "不支持的识别内容；可用目标: "
                + ", ".join(self._target_catalog.names),
            )
            return

        with self._image_lock:
            image = self._latest_image

        if image is None:
            self._publish_error(request_id, target, "尚未收到 RGB 图像，不调用模型")
            return
        try:
            cv_image = self._bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            encoded, jpeg = cv2.imencode(
                ".jpg", cv_image, [cv2.IMWRITE_JPEG_QUALITY, 95]
            )
            if not encoded:
                raise ValueError("OpenCV JPEG 编码失败")
            image_data = jpeg.tobytes()
        except Exception as exc:
            self._publish_error(request_id, target, f"RGB 图像编码失败：{exc}")
            return
        image_format = "jpeg"
        frame_id = image.header.frame_id
        stamp_sec = image.header.stamp.sec
        stamp_nanosec = image.header.stamp.nanosec

        pending = PendingRequest(
            request_id=request_id,
            target=target,
            target_zh=self._target_catalog.spec(target).description,
            image_data=image_data,
            image_format=image_format,
            frame_id=frame_id,
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
            enqueued_at=time.monotonic(),
        )
        replaced = put_latest(
            self._requests, pending, mark_dropped_done=True
        )
        if replaced is not None:
            self._publish_error(
                replaced.request_id,
                replaced.target,
                "识别请求已被更新的请求替代",
            )
        hold_message = Header()
        hold_message.stamp.sec = stamp_sec
        hold_message.stamp.nanosec = stamp_nanosec
        hold_message.frame_id = frame_id
        self._hold_frame_publisher.publish(hold_message)
        self.get_logger().info(f"识别请求已入队: {request_id} target={target}")

    def _worker_loop(self) -> None:
        while not self._stop_requested.is_set() and rclpy.ok():
            try:
                request = self._requests.get(timeout=0.1)
            except Empty:
                continue
            try:
                if request is None:
                    return
                if self._stop_requested.is_set():
                    return
                if (
                    time.monotonic() - request.enqueued_at
                    > self._max_queue_age_seconds
                ):
                    self._publish_error(
                        request.request_id,
                        request.target,
                        "识别请求排队时间过长，已丢弃",
                    )
                    continue
                self._invoke_once(request)
            finally:
                self._requests.task_done()

    def _invoke_once(self, request: PendingRequest) -> None:
        if self._stop_requested.is_set():
            return
        started = time.monotonic()
        mime_type = detect_mime_type(request.image_data, request.image_format)
        dimensions = image_dimensions(request.image_data)
        width, height = dimensions if dimensions is not None else (0, 0)
        encoded_image = base64.b64encode(request.image_data).decode("ascii")

        try:
            response = self._client.create_chat_completion(
                model=self._model,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": (
                                        f"data:{mime_type};base64,{encoded_image}"
                                    )
                                },
                            },
                            {
                                "type": "text",
                                "text": build_prompt(request.target_zh),
                            },
                        ],
                    }
                ],
            )
            if self._stop_requested.is_set():
                return
            content = self._response_text(completion_content(response))
            boxes = parse_bboxes(content)
            if content.strip().upper() != "NONE" and not boxes:
                raise ValueError(f"模型响应中没有有效的 <bbox> 标签: {content}")

            result_boxes = []
            for box in boxes:
                result_boxes.append(
                    {
                        "normalized_1000": box.as_dict(),
                        "pixel": box.to_pixels(width, height),
                    }
                )

            annotated_image_path = self._save_annotated_image(request, boxes)
            selected = select_box(boxes, self._bbox_selection)

            payload = self._base_payload(request)
            payload.update(
                {
                    "success": True,
                    "image_width": width,
                    "image_height": height,
                    "boxes": result_boxes,
                    "bbox_selection": self._bbox_selection,
                    "raw_response": content,
                    "annotated_image_path": annotated_image_path,
                    "latency_ms": round((time.monotonic() - started) * 1000),
                    "error": "",
                }
            )
            if selected is not None:
                self._publish_bbox(request, selected, width, height)
            self._publish(payload)
            self.get_logger().info(
                f"识别完成: {request.request_id} boxes={len(result_boxes)}"
            )
        except Exception as exc:  # Keep the worker alive after network/API errors.
            if self._stop_requested.is_set():
                return
            payload = self._base_payload(request)
            payload.update(
                {
                    "success": False,
                    "image_width": width,
                    "image_height": height,
                    "boxes": [],
                    "raw_response": "",
                    "annotated_image_path": "",
                    "latency_ms": round((time.monotonic() - started) * 1000),
                    "error": str(exc),
                }
            )
            self._publish(payload)
            self.get_logger().error(f"识别失败: {request.request_id}: {exc}")

    @staticmethod
    def _response_text(content: Any) -> str:
        if isinstance(content, str):
            return content
        if isinstance(content, list):
            parts: list[str] = []
            for item in content:
                if isinstance(item, dict) and isinstance(item.get("text"), str):
                    parts.append(item["text"])
                elif isinstance(getattr(item, "text", None), str):
                    parts.append(item.text)
            return "".join(parts)
        return str(content)

    def _publish_bbox(
        self,
        request: PendingRequest,
        box: NormalizedBox,
        width: int,
        height: int,
    ) -> None:
        pixel = box.to_pixels(width, height)
        if pixel is None:
            raise ValueError("无法取得输入图像尺寸，不能发布像素检测框")
        message = PolygonStamped()
        message.header.stamp.sec = request.stamp_sec
        message.header.stamp.nanosec = request.stamp_nanosec
        message.header.frame_id = request.frame_id
        first = Point32()
        first.x = float(pixel["x_min"])
        first.y = float(pixel["y_min"])
        second = Point32()
        second.x = float(pixel["x_max"])
        second.y = float(pixel["y_max"])
        message.polygon.points = [first, second]
        self._bbox_publisher.publish(message)
        self.get_logger().info(
            "已发布抓取框: "
            f"({first.x:.0f}, {first.y:.0f})-({second.x:.0f}, {second.y:.0f})"
        )

    def _save_annotated_image(
        self,
        request: PendingRequest,
        boxes: list[NormalizedBox],
    ) -> str:
        if not self._annotated_image_directory:
            return ""

        try:
            import cv2
            import numpy as np

            image = cv2.imdecode(
                np.frombuffer(request.image_data, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if image is None:
                raise ValueError("OpenCV 无法解码压缩图像")

            height, width = image.shape[:2]
            for index, box in enumerate(boxes, start=1):
                pixel = box.to_pixels(width, height)
                if pixel is None:
                    continue
                start = (pixel["x_min"], pixel["y_min"])
                end = (pixel["x_max"], pixel["y_max"])
                cv2.rectangle(image, start, end, (0, 255, 0), 5)
                label_y = max(32, pixel["y_min"] - 12)
                cv2.putText(
                    image,
                    f"{request.target} {index}",
                    (pixel["x_min"], label_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    3,
                    cv2.LINE_AA,
                )

            if not boxes:
                cv2.putText(
                    image,
                    f"{request.target}: NO DETECTION",
                    (30, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 0, 255),
                    3,
                    cv2.LINE_AA,
                )

            output_directory = Path(self._annotated_image_directory).expanduser()
            output_directory.mkdir(parents=True, exist_ok=True)
            request_component = safe_artifact_component(request.request_id)
            output_path = output_directory / f"{request.target}_{request_component}.jpg"
            if not cv2.imwrite(
                str(output_path),
                image,
                [cv2.IMWRITE_JPEG_QUALITY, 95],
            ):
                raise OSError(f"无法写入标注图像: {output_path}")
            return str(output_path.resolve())
        except Exception as exc:
            self.get_logger().warning(f"保存标注图像失败: {exc}")
            return ""

    @staticmethod
    def _base_payload(request: PendingRequest) -> dict[str, Any]:
        return {
            "request_id": request.request_id,
            "target": request.target,
            "target_zh": request.target_zh,
            "image_stamp": {
                "sec": request.stamp_sec,
                "nanosec": request.stamp_nanosec,
            },
            "frame_id": request.frame_id,
        }

    def _publish_error(self, request_id: str, target: str, error: str) -> None:
        self._publish(
            {
                "request_id": request_id,
                "success": False,
                "target": target,
                "target_zh": (
                    self._target_catalog.spec(target).description
                    if target in self._target_catalog.specs
                    else ""
                ),
                "image_stamp": {"sec": 0, "nanosec": 0},
                "frame_id": "",
                "image_width": 0,
                "image_height": 0,
                "boxes": [],
                "raw_response": "",
                "annotated_image_path": "",
                "latency_ms": 0,
                "error": error,
            }
        )
        self.get_logger().warning(error)

    def _publish(self, payload: dict[str, Any]) -> None:
        stamp = payload.get("image_stamp") or {}
        message = GroundingResult()
        message.image_stamp.sec = int(stamp.get("sec", 0))
        message.image_stamp.nanosec = int(stamp.get("nanosec", 0))
        message.request_id = str(payload.get("request_id", ""))
        message.target = str(payload.get("target", ""))
        message.target_zh = str(payload.get("target_zh", ""))
        message.success = bool(payload.get("success", False))
        message.frame_id = str(payload.get("frame_id", ""))
        message.image_width = int(payload.get("image_width", 0))
        message.image_height = int(payload.get("image_height", 0))
        message.box_count = len(payload.get("boxes") or [])
        message.bbox_selection = str(payload.get("bbox_selection", ""))
        message.annotated_image_path = str(payload.get("annotated_image_path", ""))
        message.latency_ms = float(payload.get("latency_ms", 0.0))
        message.error = str(payload.get("error", ""))
        self._result_publisher.publish(message)

    def stop(self) -> bool:
        self._stop_requested.set()
        try:
            queued = self._requests.get_nowait()
            self._requests.task_done()
            if queued is not None:
                self.get_logger().warning(
                    f"停止时丢弃排队请求: {queued.request_id}"
                )
        except Empty:
            pass
        try:
            self._requests.put_nowait(None)
        except Full:
            pass
        self._worker.join(timeout=2.0)
        stopped = not self._worker.is_alive()
        if not stopped:
            self.get_logger().error(
                "Grounding worker 未在 2 秒内退出；保留节点资源以避免后台线程访问已销毁对象"
            )
        return stopped


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: GroundingNode | None = None
    try:
        node = GroundingNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"x2_grasp grounding 启动失败: {exc}")
    finally:
        if node is not None:
            try:
                if node.stop():
                    node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
