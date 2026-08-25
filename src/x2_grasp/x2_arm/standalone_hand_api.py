#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立的 X2 OmniPicker 夹爪控制 API。

这个文件只负责向官方 Hand HAL 发布控制命令，不导入仓库中的任何其他
模块，也不负责 ``rclpy.init``、``rclpy.shutdown`` 或 ROS 环境加载。
因此可以把它单独复制到任意 ROS 2 Python 功能包中使用。

最简单的用法是：

    from x2_arm.standalone_hand_api import StandaloneHandAPI

    self.hand = StandaloneHandAPI(self)
    self.hand.left.open()
    self.hand.right.close()
    self.hand.both.set_positions(0.3, 0.7)

也可以使用 API 根对象的统一入口：

    self.hand.set_position("right", 0.5)
    self.hand.open("left")
    self.hand.close_grippers("both")

如果希望在业务节点中直接写 ``self.left``、``self.right`` 和 ``self.both``，可以使用
``attach_hand_controls(self)``：

    self.hand = attach_hand_controls(self)
    self.left.open()
    self.right.close()
    self.both.close()

位置范围为 ``0.0``（闭合）到 ``1.0``（打开）。这个 API 只暴露控制链路，
不包含旧模块中的状态订阅、手型查询、诊断和命令行逻辑。
"""

import math
import sys
import time
from types import SimpleNamespace


COMMAND_TOPIC = "/aima/hal/joint/hand/command"
LEFT_JOINT_NAME = "left_claw_joint"
RIGHT_JOINT_NAME = "right_claw_joint"
GRIPPER_HAND_TYPE = 2
NONE_HAND_TYPE = 0


class HandControlError(RuntimeError):
    """夹爪控制 API 初始化或使用失败。"""


_ROS_BINDINGS = None

_REQUIRED_ROS_BINDINGS = (
    "HandCommand",
    "HandCommandArray",
    "HandType",
    "MessageHeader",
    "QoSProfile",
    "ReliabilityPolicy",
    "DurabilityPolicy",
)


def load_ros_bindings():
    """延迟加载官方 ROS 2/AimDK 类型。

    延迟导入使得本文件在没有 ROS 2 的开发机上也可以被导入和单元测试。
    生产代码通常不需要传 ``ros`` 参数；单元测试可以注入同样字段的轻量
    fake bindings，从而完全绕过 ROS 运行时。

    业务代码通常不需要直接调用这个函数。``StandaloneHandAPI`` 在未传入
    ``ros`` 时会自动调用它；只有需要预加载官方类型或注入测试 bindings 时，
    才需要显式使用。
    """

    global _ROS_BINDINGS
    if _ROS_BINDINGS is not None:
        return _ROS_BINDINGS

    try:
        import rclpy
        from aimdk_msgs.msg import (
            HandCommand,
            HandCommandArray,
            HandType,
            MessageHeader,
        )
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        raise HandControlError(
            "无法导入 ROS2/AimDK 消息类型: "
            f"{exc}\n"
            f"Python: {sys.executable} {sys.version.split()[0]}\n"
            "请在已加载 ROS2 环境的功能包节点中使用此 API；"
            "本文件不会自动 source ROS 环境。"
        ) from exc

    _ROS_BINDINGS = SimpleNamespace(
        rclpy=rclpy,
        HandCommand=HandCommand,
        HandCommandArray=HandCommandArray,
        HandType=HandType,
        MessageHeader=MessageHeader,
        QoSProfile=QoSProfile,
        ReliabilityPolicy=ReliabilityPolicy,
        DurabilityPolicy=DurabilityPolicy,
    )
    return _ROS_BINDINGS


def _validate_ros_bindings(bindings):
    missing = [
        name for name in _REQUIRED_ROS_BINDINGS if not hasattr(bindings, name)
    ]
    if missing:
        names = ", ".join(missing)
        raise HandControlError(f"ROS bindings 缺少必要类型: {names}")


def _topic(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError("command_topic 必须是非空字符串")
    return value


def _position(value, label):
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} 必须是 0.0 到 1.0 之间的数字") from exc
    if not math.isfinite(number) or not 0.0 <= number <= 1.0:
        raise ValueError(f"{label} 必须在 0.0 到 1.0 之间")
    return number


def _duration(value, label):
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} 必须是大于 0 的数字") from exc
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{label} 必须是大于 0 的数字")
    return number


def _publish_hz(value):
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("publish_hz 必须是大于 0 的数字") from exc
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError("publish_hz 必须是大于 0 的数字")
    return number


def _target(value):
    if value not in ("left", "right", "both"):
        raise ValueError("target 必须是 left、right 或 both")
    return value


class HandTarget:
    """一个可直接控制的单侧或双侧目标。

    ``StandaloneHandAPI`` 会把三个实例分别放在 ``self.left``、
    ``self.right`` 和 ``self.both`` 上。目标对象故意只保留控制方法，
    不携带 ROS 订阅或状态缓存。
    """

    def __init__(self, api, target):
        self._api = api
        self.target = _target(target)

    def __call__(self, position, seconds=2.0, cancel_requested=lambda: False):
        """允许用 ``self.left(0.5)``、``self.right(0.5)`` 或
        ``self.both(0.5)`` 快速控制。
        """

        return self.set_position(
            position, seconds=seconds, cancel_requested=cancel_requested
        )

    def set_position(self, position, seconds=2.0, cancel_requested=lambda: False):
        """设置位置；对 ``both`` 目标会把同一位置发送给左右手。"""

        if self.target == "left":
            return self._api.set_left_position(
                position, seconds=seconds, cancel_requested=cancel_requested
            )
        if self.target == "right":
            return self._api.set_right_position(
                position, seconds=seconds, cancel_requested=cancel_requested
            )
        return self._api.set_both_position(
            position, seconds=seconds, cancel_requested=cancel_requested
        )

    move = set_position

    def set_positions(
        self,
        left_position,
        right_position,
        seconds=2.0,
        cancel_requested=lambda: False,
    ):
        """分别设置左右位置，仅 ``self.both`` 支持此方法。"""

        if self.target != "both":
            raise ValueError("self.left 不支持 set_positions，请分别控制目标")
        return self._api.set_both_positions(
            left_position,
            right_position,
            seconds=seconds,
            cancel_requested=cancel_requested,
        )

    def open(self, seconds=2.0, cancel_requested=lambda: False):
        """打开目标夹爪。"""

        return self.set_position(
            1.0, seconds=seconds, cancel_requested=cancel_requested
        )

    def close(self, seconds=2.0, cancel_requested=lambda: False):
        """闭合目标夹爪。"""

        return self.set_position(
            0.0, seconds=seconds, cancel_requested=cancel_requested
        )


class StandaloneHandAPI:
    """不依赖仓库其他代码的最小双夹爪控制 API。

    参数 ``node`` 必须是调用方已经创建好的 ROS 2 Node。API 只创建并
    管理自己的一个 publisher，不会初始化或关闭调用方的 ROS context。
    """

    def __init__(
        self,
        node,
        *,
        ros=None,
        command_topic=COMMAND_TOPIC,
        publish_hz=50.0,
        qos_profile=None,
    ):
        if node is None:
            raise TypeError("StandaloneHandAPI 需要一个已创建的 ROS2 Node")

        self.node = node
        self.ros = load_ros_bindings() if ros is None else ros
        _validate_ros_bindings(self.ros)
        self.command_topic = _topic(command_topic)
        self.publish_hz = _publish_hz(publish_hz)
        self.publisher = None
        self._closed = False

        # 这三个属性就是业务代码的主要入口。
        self.left = HandTarget(self, "left")
        self.right = HandTarget(self, "right")
        self.both = HandTarget(self, "both")

        try:
            qos = (
                self.ros.QoSProfile(
                    depth=10,
                    reliability=self.ros.ReliabilityPolicy.BEST_EFFORT,
                    durability=self.ros.DurabilityPolicy.TRANSIENT_LOCAL,
                )
                if qos_profile is None
                else qos_profile
            )
            self.publisher = self.node.create_publisher(
                self.ros.HandCommandArray,
                self.command_topic,
                qos,
            )
        except Exception as exc:
            self.shutdown()
            raise HandControlError(
                f"无法创建夹爪 command publisher: {self.command_topic}"
            ) from exc

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.shutdown()
        return False

    def shutdown(self):
        """只释放本 API 创建的 publisher。可重复调用。"""

        if self._closed:
            return
        self._closed = True
        destroy = getattr(self.node, "destroy_publisher", None)
        if self.publisher is not None and callable(destroy):
            try:
                destroy(self.publisher)
            except Exception:
                pass
        self.publisher = None

    close = shutdown

    def _ensure_open(self):
        if self._closed:
            raise HandControlError("夹爪控制 API 已关闭")

    def _ros_ok(self):
        """没有 rclpy 时默认允许 fake bindings；有 rclpy 时尊重 context 状态。"""

        rclpy = getattr(self.ros, "rclpy", None)
        ok = getattr(rclpy, "ok", None)
        if not callable(ok):
            return True
        try:
            return bool(ok())
        except Exception:
            return False

    def _make_command(self, name, position):
        command = self.ros.HandCommand()
        command.name = name
        command.position = _position(position, "position")
        command.velocity = 1.0
        command.acceleration = 1.0
        command.deceleration = 1.0
        command.effort = 1.0
        return command

    def _make_hand_type(self, value):
        hand_type = self.ros.HandType()
        hand_type.value = value
        return hand_type

    def build_command(
        self,
        left_position=None,
        right_position=None,
        target="both",
    ):
        """构造命令消息但不发布。

        ``target="left"`` 或 ``target="right"`` 时只填充对应侧命令；
        ``target="both"`` 时必须同时提供左右位置。这个方法便于业务代码
        在发布前检查或记录消息。
        """

        target = _target(target)
        if target == "left":
            left_position = _position(left_position, "left_position")
            if right_position is not None:
                raise ValueError("target=left 时不能提供 right_position")
            right_position = None
        elif target == "right":
            right_position = _position(right_position, "right_position")
            if left_position is not None:
                raise ValueError("target=right 时不能提供 left_position")
            left_position = None
        else:
            left_position = _position(left_position, "left_position")
            right_position = _position(right_position, "right_position")

        message = self.ros.HandCommandArray()
        message.header = self.ros.MessageHeader()
        message.left_hand_type = self._make_hand_type(
            GRIPPER_HAND_TYPE if target in ("left", "both") else NONE_HAND_TYPE
        )
        message.right_hand_type = self._make_hand_type(
            GRIPPER_HAND_TYPE if target in ("right", "both") else NONE_HAND_TYPE
        )
        message.left_hands = (
            [self._make_command(LEFT_JOINT_NAME, left_position)]
            if target in ("left", "both")
            else []
        )
        message.right_hands = (
            [self._make_command(RIGHT_JOINT_NAME, right_position)]
            if target in ("right", "both")
            else []
        )
        return message

    build = build_command

    def _publish(
        self,
        left_position,
        right_position,
        target,
        seconds,
        cancel_requested=lambda: False,
    ):
        self._ensure_open()
        if not self._ros_ok():
            raise HandControlError("ROS2 context 未运行，无法发布夹爪命令")
        target = _target(target)
        seconds = _duration(seconds, "seconds")
        message = self.build_command(
            left_position,
            right_position,
            target=target,
        )

        interval = 1.0 / self.publish_hz
        deadline = time.monotonic() + seconds
        frames = 0
        while self._ros_ok():
            self._ensure_open()
            if cancel_requested():
                raise InterruptedError("gripper command canceled")
            try:
                self.publisher.publish(message)
            except Exception as exc:
                raise HandControlError("发布夹爪命令失败") from exc
            frames += 1
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(interval, remaining))

        if frames == 0:
            raise HandControlError("ROS2 context 在发布前已停止")

        return {
            "topic": self.command_topic,
            "target": target,
            "left_position": left_position,
            "right_position": right_position,
            "seconds": seconds,
            "frames": frames,
            "completed": self._ros_ok() and not self._closed,
        }

    def _handle(self, hand):
        hand = _target(hand)
        return getattr(self, hand)

    def set_position(
        self, hand, position, seconds=2.0, cancel_requested=lambda: False
    ):
        """统一入口：按 ``left``、``right`` 或 ``both`` 控制。"""

        return self._handle(hand).set_position(
            position, seconds=seconds, cancel_requested=cancel_requested
        )

    def set_positions(
        self,
        left_position,
        right_position,
        seconds=2.0,
        cancel_requested=lambda: False,
    ):
        """统一双手入口，等同于 ``self.both.set_positions(...)``。"""

        return self.both.set_positions(
            left_position,
            right_position,
            seconds=seconds,
            cancel_requested=cancel_requested,
        )

    def open(
        self, hand="both", seconds=2.0, cancel_requested=lambda: False
    ):
        """统一打开入口，按 ``hand`` 选择一侧或双手。"""

        return self._handle(hand).open(
            seconds=seconds, cancel_requested=cancel_requested
        )

    def close_grippers(
        self, hand="both", seconds=2.0, cancel_requested=lambda: False
    ):
        """统一闭合入口；资源生命周期请使用 ``shutdown()``。"""

        return self._handle(hand).close(
            seconds=seconds, cancel_requested=cancel_requested
        )

    def set_left_position(
        self, position, seconds=2.0, cancel_requested=lambda: False
    ):
        """发布左手位置。业务代码通常直接使用 ``self.left``。"""

        position = _position(position, "position")
        return self._publish(
            position, None, "left", seconds, cancel_requested
        )

    def set_right_position(
        self, position, seconds=2.0, cancel_requested=lambda: False
    ):
        """发布右手位置。业务代码通常直接使用 ``self.right``。"""

        position = _position(position, "position")
        return self._publish(
            None, position, "right", seconds, cancel_requested
        )

    def set_both_position(
        self, position, seconds=2.0, cancel_requested=lambda: False
    ):
        """让左右手移动到同一个位置。"""

        position = _position(position, "position")
        return self._publish(
            position, position, "both", seconds, cancel_requested
        )

    def set_both_positions(
        self,
        left_position,
        right_position,
        seconds=2.0,
        cancel_requested=lambda: False,
    ):
        """分别发布左右手位置。"""

        left_position = _position(left_position, "left_position")
        right_position = _position(right_position, "right_position")
        return self._publish(
            left_position,
            right_position,
            "both",
            seconds,
            cancel_requested,
        )


def attach_hand_controls(
    owner,
    *,
    ros=None,
    command_topic=COMMAND_TOPIC,
    publish_hz=50.0,
    qos_profile=None,
):
    """把 ``left``/``right``/``both`` 三个入口直接挂到业务节点对象上。

    返回的 API 对象应由调用方保存，并在节点销毁前调用 ``shutdown``。
    函数不会替业务节点初始化或关闭 ROS：

        self.hand = attach_hand_controls(self)
        self.left.open()
        self.right.close()
        self.both.set_positions(0.3, 0.7)
    """

    api = StandaloneHandAPI(
        owner,
        ros=ros,
        command_topic=command_topic,
        publish_hz=publish_hz,
        qos_profile=qos_profile,
    )
    owner.left = api.left
    owner.right = api.right
    owner.both = api.both
    return api


# 便于不同项目使用自己习惯的名称；这些别名仍然指向同一个独立实现。
HandControlAPI = StandaloneHandAPI
CleanHandAPI = StandaloneHandAPI


__all__ = [
    "COMMAND_TOPIC",
    "CleanHandAPI",
    "GRIPPER_HAND_TYPE",
    "HandControlAPI",
    "HandControlError",
    "HandTarget",
    "NONE_HAND_TYPE",
    "RIGHT_JOINT_NAME",
    "LEFT_JOINT_NAME",
    "StandaloneHandAPI",
    "attach_hand_controls",
    "load_ros_bindings",
]
