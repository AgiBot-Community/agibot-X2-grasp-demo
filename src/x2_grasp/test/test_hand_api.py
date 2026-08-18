from types import SimpleNamespace

import pytest

from x2_arm.standalone_hand_api import HandControlError, StandaloneHandAPI


class FakeHandCommand:
    def __init__(self):
        self.name = ""
        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.deceleration = 0.0
        self.effort = 0.0


class FakeHandType:
    def __init__(self):
        self.value = 0


class FakeHandCommandArray:
    def __init__(self):
        self.header = None
        self.left_hand_type = None
        self.right_hand_type = None
        self.left_hands = []
        self.right_hands = []


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.publisher = None

    def create_publisher(self, message_type, topic, qos):
        self.publisher = FakePublisher()
        assert message_type is FakeHandCommandArray
        assert topic == "/test/hand"
        return self.publisher

    def destroy_publisher(self, publisher):
        assert publisher is self.publisher


def fake_bindings():
    return SimpleNamespace(
        rclpy=SimpleNamespace(ok=lambda: True),
        HandCommand=FakeHandCommand,
        HandCommandArray=FakeHandCommandArray,
        HandType=FakeHandType,
        MessageHeader=lambda: object(),
        QoSProfile=lambda **kwargs: kwargs,
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT="best_effort"),
        DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL="transient_local"),
    )


def test_hand_api_builds_left_and_right_gripper_commands():
    node = FakeNode()
    api = StandaloneHandAPI(node, ros=fake_bindings(), command_topic="/test/hand")

    message = api.build_command(0.25, 0.75)

    assert message.left_hand_type.value == 2
    assert message.right_hand_type.value == 2
    assert message.left_hands[0].name == "left_claw_joint"
    assert message.left_hands[0].position == pytest.approx(0.25)
    assert message.right_hands[0].name == "right_claw_joint"
    assert message.right_hands[0].position == pytest.approx(0.75)
    api.shutdown()


def test_hand_api_validates_positions_and_lifecycle():
    api = StandaloneHandAPI(FakeNode(), ros=fake_bindings(), command_topic="/test/hand")

    with pytest.raises(ValueError, match="0.0 到 1.0"):
        api.set_position("left", 1.1)
    with pytest.raises(ValueError, match="left、right 或 both"):
        api.set_position("middle", 0.5)

    api.shutdown()
    with pytest.raises(HandControlError, match="已关闭"):
        api.open()
