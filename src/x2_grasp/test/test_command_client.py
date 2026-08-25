from types import SimpleNamespace

from x2_arm.command_client import CppCommandClient


class FakeGoal:
    ARM_TRAJECTORY = 1
    HAND_COMMAND = 2

    def __init__(self):
        self.kind = 0
        self.start_arm_pos = []
        self.goal_arm_pos = []
        self.duration = 0.0
        self.hand = ""
        self.left_hand_position = 0.0
        self.right_hand_position = 0.0


def _client():
    client = CppCommandClient.__new__(CppCommandClient)
    client.action_type = SimpleNamespace(Goal=FakeGoal)
    client._execute = lambda goal, cancel: (goal, cancel)
    return client


def test_cpp_command_client_builds_complete_arm_segment():
    client = _client()
    start = [float(index) for index in range(14)]
    goal = [value + 0.5 for value in start]
    cancel = lambda: False

    message, callback = client.execute_arm(start, goal, 2.5, cancel)

    assert message.kind == FakeGoal.ARM_TRAJECTORY
    assert message.start_arm_pos == start
    assert message.goal_arm_pos == goal
    assert message.duration == 2.5
    assert callback is cancel


def test_cpp_command_client_builds_targeted_hand_command():
    client = _client()

    message, _ = client.execute_hand("right", None, 0.25, 1.0)

    assert message.kind == FakeGoal.HAND_COMMAND
    assert message.hand == "right"
    assert message.left_hand_position == 0.0
    assert message.right_hand_position == 0.25
    assert message.duration == 1.0
