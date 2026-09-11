import sys
from types import ModuleType, SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from x2_arm.config import ArmSide

pytest.importorskip("rclpy")

from x2_grasp.grasp_node import (
    acquire_grounding_target_with_retry,
    execute_grasp,
    ik_seed_candidates,
    solve_cartesian_segment,
    solve_high_retract_segment,
)
from x2_grasp.audio_player import GraspAudioPlayer
from x2_grasp.grasp_errors import GraspCancelled


class _RetryLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


@pytest.fixture(autouse=True)
def _stub_aimdk_audio_interfaces(monkeypatch):
    class Message:
        pass

    class AudioPlayback:
        def __init__(self):
            self.info = Message()
            self.data = Message()

    class Service:
        class Request:
            pass

    package = ModuleType("aimdk_msgs")
    messages = ModuleType("aimdk_msgs.msg")
    services = ModuleType("aimdk_msgs.srv")
    messages.AudioData = Message
    messages.AudioInfo = Message
    messages.AudioPlayback = AudioPlayback
    messages.FocusRequester = Message
    messages.FocusResponse = Message
    services.AbandonAudioFocus = Service
    services.RequestAudioFocus = Service
    package.msg = messages
    package.srv = services
    monkeypatch.setitem(sys.modules, "aimdk_msgs", package)
    monkeypatch.setitem(sys.modules, "aimdk_msgs.msg", messages)
    monkeypatch.setitem(sys.modules, "aimdk_msgs.srv", services)


class _RetryNode:
    def get_logger(self):
        return _RetryLogger()

    @staticmethod
    def get_clock():
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))


class _RetryPublisher:
    def __init__(self):
        self.attempt = 0
        self.request_id = None

    def publish(self, message):
        self.attempt += 1
        self.request_id = message.request_id


class _RetryResultListener:
    def __init__(self, publisher, failures):
        self.publisher = publisher
        self.failures = failures
        self.delivered = set()

    def clear(self):
        pass

    def pop_result(self):
        attempt = self.publisher.attempt
        if attempt in self.delivered or attempt == 0:
            return None
        self.delivered.add(attempt)
        if attempt <= len(self.failures):
            return {
                "success": False,
                "error": self.failures[attempt - 1],
                "request_id": self.publisher.request_id,
            }
        return {
            "success": True,
            "request_id": self.publisher.request_id,
            "boxes": [{}],
            "image_stamp": {"sec": attempt, "nanosec": 0},
        }


class _StaleThenCurrentResultListener(_RetryResultListener):
    def __init__(self, publisher):
        super().__init__(publisher, [])
        self.returned_stale = False

    def pop_result(self):
        if not self.returned_stale:
            self.returned_stale = True
            return {"success": True, "request_id": "stale", "boxes": [{}]}
        return super().pop_result()


class _RetryTargetListener:
    def __init__(self, publisher):
        self.publisher = publisher

    def clear(self):
        pass

    def take_for_stamp(self, stamp):
        if stamp == (self.publisher.attempt, 0):
            return "grounding", [0.3, -0.2, 0.2], "base_link", f"{stamp[0]}.0"
        return None


class _RetryLocalizerListener:
    def __init__(self, publisher, fail_attempts=()):
        self.publisher = publisher
        self.fail_attempts = set(fail_attempts)
        self.delivered = set()

    def clear(self):
        pass

    def pop_status(self):
        attempt = self.publisher.attempt
        if attempt not in self.fail_attempts or attempt in self.delivered:
            return None
        self.delivered.add(attempt)
        return {
            "success": False,
            "error": "no valid depth sample",
            "image_stamp": {"sec": attempt, "nanosec": 0},
        }


class _RetryRclpy:
    @staticmethod
    def spin_once(_node, timeout_sec=0.0):
        del timeout_sec


def test_audio_player_streams_matching_pcm_and_releases_focus(tmp_path):
    class Message:
        pass

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Clock:
        class Now:
            @staticmethod
            def to_msg():
                return Time()

        @staticmethod
        def now():
            return Clock.Now()

    class Node:
        def __init__(self):
            self.publisher = Publisher()

        def create_publisher(self, *_args):
            return self.publisher

        @staticmethod
        def create_subscription(*_args):
            return Message()

        @staticmethod
        def create_client(*_args):
            return Message()

        @staticmethod
        def get_clock():
            return Clock()

        @staticmethod
        def get_logger():
            return _RetryLogger()

    pcm = bytes(range(16))
    paths = {}
    for target in ("cup", "bread", "bottle"):
        path = tmp_path / f"{target}.pcm"
        path.write_bytes(pcm)
        paths[target] = str(path)
    args = SimpleNamespace(
        audio_playback_topic="/audio",
        audio_pkg_name="test",
        audio_focus_priority=6,
        audio_chunk_ms=1,
        audio_focus_response_topic="/focus",
        audio_focus_request_service="/request",
        audio_focus_release_service="/release",
        cup_pcm_path=paths["cup"],
        bread_pcm_path=paths["bread"],
        bottle_pcm_path=paths["bottle"],
    )
    node = Node()
    player = GraspAudioPlayer(node, args)
    focus_calls = []
    player._call_focus = lambda _rclpy, acquire: focus_calls.append(acquire) or True

    player.play(_RetryRclpy(), "bottle")

    assert focus_calls == [True, False]
    assert bytes(node.publisher.messages[0].data.data) == pcm
    assert bytes(node.publisher.messages[-1].data.data) == b""
    assert node.publisher.messages[0].info.sample_rate == 16000
    assert node.publisher.messages[0].info.sample_format == "S16LE"
    assert node.publisher.messages[0].info.coding_format == "pcm"


def test_audio_player_retries_release_when_service_temporarily_fails(tmp_path):
    class Node:
        def __init__(self):
            self.publisher = SimpleNamespace(publish=lambda _message: None)
            self.logger = SimpleNamespace(warning=lambda _message: None)

        def create_publisher(self, *_args):
            return self.publisher

        @staticmethod
        def create_subscription(*_args):
            return object()

        @staticmethod
        def create_client(*_args):
            return object()

        @staticmethod
        def get_clock():
            return SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=lambda: Time())
            )

        def get_logger(self):
            return self.logger

    pcm_paths = {}
    for target in ("cup", "bread", "bottle"):
        path = tmp_path / f"{target}.pcm"
        path.write_bytes(b"\x00\x00")
        pcm_paths[target] = str(path)
    args = SimpleNamespace(
        audio_playback_topic="/audio",
        audio_pkg_name="x2_grasp",
        audio_focus_priority=10,
        audio_chunk_ms=50,
        audio_focus_response_topic="/focus",
        audio_focus_request_service="/request",
        audio_focus_release_service="/release",
        cup_pcm_path=pcm_paths["cup"],
        bread_pcm_path=pcm_paths["bread"],
        bottle_pcm_path=pcm_paths["bottle"],
    )
    player = GraspAudioPlayer(Node(), args)
    calls = []

    def release_after_retry(_rclpy, acquire):
        calls.append(acquire)
        if len(calls) < 3:
            raise RuntimeError("temporary failure")
        return True

    player._call_focus = release_after_retry

    assert player.release(_RetryRclpy()) is True
    assert calls == [False, False, False]


def test_audio_player_resolves_relative_pcm_from_package():
    resolved = GraspAudioPlayer._resolve_pcm_path("cup.pcm")

    assert resolved.as_posix().endswith("x2_grasp/audio/cup.pcm")


def _retry_args(attempts=3):
    return SimpleNamespace(
        grounding_retry_attempts=attempts,
        grounding_retry_delay=0.0,
        grounding_attempt_timeout=0.02,
        localization_result_timeout=0.01,
    )


def test_grounding_retries_api_error_before_returning_target():
    publisher = _RetryPublisher()
    result = _RetryResultListener(publisher, ["network error"])

    target = acquire_grounding_target_with_retry(
        _RetryNode(), "cup", publisher, _RetryTargetListener(publisher),
        result, _RetryLocalizerListener(publisher), _RetryRclpy(),
        _retry_args())

    assert target[0] == "grounding"
    assert publisher.attempt == 2


def test_grounding_drains_stale_result_without_losing_current_result():
    publisher = _RetryPublisher()

    target = acquire_grounding_target_with_retry(
        _RetryNode(), "cup", publisher, _RetryTargetListener(publisher),
        _StaleThenCurrentResultListener(publisher),
        _RetryLocalizerListener(publisher), _RetryRclpy(), _retry_args())

    assert target[0] == "grounding"
    assert publisher.attempt == 1


def test_grounding_retries_depth_error_before_returning_target():
    publisher = _RetryPublisher()
    result = _RetryResultListener(publisher, [])

    target = acquire_grounding_target_with_retry(
        _RetryNode(), "bottle", publisher, _RetryTargetListener(publisher),
        result, _RetryLocalizerListener(publisher, {1}), _RetryRclpy(),
        _retry_args())

    assert target[0] == "grounding"
    assert publisher.attempt == 2


def test_grounding_reports_all_failures_after_retry_exhaustion():
    publisher = _RetryPublisher()
    result = _RetryResultListener(publisher, ["dns", "http", "empty"])

    with pytest.raises(RuntimeError, match="第 3 次") as error:
        acquire_grounding_target_with_retry(
            _RetryNode(), "bread", publisher, _RetryTargetListener(publisher),
            result, _RetryLocalizerListener(publisher), _RetryRclpy(),
            _retry_args())

    assert "dns" in str(error.value)
    assert "http" in str(error.value)
    assert publisher.attempt == 3


class _Solver:
    @staticmethod
    def clip_arm_pos(arm_pos):
        return list(arm_pos)

    @staticmethod
    def fk_xyz(_side, arm_pos):
        return list(arm_pos[:3])

    @staticmethod
    def fk_axis(_side, _arm_pos):
        return [1.0, 0.0, 0.0]


class _Node:
    def __init__(self):
        self.solver = _Solver()
        self.calls = []
        self.node = SimpleNamespace(get_logger=lambda: _Logger())

    def close_gripper(self, hand, seconds, **_kwargs):
        self.calls.append(("close", hand, seconds))

    def open_gripper(self, hand, seconds, **_kwargs):
        self.calls.append(("open", hand, seconds))

    def set_gripper_position(self, hand, position, seconds, **_kwargs):
        self.calls.append(("grip", hand, position, seconds))

    def publish_trajectory(
        self, start, goal, duration, *, cancel_requested=lambda: False
    ):
        assert not cancel_requested()
        self.calls.append(("trajectory", list(start), list(goal), duration))

    def read_current_arm_pos(self):
        trajectories = [call for call in self.calls if call[0] == "trajectory"]
        return list(trajectories[-1][2])


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


def test_execute_grasp_uses_high_retract_after_lift():
    node = _Node()
    current = [0.0] * 14
    retract = [0.0] * 7 + [-0.35, -0.45, 0.0, -1.0, 0.0, 0.15, 0.0]
    pre = [0.1] * 14
    grasp = [0.2] * 14
    raised = [0.3] * 14
    high_retract = [0.4] * 14
    plan = SimpleNamespace(
        side=ArmSide.LEFT,
        retract_arm_pos=retract,
        retract_xyz=retract[:3],
        pre_grasp=SimpleNamespace(arm_pos=pre),
        pre_grasp_xyz=pre[:3],
        approach_steps=[SimpleNamespace(arm_pos=grasp)],
        grasp=SimpleNamespace(arm_pos=grasp),
        grasp_xyz=grasp[:3],
        lift_steps=[SimpleNamespace(arm_pos=raised)],
        post_grasp=SimpleNamespace(arm_pos=raised),
        post_grasp_xyz=raised[:3],
        high_retract=SimpleNamespace(arm_pos=high_retract),
        high_retract_steps=[SimpleNamespace(arm_pos=high_retract)],
        return_segments=[("return_lowering", [SimpleNamespace(arm_pos=retract)])],
        high_retract_xyz=high_retract[:3],
        achieved_lift=0.06,
    )
    args = SimpleNamespace(
        cup_grip_close_position=0.1,
        post_grasp_lift=0.06,
        lift_step=0.02,
        initial_close_seconds=2.0,
        open_seconds=0.5,
        grip_close_seconds=2.0,
        duration=4.0,
        approach_duration=3.0,
    )

    execute_grasp(node, current, plan, args, "cup")

    trajectories = [call for call in node.calls if call[0] == "trajectory"]
    assert [(call[1], call[2]) for call in trajectories] == [
        (current, retract),
        (retract, pre),
        (pre, grasp),
        (grasp, raised),
        (raised, high_retract),
        (high_retract, retract),
        (retract, current),
    ]
    hand_calls = [call for call in node.calls if call[0] in {"close", "open", "grip"}]
    assert all(call[1] == "left" for call in hand_calls)


def test_ik_seed_candidates_only_perturb_right_arm():
    node = _Node()
    primary = [float(value) for value in range(14)]
    retract = list(primary)
    retract[7:] = [0.0] * 7

    candidates = list(ik_seed_candidates(node, primary, retract, 0.12))

    assert len(candidates) == 8
    assert all(candidate[:7] == primary[:7] for candidate in candidates)
    assert candidates[0] == primary
    assert candidates[1] == retract


def test_cartesian_segment_splits_long_motion(monkeypatch):
    calls = []

    def fake_solve(_node, xyz, current, _retract, _args, _description, **_kwargs):
        calls.append(list(xyz))
        return SimpleNamespace(arm_pos=list(current))

    monkeypatch.setattr("x2_grasp.grasp_planner.solve_grasp_axis", fake_solve)
    args = SimpleNamespace(cartesian_step=0.02)
    start_result = SimpleNamespace(arm_pos=[0.0] * 14)

    results = solve_cartesian_segment(
        _Node(), [0.20, -0.2, 0.30], [0.25, -0.2, 0.30],
        start_result, [0.0] * 14, args, "test")

    assert len(results) == 3
    assert calls[-1] == pytest.approx([0.25, -0.2, 0.30])
    assert all(point[2] == pytest.approx(0.30) for point in calls)


def test_high_retract_keeps_successful_prefix_when_later_step_fails(monkeypatch):
    calls = []

    def fake_solve(_node, xyz, current, _retract, _args, _description,
                   **kwargs):
        calls.append((list(xyz), dict(kwargs)))
        if len(calls) == 2:
            raise RuntimeError("unreachable")
        return SimpleNamespace(arm_pos=list(current), target_xyz=list(xyz))

    monkeypatch.setattr("x2_grasp.grasp_planner.solve_grasp_axis", fake_solve)
    args = SimpleNamespace(
        cartesian_step=0.02,
        high_retract_position_tolerance=0.005,
        high_retract_axis_tolerance=0.05,
    )
    start_result = SimpleNamespace(arm_pos=[0.0] * 14)

    results = solve_high_retract_segment(
        _Node(), [0.26, -0.2, 0.32], [0.20, -0.2, 0.32],
        start_result, [0.0] * 14, args, "retract")

    assert len(results) == 1
    assert calls[0][0][2] == pytest.approx(0.32)
    assert calls[0][1]["approximate_position_tolerance"] == pytest.approx(0.005)


def test_execute_grasp_can_finish_without_high_retract():
    node = _Node()
    current = [0.0] * 14
    retract = [0.1] * 14
    pre = [0.2] * 14
    grasp = [0.3] * 14
    raised = [0.4] * 14
    plan = SimpleNamespace(
        side=ArmSide.RIGHT,
        retract_arm_pos=retract,
        pre_grasp=SimpleNamespace(arm_pos=pre),
        approach_steps=[SimpleNamespace(arm_pos=grasp)],
        grasp=SimpleNamespace(arm_pos=grasp),
        lift_steps=[SimpleNamespace(arm_pos=raised)],
        post_grasp=SimpleNamespace(arm_pos=raised),
        high_retract_steps=[],
        achieved_lift=0.06,
        return_segments=[("return_lowering", [SimpleNamespace(arm_pos=retract)])],
    )
    args = SimpleNamespace(
        bottle_grip_close_position=0.1,
        post_grasp_lift=0.06,
        lift_step=0.02,
        initial_close_seconds=2.0,
        open_seconds=0.5,
        grip_close_seconds=2.0,
        duration=4.0,
        approach_duration=3.0,
    )

    execute_grasp(node, current, plan, args, "bottle")

    trajectories = [call for call in node.calls if call[0] == "trajectory"]
    assert len(trajectories) == 6
    assert trajectories[-2][1:3] == (raised, retract)
    assert trajectories[-1][2] == current


def test_execute_grasp_stops_before_motion_when_action_is_canceled():
    node = _Node()
    plan = SimpleNamespace(
        side=ArmSide.RIGHT,
        retract_arm_pos=[0.1] * 14,
        pre_grasp=SimpleNamespace(arm_pos=[0.2] * 14),
        approach_steps=[SimpleNamespace(arm_pos=[0.3] * 14)],
        grasp=SimpleNamespace(arm_pos=[0.3] * 14),
        lift_steps=[SimpleNamespace(arm_pos=[0.4] * 14)],
        post_grasp=SimpleNamespace(arm_pos=[0.4] * 14),
        high_retract_steps=[],
        achieved_lift=0.06,
        return_segments=[],
    )
    args = SimpleNamespace(
        cup_grip_close_position=0.1,
        lift_step=0.02,
        initial_close_seconds=2.0,
        open_seconds=0.5,
        grip_close_seconds=2.0,
        duration=4.0,
        approach_duration=3.0,
    )

    with pytest.raises(GraspCancelled):
        execute_grasp(
            node,
            [0.0] * 14,
            plan,
            args,
            "cup",
            cancel_requested=lambda: True,
        )

    assert node.calls == []
