from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from x2_common.blocking_audio import BlockingPcmPlayer  # noqa: E402


class _AudioField:
    pass


class _AudioPlayback:
    def __init__(self) -> None:
        self.info = _AudioField()
        self.data = _AudioField()


class _Service:
    class Request:
        pass


def test_blocking_player_uses_shared_pcm_contract_and_releases_focus() -> None:
    msg_module = ModuleType("aimdk_msgs.msg")
    msg_module.AudioData = _AudioField  # type: ignore[attr-defined]
    msg_module.AudioInfo = _AudioField  # type: ignore[attr-defined]
    msg_module.AudioPlayback = _AudioPlayback  # type: ignore[attr-defined]
    msg_module.FocusRequester = _AudioField  # type: ignore[attr-defined]
    msg_module.FocusResponse = object  # type: ignore[attr-defined]
    srv_module = ModuleType("aimdk_msgs.srv")
    srv_module.RequestAudioFocus = _Service  # type: ignore[attr-defined]
    srv_module.AbandonAudioFocus = _Service  # type: ignore[attr-defined]
    publisher = SimpleNamespace(messages=[])
    publisher.publish = publisher.messages.append
    node = SimpleNamespace(
        create_publisher=lambda *_args: publisher,
        create_subscription=lambda *_args: object(),
        create_client=lambda *_args: object(),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: "stamp")
        ),
        get_logger=lambda: SimpleNamespace(warning=lambda _message: None),
    )

    with mock.patch.dict(
        sys.modules,
        {
            "aimdk_msgs": ModuleType("aimdk_msgs"),
            "aimdk_msgs.msg": msg_module,
            "aimdk_msgs.srv": srv_module,
        },
    ):
        player = BlockingPcmPlayer(
            node,
            playback_topic="/audio",
            package_name="test",
            focus_priority=6,
            chunk_ms=1,
            focus_response_topic="/focus",
            focus_request_service="/request",
            focus_release_service="/release",
        )

    focus_calls = []
    player._call_focus = lambda _rclpy, acquire: focus_calls.append(acquire) or True
    rclpy_mod = SimpleNamespace(spin_once=lambda *_args, **_kwargs: None)
    player.play(rclpy_mod, b"\x01\x00" * 20, stream_name="notice")

    assert focus_calls == [True, False]
    assert b"".join(message.data.data for message in publisher.messages[:-1]) == (
        b"\x01\x00" * 20
    )
    assert publisher.messages[0].info.sample_rate == 16_000
    assert publisher.messages[0].token_id.startswith("test-notice-")
    assert publisher.messages[-1].data.data == b""
