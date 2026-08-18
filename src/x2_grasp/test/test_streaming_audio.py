from __future__ import annotations

from array import array
from pathlib import Path
import base64
import sys
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from x2_common.streaming_audio import (  # noqa: E402
    ABANDON_FOCUS_SERVICE,
    OUTPUT_CHUNK_BYTES,
    OUTPUT_SAMPLE_RATE,
    PLAYBACK_TOPIC,
    REQUEST_FOCUS_SERVICE,
    Pcm24kTo16kResampler,
    StreamingPcmPlayer,
)


def pcm_bytes(samples: list[int]) -> bytes:
    values = array("h", samples)
    if sys.byteorder != "little":
        values.byteswap()
    return values.tobytes()


class StreamingPcmPlayerTests(unittest.TestCase):
    def test_aimdk_playback_contract_is_fixed(self) -> None:
        self.assertEqual(PLAYBACK_TOPIC, "/aima/hal/audio/playback")
        self.assertEqual(
            REQUEST_FOCUS_SERVICE,
            "/aimdk_5Fmsgs/srv/RequestAudioFocus",
        )
        self.assertEqual(
            ABANDON_FOCUS_SERVICE,
            "/aimdk_5Fmsgs/srv/AbandonAudioFocus",
        )
        self.assertEqual(OUTPUT_SAMPLE_RATE, 16_000)
        self.assertEqual(OUTPUT_CHUNK_BYTES, 1_600)

    def test_100_ms_of_24k_audio_becomes_100_ms_at_16k(self) -> None:
        resampler = Pcm24kTo16kResampler()

        result = resampler.process(pcm_bytes([1_000] * 2_400))

        self.assertEqual(len(result), 1_600 * 2)
        self.assertEqual(result, pcm_bytes([1_000] * 1_600))

    def test_delta_boundaries_do_not_change_resampled_output(self) -> None:
        samples = [((index * 97) % 60_000) - 30_000 for index in range(2_400)]
        encoded = pcm_bytes(samples)
        whole = Pcm24kTo16kResampler().process(encoded)
        split_resampler = Pcm24kTo16kResampler()

        split = b"".join(
            split_resampler.process(encoded[start:end])
            for start, end in ((0, 701), (701, 2_003), (2_003, len(encoded)))
        )

        self.assertEqual(split, whole)

    def test_startup_pcm_is_split_into_playback_chunks(self) -> None:
        player = StreamingPcmPlayer.__new__(StreamingPcmPlayer)
        player._stopped = mock.Mock(is_set=lambda: False)
        player._active = mock.Mock()
        player.cancel = mock.Mock()
        player._put_fresh = mock.Mock()
        pcm = b"\x01\x00" * (OUTPUT_SAMPLE_RATE // 10)

        player.play_pcm_16k(pcm)

        player.cancel.assert_called_once_with()
        player._active.set.assert_called_once_with()
        self.assertEqual(player._put_fresh.call_count, 2)
        self.assertEqual(
            b"".join(call.args[0] for call in player._put_fresh.call_args_list),
            pcm,
        )

    def test_player_publishes_16k_chunks_and_changes_token_on_cancel(self) -> None:
        class AudioPlayback:
            def __init__(self) -> None:
                self.stamps = None
                self.info = SimpleNamespace()
                self.data = SimpleNamespace()
                self.pkg_name = ""
                self.token_id = ""

        class ServiceType:
            class Request:
                def __init__(self) -> None:
                    self.focus_requester = SimpleNamespace()

        class Publisher:
            def __init__(self) -> None:
                self.messages: list[AudioPlayback] = []

            def publish(self, message: AudioPlayback) -> None:
                self.messages.append(message)

        class Client:
            def __init__(self) -> None:
                self.ready = False
                self.requests: list[object] = []

            def service_is_ready(self) -> bool:
                return self.ready

            def call_async(self, request: object) -> SimpleNamespace:
                self.requests.append(request)
                return SimpleNamespace(add_done_callback=lambda _callback: None)

        class Node:
            def __init__(self) -> None:
                self.publisher = Publisher()
                self.clients: list[Client] = []

            def create_publisher(self, *_args: object) -> Publisher:
                return self.publisher

            def create_subscription(self, *_args: object) -> object:
                return object()

            def create_client(self, *_args: object) -> Client:
                client = Client()
                self.clients.append(client)
                return client

            def get_clock(self) -> SimpleNamespace:
                return SimpleNamespace(
                    now=lambda: SimpleNamespace(to_msg=lambda: "stamp")
                )

            def get_logger(self) -> SimpleNamespace:
                return SimpleNamespace(error=lambda _message: None)

        msg_module = ModuleType("aimdk_msgs.msg")
        msg_module.AudioPlayback = AudioPlayback  # type: ignore[attr-defined]
        msg_module.FocusResponse = object  # type: ignore[attr-defined]
        srv_module = ModuleType("aimdk_msgs.srv")
        srv_module.RequestAudioFocus = ServiceType  # type: ignore[attr-defined]
        srv_module.AbandonAudioFocus = ServiceType  # type: ignore[attr-defined]
        qos_module = ModuleType("rclpy.qos")
        qos_module.HistoryPolicy = SimpleNamespace(KEEP_LAST=1)  # type: ignore[attr-defined]
        qos_module.ReliabilityPolicy = SimpleNamespace(RELIABLE=1)  # type: ignore[attr-defined]
        qos_module.QoSProfile = lambda **kwargs: kwargs  # type: ignore[attr-defined]
        node = Node()

        with mock.patch.dict(
            sys.modules,
            {
                "aimdk_msgs": ModuleType("aimdk_msgs"),
                "aimdk_msgs.msg": msg_module,
                "aimdk_msgs.srv": srv_module,
                "rclpy": ModuleType("rclpy"),
                "rclpy.qos": qos_module,
            },
        ):
            player = StreamingPcmPlayer(
                node, tail_silence_ms=0, package_name="test"
            )
            player.begin_response()
            first_token = node.publisher.messages[-1].token_id
            with player._focus_lock:
                player._focus_granted = True
            player.add(base64.b64encode(pcm_bytes([1_000] * 2_400)).decode())
            player.finish_response()
            deadline = time.monotonic() + 1.0
            while (
                len([message for message in node.publisher.messages if message.data.data])
                < 2
                and time.monotonic() < deadline
            ):
                time.sleep(0.01)

            audio_messages = [
                message for message in node.publisher.messages if message.data.data
            ]
            self.assertEqual(len(audio_messages), 2)
            self.assertTrue(
                all(len(message.data.data) == OUTPUT_CHUNK_BYTES for message in audio_messages)
            )
            self.assertTrue(
                all(message.info.sample_rate == 16_000 for message in audio_messages)
            )
            self.assertTrue(
                all(message.info.sample_format == "S16LE" for message in audio_messages)
            )
            player.cancel()
            self.assertNotEqual(node.publisher.messages[-1].token_id, first_token)
            self.assertEqual(node.publisher.messages[-1].data.data, b"")
            player._abandon_client.ready = True
            late_focus = SimpleNamespace(
                result=lambda: SimpleNamespace(
                    reponse=SimpleNamespace(status=SimpleNamespace(value=1)),
                    focus_response=SimpleNamespace(focus_gain=True),
                )
            )
            player._on_focus_request_done(late_focus)
            self.assertEqual(len(player._abandon_client.requests), 1)
            player.close()


if __name__ == "__main__":
    unittest.main()
