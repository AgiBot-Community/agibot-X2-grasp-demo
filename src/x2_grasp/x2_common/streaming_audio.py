"""AIMDK audio-focus management and asynchronous PCM streaming."""

from __future__ import annotations

from array import array
import base64
import os
import queue
import sys
import threading
import time
from typing import Any
import uuid

from .pcm import (
    PCM_CHANNELS,
    PCM_SAMPLE_RATE,
    PCM_SAMPLE_WIDTH,
    iter_chunks,
    populate_playback_message,
    validate_s16le_mono,
)
from .concurrency import put_latest


PLAYBACK_TOPIC = "/aima/hal/audio/playback"
FOCUS_RESPONSE_TOPIC = "/aima/hal/audio/focus_response"
REQUEST_FOCUS_SERVICE = "/aimdk_5Fmsgs/srv/RequestAudioFocus"
ABANDON_FOCUS_SERVICE = "/aimdk_5Fmsgs/srv/AbandonAudioFocus"
INPUT_SAMPLE_RATE = 24_000
OUTPUT_SAMPLE_RATE = PCM_SAMPLE_RATE
OUTPUT_CHANNELS = PCM_CHANNELS
SAMPLE_WIDTH = PCM_SAMPLE_WIDTH
CHUNK_DURATION_MS = 50
OUTPUT_CHUNK_BYTES = (
    OUTPUT_SAMPLE_RATE * OUTPUT_CHANNELS * SAMPLE_WIDTH * CHUNK_DURATION_MS // 1_000
)


class Pcm24kTo16kResampler:
    """Stateful linear PCM resampler that preserves delta boundaries."""

    def __init__(self) -> None:
        self._samples: list[int] = []
        self._position = 0.0
        self._partial_byte = b""

    def process(self, pcm: bytes) -> bytes:
        pcm = self._partial_byte + pcm
        if len(pcm) % SAMPLE_WIDTH:
            self._partial_byte = pcm[-1:]
            pcm = pcm[:-1]
        else:
            self._partial_byte = b""
        if not pcm:
            return b""

        incoming = array("h")
        incoming.frombytes(pcm)
        if sys.byteorder != "little":
            incoming.byteswap()
        self._samples.extend(incoming)

        output = array("h")
        step = INPUT_SAMPLE_RATE / OUTPUT_SAMPLE_RATE
        while self._position + 1 < len(self._samples):
            index = int(self._position)
            fraction = self._position - index
            value = round(
                self._samples[index] * (1.0 - fraction)
                + self._samples[index + 1] * fraction
            )
            output.append(max(-32_768, min(32_767, value)))
            self._position += step

        consumed = int(self._position)
        if consumed:
            del self._samples[:consumed]
            self._position -= consumed

        if sys.byteorder != "little":
            output.byteswap()
        return output.tobytes()

    def reset(self) -> None:
        self._samples.clear()
        self._position = 0.0
        self._partial_byte = b""


class StreamingPcmPlayer:
    """Publish streaming PCM through the AIMDK audio interface."""

    def __init__(
        self,
        node: Any,
        tail_silence_ms: int = 300,
        priority: int = 6,
        package_name: str = "x2_grasp",
        playback_topic: str = PLAYBACK_TOPIC,
        focus_response_topic: str = FOCUS_RESPONSE_TOPIC,
        request_focus_service: str = REQUEST_FOCUS_SERVICE,
        abandon_focus_service: str = ABANDON_FOCUS_SERVICE,
    ) -> None:
        from aimdk_msgs.msg import AudioPlayback, FocusResponse
        from aimdk_msgs.srv import AbandonAudioFocus, RequestAudioFocus
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

        self._node = node
        self._AudioPlayback = AudioPlayback
        self._RequestAudioFocus = RequestAudioFocus
        self._AbandonAudioFocus = AbandonAudioFocus
        self._pkg_name = f"{package_name}_{os.getpid()}"
        self._priority = priority
        self._tail_seconds = max(0, tail_silence_ms) / 1_000
        self._resampler = Pcm24kTo16kResampler()
        self._pending = bytearray()
        self._chunks: queue.Queue[bytes] = queue.Queue(maxsize=200)
        self._stopped = threading.Event()
        self._active = threading.Event()
        self._accept_audio = threading.Event()
        self._focus_lock = threading.Lock()
        self._focus_granted = False
        self._focus_request_pending = False
        self._last_focus_request_at = 0.0
        self._last_publish_at = 0.0
        self._token_id = uuid.uuid4().hex

        self._publisher = node.create_publisher(AudioPlayback, playback_topic, 10)
        focus_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._focus_subscription = node.create_subscription(
            FocusResponse,
            focus_response_topic,
            self._on_focus_response,
            focus_qos,
        )
        self._request_client = node.create_client(
            RequestAudioFocus,
            request_focus_service,
        )
        self._abandon_client = node.create_client(
            AbandonAudioFocus,
            abandon_focus_service,
        )
        self._worker = threading.Thread(
            target=self._publish_loop,
            name=f"{package_name}-playback",
            daemon=True,
        )
        self._worker.start()

    def begin_response(self) -> None:
        self.cancel()
        self._accept_audio.set()

    def add(self, encoded_audio: str) -> None:
        if self._stopped.is_set() or not self._accept_audio.is_set():
            return
        try:
            pcm_24k = base64.b64decode(encoded_audio, validate=True)
        except (ValueError, TypeError):
            return
        pcm_16k = self._resampler.process(pcm_24k)
        if not pcm_16k:
            return

        self._active.set()
        self._pending.extend(pcm_16k)
        while len(self._pending) >= OUTPUT_CHUNK_BYTES:
            chunk = bytes(self._pending[:OUTPUT_CHUNK_BYTES])
            del self._pending[:OUTPUT_CHUNK_BYTES]
            self._put_fresh(chunk)

    def finish_response(self) -> None:
        self._accept_audio.clear()
        if self._pending:
            self._put_fresh(bytes(self._pending))
            self._pending.clear()

    def play_pcm_16k(self, pcm: bytes) -> None:
        """Play a complete mono S16LE/16 kHz PCM sound."""
        validate_s16le_mono(pcm)
        if self._stopped.is_set():
            return

        self.cancel()
        self._active.set()
        for chunk in iter_chunks(pcm, OUTPUT_CHUNK_BYTES):
            self._put_fresh(chunk)

    def cancel(self) -> None:
        self._clear(self._chunks)
        self._pending.clear()
        self._resampler.reset()
        self._accept_audio.clear()
        self._token_id = uuid.uuid4().hex
        self._publish(b"")
        self._active.clear()
        self._release_focus()

    def is_active(self) -> bool:
        return self._active.is_set()

    def close(self) -> None:
        if self._stopped.is_set():
            return
        self.cancel()
        self._stopped.set()
        self._worker.join(timeout=2.0)
        for destroy_name, entity in (
            ("destroy_subscription", self._focus_subscription),
            ("destroy_client", self._request_client),
            ("destroy_client", self._abandon_client),
            ("destroy_publisher", self._publisher),
        ):
            destroy = getattr(self._node, destroy_name, None)
            if destroy is not None:
                destroy(entity)

    def _put_fresh(self, chunk: bytes) -> None:
        put_latest(self._chunks, chunk)

    def _publish_loop(self) -> None:
        next_publish_at = time.monotonic()
        while not self._stopped.is_set():
            if self._chunks.empty():
                self._finish_after_tail()
                time.sleep(0.01)
                continue
            if not self._has_focus():
                self._request_focus()
                time.sleep(0.01)
                continue

            now = time.monotonic()
            if now < next_publish_at:
                time.sleep(min(0.01, next_publish_at - now))
                continue
            try:
                chunk = self._chunks.get_nowait()
            except queue.Empty:
                continue
            self._publish(chunk)
            self._last_publish_at = time.monotonic()
            duration = len(chunk) / (OUTPUT_SAMPLE_RATE * SAMPLE_WIDTH)
            next_publish_at = max(next_publish_at + duration, self._last_publish_at)

    def _publish(self, chunk: bytes) -> None:
        message = self._AudioPlayback()
        populate_playback_message(
            message,
            stamp=self._node.get_clock().now().to_msg(),
            pcm=chunk,
            package_name=self._pkg_name,
            token_id=self._token_id,
        )
        self._publisher.publish(message)

    def _request_focus(self) -> None:
        now = time.monotonic()
        with self._focus_lock:
            if self._focus_request_pending or now - self._last_focus_request_at < 1.0:
                return
            if not self._request_client.service_is_ready():
                self._last_focus_request_at = now
                return
            self._focus_request_pending = True
            self._last_focus_request_at = now

        request = self._RequestAudioFocus.Request()
        request.focus_requester.pkg_name = self._pkg_name
        request.focus_requester.priority = self._priority
        request.focus_requester.priority_weight = 0
        future = self._request_client.call_async(request)
        future.add_done_callback(self._on_focus_request_done)

    def _on_focus_request_done(self, future: Any) -> None:
        granted = False
        try:
            response = future.result()
            granted = bool(
                response
                and response.reponse.status.value == 1
                and response.focus_response.focus_gain
            )
        except Exception as exc:
            self._node.get_logger().error(f"申请音频焦点失败：{exc}")
        with self._focus_lock:
            self._focus_request_pending = False
            self._focus_granted = granted
        if granted and not self._active.is_set():
            self._release_focus()

    def _on_focus_response(self, message: Any) -> None:
        if message.pkg_name != self._pkg_name:
            return
        with self._focus_lock:
            self._focus_granted = bool(message.focus_gain)
        if not message.focus_gain:
            self._clear(self._chunks)
            self._pending.clear()
            self._resampler.reset()
            self._accept_audio.clear()
            self._active.clear()

    def _release_focus(self) -> None:
        with self._focus_lock:
            if not self._focus_granted:
                return
        if not self._abandon_client.service_is_ready():
            return
        with self._focus_lock:
            self._focus_granted = False
        request = self._AbandonAudioFocus.Request()
        request.focus_requester.pkg_name = self._pkg_name
        request.focus_requester.priority = self._priority
        request.focus_requester.priority_weight = 0
        self._abandon_client.call_async(request)

    def _has_focus(self) -> bool:
        with self._focus_lock:
            return self._focus_granted

    def _finish_after_tail(self) -> None:
        if not self._active.is_set():
            self._release_focus()
            return
        if self._accept_audio.is_set() or self._pending:
            return
        if not self._last_publish_at:
            return
        if time.monotonic() - self._last_publish_at < self._tail_seconds:
            return
        self._active.clear()
        self._release_focus()

    @staticmethod
    def _clear(items: queue.Queue[bytes]) -> None:
        with items.mutex:
            items.queue.clear()
