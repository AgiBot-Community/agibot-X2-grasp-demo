"""Blocking AIMDK playback for complete raw PCM sounds."""

from __future__ import annotations

import time

from .pcm import (
    PCM_SAMPLE_RATE,
    PCM_SAMPLE_WIDTH,
    chunk_size_bytes,
    iter_chunks,
    populate_playback_message,
    validate_s16le_mono,
)

AUDIO_SAMPLE_RATE = PCM_SAMPLE_RATE
AUDIO_BYTES_PER_SAMPLE = PCM_SAMPLE_WIDTH
FOCUS_CALL_ATTEMPTS = 8
FOCUS_RELEASE_ATTEMPTS = 3


class BlockingPcmPlayer:
    """Play complete S16LE/16 kHz/mono PCM through the AIMDK interface."""

    def __init__(
        self,
        node,
        *,
        playback_topic,
        package_name,
        focus_priority,
        chunk_ms,
        focus_response_topic,
        focus_request_service,
        focus_release_service,
    ):
        from aimdk_msgs.msg import (
            AudioData,
            AudioInfo,
            AudioPlayback,
            FocusRequester,
            FocusResponse,
        )
        from aimdk_msgs.srv import AbandonAudioFocus, RequestAudioFocus

        self.node = node
        self._AudioData = AudioData
        self._AudioInfo = AudioInfo
        self._AudioPlayback = AudioPlayback
        self._FocusRequester = FocusRequester
        self._RequestAudioFocus = RequestAudioFocus
        self._AbandonAudioFocus = AbandonAudioFocus
        self.topic = playback_topic
        self.pkg_name = package_name
        self.priority = focus_priority
        self.chunk_ms = chunk_ms
        self.focus = False
        self.focus_force = True
        self.publisher = node.create_publisher(AudioPlayback, self.topic, 10)
        self.focus_subscription = node.create_subscription(
            FocusResponse, focus_response_topic, self._on_focus, 10)
        self.request_client = node.create_client(
            RequestAudioFocus, focus_request_service)
        self.release_client = node.create_client(
            AbandonAudioFocus, focus_release_service)

    def _on_focus(self, message):
        if message.pkg_name == self.pkg_name:
            self.focus_force = bool(message.focus_gain)

    def _call_focus(self, rclpy_mod, acquire):
        client = self.request_client if acquire else self.release_client
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("AimDK audio focus service is unavailable")
        request_type = self._RequestAudioFocus if acquire else self._AbandonAudioFocus
        request = request_type.Request()
        requester = self._FocusRequester()
        requester.pkg_name = self.pkg_name
        requester.priority = self.priority
        requester.priority_weight = 100
        request.focus_requester = requester

        for _attempt in range(FOCUS_CALL_ATTEMPTS):
            future = client.call_async(request)
            rclpy_mod.spin_until_future_complete(
                self.node, future, timeout_sec=0.25)
            if future.done() and future.result() is not None:
                response = future.result()
                if response.reponse.status.value == 1:
                    self.focus = bool(response.focus_response.focus_gain)
                    return self.focus if acquire else True
                self.node.get_logger().warning(
                    "Audio focus rejected: "
                    f"status={response.reponse.status.value} "
                    f"message={response.reponse.message}")
            time.sleep(0.05)
        raise RuntimeError("AimDK audio focus request timed out")

    def _publish_end_of_stream(self):
        message = self._AudioPlayback()
        message.info = self._AudioInfo()
        message.data = self._AudioData()
        populate_playback_message(
            message,
            stamp=self.node.get_clock().now().to_msg(),
            pcm=b"",
            package_name=self.pkg_name,
            token_id="",
        )
        self.publisher.publish(message)

    def release(self, rclpy_mod, *, raise_on_failure=False):
        """End the stream and retry releasing any focus held by this package."""
        try:
            self._publish_end_of_stream()
        except Exception as error:  # noqa: BLE001 - focus still must be released
            self.node.get_logger().warning(
                f"Failed to publish audio end-of-stream marker: {error}"
            )
        last_error = None
        for attempt in range(1, FOCUS_RELEASE_ATTEMPTS + 1):
            try:
                self._call_focus(rclpy_mod, False)
                self.focus = False
                self.focus_force = False
                return True
            except RuntimeError as error:
                last_error = error
                self.node.get_logger().warning(
                    "Failed to release audio focus "
                    f"(attempt {attempt}/{FOCUS_RELEASE_ATTEMPTS}): {error}"
                )
                time.sleep(0.1)
        self.focus = False
        self.focus_force = False
        if raise_on_failure:
            raise RuntimeError(
                f"failed to release AimDK audio focus: {last_error}"
            ) from last_error
        return False

    def recover_stale_focus(self, rclpy_mod):
        """Release a focus lease left by an earlier abnormal process exit."""
        return self.release(rclpy_mod, raise_on_failure=False)

    def play(self, rclpy_mod, pcm, *, stream_name="pcm"):
        try:
            validate_s16le_mono(pcm)
        except ValueError as error:
            raise RuntimeError(str(error)) from error

        self.focus_force = True
        if not self._call_focus(rclpy_mod, True):
            raise RuntimeError("AimDK did not grant audio focus")
        chunk_bytes = chunk_size_bytes(self.chunk_ms)
        token = f"{self.pkg_name}-{stream_name}-{time.monotonic_ns()}"
        try:
            for chunk in iter_chunks(pcm, chunk_bytes):
                if not self.focus_force:
                    raise RuntimeError("audio focus was lost during announcement")
                message = self._AudioPlayback()
                message.info = self._AudioInfo()
                message.data = self._AudioData()
                populate_playback_message(
                    message,
                    stamp=self.node.get_clock().now().to_msg(),
                    pcm=chunk,
                    package_name=self.pkg_name,
                    token_id=token,
                )
                self.publisher.publish(message)
                rclpy_mod.spin_once(self.node, timeout_sec=self.chunk_ms / 1000.0)

            deadline = time.monotonic() + self.chunk_ms / 1000.0
            while time.monotonic() < deadline:
                rclpy_mod.spin_once(self.node, timeout_sec=0.01)
        finally:
            self.release(rclpy_mod, raise_on_failure=True)
