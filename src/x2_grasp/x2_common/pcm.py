"""Raw PCM validation, chunking, and AIMDK message helpers."""

from collections.abc import Iterator
from typing import Any


PCM_SAMPLE_RATE = 16_000
PCM_CHANNELS = 1
PCM_SAMPLE_WIDTH = 2


def chunk_size_bytes(duration_ms: int) -> int:
    """Return a whole-sample chunk size for mono 16 kHz S16LE audio."""
    return max(
        PCM_SAMPLE_WIDTH,
        PCM_SAMPLE_RATE * PCM_SAMPLE_WIDTH * max(1, duration_ms) // 1_000,
    )


def validate_s16le_mono(pcm: bytes) -> None:
    """Reject empty or partial-sample PCM buffers."""
    if not pcm or len(pcm) % PCM_SAMPLE_WIDTH:
        raise ValueError("PCM must contain complete, non-empty S16LE samples")


def iter_chunks(pcm: bytes, chunk_bytes: int) -> Iterator[bytes]:
    """Yield a PCM buffer in stable, whole-sample chunks."""
    validate_s16le_mono(pcm)
    chunk_bytes = max(PCM_SAMPLE_WIDTH, chunk_bytes)
    chunk_bytes -= chunk_bytes % PCM_SAMPLE_WIDTH
    for offset in range(0, len(pcm), chunk_bytes):
        yield pcm[offset:offset + chunk_bytes]


def populate_playback_message(
    message: Any,
    *,
    stamp: Any,
    pcm: bytes,
    package_name: str,
    token_id: str,
) -> Any:
    """Populate the common AIMDK AudioPlayback PCM contract."""
    message.stamps = stamp
    message.info.channels = PCM_CHANNELS
    message.info.sample_rate = PCM_SAMPLE_RATE
    message.info.size = len(pcm)
    message.info.sample_format = "S16LE"
    message.info.coding_format = "pcm"
    message.data.data = pcm
    message.pkg_name = package_name
    message.token_id = token_id
    return message
