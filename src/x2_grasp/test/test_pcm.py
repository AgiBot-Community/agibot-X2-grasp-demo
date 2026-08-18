from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from x2_common.pcm import (  # noqa: E402
    chunk_size_bytes,
    iter_chunks,
    populate_playback_message,
    validate_s16le_mono,
)


def test_pcm_validation_rejects_empty_and_partial_samples() -> None:
    with pytest.raises(ValueError):
        validate_s16le_mono(b"")
    with pytest.raises(ValueError):
        validate_s16le_mono(b"\x00")


def test_generated_completion_tone_has_expected_raw_pcm_format() -> None:
    pcm = (PROJECT_ROOT / "audio/grasp_complete.pcm").read_bytes()

    validate_s16le_mono(pcm)
    assert len(pcm) == round(16_000 * 2 * 1.6)
    assert any(pcm)


def test_chunks_preserve_pcm_and_sample_boundaries() -> None:
    pcm = bytes(range(12))

    chunks = list(iter_chunks(pcm, 5))

    assert [len(chunk) for chunk in chunks] == [4, 4, 4]
    assert b"".join(chunks) == pcm
    assert chunk_size_bytes(50) == 1600


def test_playback_message_uses_shared_aimdk_contract() -> None:
    message = SimpleNamespace(
        info=SimpleNamespace(),
        data=SimpleNamespace(),
    )

    populate_playback_message(
        message,
        stamp="now",
        pcm=b"\x01\x00",
        package_name="test",
        token_id="token",
    )

    assert message.stamps == "now"
    assert message.info.sample_rate == 16_000
    assert message.info.channels == 1
    assert message.info.sample_format == "S16LE"
    assert message.info.coding_format == "pcm"
    assert message.data.data == b"\x01\x00"
