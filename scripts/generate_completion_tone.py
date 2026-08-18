#!/usr/bin/env python3
"""Generate the raw 16 kHz mono S16LE grasp-completion music cue."""

from __future__ import annotations

import math
from pathlib import Path
import struct


SAMPLE_RATE = 16_000
DURATION_SECONDS = 1.6
OUTPUT = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "x2_grasp"
    / "audio"
    / "grasp_complete.pcm"
)

# A short ascending C-major bell phrase followed by a soft resolving chord.
NOTES = (
    (0.00, 523.25, 0.42, 0.55),
    (0.22, 659.25, 0.44, 0.52),
    (0.44, 783.99, 0.48, 0.50),
    (0.72, 523.25, 0.72, 0.20),
    (0.72, 659.25, 0.72, 0.18),
    (0.72, 783.99, 0.72, 0.17),
    (0.72, 1046.50, 0.76, 0.36),
)


def bell_sample(age: float, frequency: float, duration: float) -> float:
    if age < 0.0 or age >= duration:
        return 0.0
    attack = min(1.0, age / 0.012)
    release = min(1.0, (duration - age) / 0.10)
    decay = math.exp(-2.2 * age / duration)
    phase = 2.0 * math.pi * frequency * age
    timbre = (
        math.sin(phase)
        + 0.22 * math.sin(2.0 * phase)
        + 0.07 * math.sin(3.0 * phase)
    )
    return attack * release * decay * timbre


def generate() -> bytes:
    samples = []
    for index in range(round(SAMPLE_RATE * DURATION_SECONDS)):
        now = index / SAMPLE_RATE
        value = sum(
            level * bell_sample(now - start, frequency, duration)
            for start, frequency, duration, level in NOTES
        )
        samples.append(value)

    peak = max(abs(value) for value in samples)
    scale = 0.72 * 32767.0 / peak
    return b"".join(
        struct.pack("<h", round(max(-32768, min(32767, value * scale))))
        for value in samples
    )


def main() -> int:
    OUTPUT.write_bytes(generate())
    print(
        f"Generated {OUTPUT} ({SAMPLE_RATE} Hz, mono, S16LE, "
        f"{DURATION_SECONDS:.1f} s)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
