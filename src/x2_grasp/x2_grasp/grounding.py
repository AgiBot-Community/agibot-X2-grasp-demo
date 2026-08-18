"""Pure helpers for prompts, compressed images, and grounding responses."""

from __future__ import annotations

from dataclasses import dataclass
import re
import struct

from .target_catalog import default_target_catalog

_DEFAULT_CATALOG = default_target_catalog()
TARGETS = _DEFAULT_CATALOG.descriptions
TARGET_ALIASES = dict(_DEFAULT_CATALOG.aliases)

BBOX_PATTERN = re.compile(
    r"<bbox>\s*"
    r"(-?\d+(?:\.\d+)?)[,\s]+"
    r"(-?\d+(?:\.\d+)?)[,\s]+"
    r"(-?\d+(?:\.\d+)?)[,\s]+"
    r"(-?\d+(?:\.\d+)?)\s*</bbox>",
    re.IGNORECASE,
)

ARTIFACT_COMPONENT_PATTERN = re.compile(r"[^A-Za-z0-9_-]+")


@dataclass(frozen=True)
class NormalizedBox:
    x_min: int
    y_min: int
    x_max: int
    y_max: int

    def to_pixels(self, width: int, height: int) -> dict[str, int] | None:
        if width <= 0 or height <= 0:
            return None
        return {
            "x_min": min(width - 1, round(self.x_min * width / 1000)),
            "y_min": min(height - 1, round(self.y_min * height / 1000)),
            "x_max": min(width - 1, round(self.x_max * width / 1000)),
            "y_max": min(height - 1, round(self.y_max * height / 1000)),
        }

    def as_dict(self) -> dict[str, int]:
        return {
            "x_min": self.x_min,
            "y_min": self.y_min,
            "x_max": self.x_max,
            "y_max": self.y_max,
        }


def normalize_target(value: str) -> str | None:
    return _DEFAULT_CATALOG.normalize(value)


def safe_artifact_component(value: str, *, fallback: str = "request") -> str:
    """Return a bounded filename component for externally supplied identifiers."""
    sanitized = ARTIFACT_COMPONENT_PATTERN.sub("_", value.strip()).strip("_")
    return sanitized[:80] or fallback


def build_prompt(target_zh: str) -> str:
    return (
        f"请检测并框出图像中所有的{target_zh}。"
        "只输出边界框，不要解释；每个目标使用一个"
        "<bbox>x_min y_min x_max y_max</bbox>，坐标归一化到1000×1000。"
        "允许边界框不完全覆盖物体，但边界框的中心点必须位于物体中心点略偏下的位置。"
        "如果没有找到目标，只输出 NONE。"
    )


def parse_bboxes(content: str) -> list[NormalizedBox]:
    boxes: list[NormalizedBox] = []
    for match in BBOX_PATTERN.finditer(content):
        values = tuple(round(float(value)) for value in match.groups())
        x_min, y_min, x_max, y_max = values
        if any(value < 0 or value > 1000 for value in values):
            continue
        if x_min >= x_max or y_min >= y_max:
            continue
        boxes.append(NormalizedBox(x_min, y_min, x_max, y_max))
    return boxes


def select_box(
    boxes: list[NormalizedBox], policy: str = "largest"
) -> NormalizedBox | None:
    """Select the single object used by one grasp command."""
    if not boxes:
        return None
    if policy == "first":
        return boxes[0]
    if policy == "largest":
        return max(
            boxes,
            key=lambda box: (box.x_max - box.x_min) * (box.y_max - box.y_min),
        )
    raise ValueError("bbox_selection must be 'largest' or 'first'")


def detect_mime_type(data: bytes, format_hint: str = "") -> str:
    if data.startswith(b"\xff\xd8\xff"):
        return "image/jpeg"
    if data.startswith(b"\x89PNG\r\n\x1a\n"):
        return "image/png"
    if data.startswith((b"GIF87a", b"GIF89a")):
        return "image/gif"
    if data.startswith(b"RIFF") and data[8:12] == b"WEBP":
        return "image/webp"

    hint = format_hint.lower()
    if "png" in hint:
        return "image/png"
    if "webp" in hint:
        return "image/webp"
    return "image/jpeg"


def image_dimensions(data: bytes) -> tuple[int, int] | None:
    if data.startswith(b"\x89PNG\r\n\x1a\n") and len(data) >= 24:
        return struct.unpack(">II", data[16:24])
    if data.startswith((b"GIF87a", b"GIF89a")) and len(data) >= 10:
        return struct.unpack("<HH", data[6:10])
    if data.startswith(b"\xff\xd8"):
        return _jpeg_dimensions(data)
    return None


def _jpeg_dimensions(data: bytes) -> tuple[int, int] | None:
    index = 2
    start_of_frame = {
        0xC0,
        0xC1,
        0xC2,
        0xC3,
        0xC5,
        0xC6,
        0xC7,
        0xC9,
        0xCA,
        0xCB,
        0xCD,
        0xCE,
        0xCF,
    }
    while index + 3 < len(data):
        if data[index] != 0xFF:
            index += 1
            continue
        while index < len(data) and data[index] == 0xFF:
            index += 1
        if index >= len(data):
            break
        marker = data[index]
        index += 1
        if marker in {0xD8, 0xD9}:
            continue
        if index + 2 > len(data):
            break
        segment_length = struct.unpack(">H", data[index : index + 2])[0]
        if segment_length < 2 or index + segment_length > len(data):
            break
        if marker in start_of_frame and segment_length >= 7:
            height, width = struct.unpack(">HH", data[index + 3 : index + 7])
            return width, height
        index += segment_length
    return None
