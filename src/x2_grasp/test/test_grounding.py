import struct

from x2_grasp.grounding import (
    TARGETS,
    build_prompt,
    detect_mime_type,
    image_dimensions,
    normalize_target,
    parse_bboxes,
    safe_artifact_component,
    select_box,
)


def test_target_commands_are_ascii_and_map_to_chinese() -> None:
    assert normalize_target(" cup ") == "cup"
    assert normalize_target("paper_cup") == "cup"
    assert normalize_target("bread") == "bread"
    assert normalize_target("medicine_bottle") == "bottle"
    assert normalize_target("unknown") is None
    assert TARGETS["bread"] == (
        "长条的玉米面包，半透明平口塑料袋用扎丝封口的长条吐司面包"
    )
    assert TARGETS["bottle"] == "长条药瓶"


def test_prompt_is_chinese_and_requests_bbox_tags() -> None:
    prompt = build_prompt(TARGETS["cup"])
    assert "一次性纸杯" in prompt
    assert "<bbox>x_min y_min x_max y_max</bbox>" in prompt
    assert "允许边界框不完全覆盖物体" in prompt
    assert "边界框的中心点必须位于物体中心点略偏下的位置" in prompt
    assert "NONE" in prompt


def test_parse_multiple_boxes_and_ignore_invalid_boxes() -> None:
    content = (
        "<bbox>10 20 300 400</bbox>\n"
        "<bbox>500, 600, 999, 999</bbox>\n"
        "<bbox>700 700 600 800</bbox>"
    )
    boxes = parse_bboxes(content)
    assert [box.as_dict() for box in boxes] == [
        {"x_min": 10, "y_min": 20, "x_max": 300, "y_max": 400},
        {"x_min": 500, "y_min": 600, "x_max": 999, "y_max": 999},
    ]
    assert boxes[0].to_pixels(1920, 1080) == {
        "x_min": 19,
        "y_min": 22,
        "x_max": 576,
        "y_max": 432,
    }
    assert select_box(boxes, "first") == boxes[0]
    assert select_box(boxes, "largest") == boxes[1]


def test_parse_rejects_zero_area_boxes() -> None:
    assert parse_bboxes("<bbox>10 20 10 200</bbox>") == []
    assert parse_bboxes("<bbox>10 20 200 20</bbox>") == []


def test_artifact_component_blocks_path_traversal_and_is_bounded() -> None:
    component = safe_artifact_component("../../outside\\name:42")

    assert component == "outside_name_42"
    assert "/" not in component and "\\" not in component
    assert len(safe_artifact_component("x" * 200)) == 80
    assert safe_artifact_component("../..") == "request"


def test_png_mime_and_dimensions_without_opencv() -> None:
    png_header = b"\x89PNG\r\n\x1a\n" + b"\x00" * 8 + struct.pack(">II", 640, 480)
    assert detect_mime_type(png_header) == "image/png"
    assert image_dimensions(png_header) == (640, 480)


def test_format_hint_is_used_when_magic_is_unknown() -> None:
    assert detect_mime_type(b"data", "png compressed") == "image/png"
