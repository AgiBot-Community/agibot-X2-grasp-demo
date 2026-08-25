from pathlib import Path
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _fields(name: str) -> set[str]:
    lines = (PROJECT_ROOT / f"msg/{name}.msg").read_text(encoding="utf-8").splitlines()
    return {
        line.split()[-1]
        for line in lines
        if line.strip() and not line.lstrip().startswith("#") and "=" not in line
    }


def test_package_is_a_rosidl_interface_package() -> None:
    package = ET.parse(PROJECT_ROOT / "package.xml").getroot()

    assert package.findtext("name") == "x2_grasp"
    assert package.findtext("member_of_group") == "rosidl_interface_packages"
    assert package.findtext("buildtool_depend") == "ament_cmake"
    assert package.findtext("build_depend") == "rosidl_default_generators"
    assert "action_msgs" in [item.text for item in package.findall("depend")]


def test_workflow_contract_carries_grasp_correlation_fields() -> None:
    assert {
        "stamp",
        "workflow_id",
        "state",
        "need",
        "grasp_request_id",
        "error",
    } <= _fields("WorkflowStatus")


def test_grasp_action_defines_goal_result_and_feedback() -> None:
    sections = (PROJECT_ROOT / "action/Grasp.action").read_text(
        encoding="utf-8"
    ).split("---")

    assert len(sections) == 3
    assert "string target" in sections[0]
    assert "bool success" in sections[1]
    assert "string error" in sections[1]
    assert "string stage" in sections[2]
    assert "string detail" in sections[2]
    assert not (PROJECT_ROOT / "msg/GraspCommand.msg").exists()
    assert not (PROJECT_ROOT / "msg/GraspStatus.msg").exists()


def test_internal_command_action_carries_stream_inputs_and_timing_metrics() -> None:
    sections = (PROJECT_ROOT / "action/ExecuteCommand.action").read_text(
        encoding="utf-8"
    ).split("---")

    assert len(sections) == 3
    assert "float64[] start_arm_pos" in sections[0]
    assert "float64[] goal_arm_pos" in sections[0]
    assert "float64 duration" in sections[0]
    assert "string hand" in sections[0]
    assert "uint32 deadline_misses" in sections[1]
    assert "int64 max_lateness_ns" in sections[1]
    assert "bool hold_published" in sections[1]


def test_perception_contracts_use_typed_stamps_and_results() -> None:
    assert {"created_at", "request_id", "target"} <= _fields("GroundingCommand")
    assert {
        "image_stamp",
        "request_id",
        "success",
        "box_count",
        "latency_ms",
        "error",
    } <= _fields("GroundingResult")
    assert {"image_stamp", "success", "stage", "error"} <= _fields(
        "PerceptionStatus"
    )
    assert {
        "source",
        "detail",
        "detected_ids",
        "sample_count",
        "required_samples",
        "spread_m",
        "reprojection_error_px",
        "target",
        "frame_id",
    } <= _fields("PerceptionStatus")
