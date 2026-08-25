from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))


def test_package_declares_runtime_dependencies_and_public_api() -> None:
    package = ET.parse(PROJECT_ROOT / "package.xml").getroot()
    dependencies = [item.text for item in package.findall("exec_depend")]

    assert package.findtext("name") == "x2_grasp"
    assert package.findtext("./export/build_type") == "ament_cmake"
    assert "aimdk_msgs" in dependencies
    assert "rclpy" in dependencies

    import x2_common

    assert x2_common.PCM_SAMPLE_RATE == 16_000
    assert x2_common.BlockingPcmPlayer is not None
    assert x2_common.StreamingPcmPlayer is not None


def test_readme_is_installed_with_package() -> None:
    cmake_source = (PROJECT_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert (PROJECT_ROOT / "README.md").is_file()
    assert "install(FILES README.md" in cmake_source
    assert "X2_WORKSPACE_BUNDLE" not in cmake_source
    assert ".python_packages" not in cmake_source
    assert '"action/Grasp.action"' in cmake_source


def test_python_dependencies_come_from_the_system_environment() -> None:
    package = ET.parse(PROJECT_ROOT / "package.xml").getroot()
    dependencies = [item.text for item in package.findall("exec_depend")]
    runner = (PROJECT_ROOT / "scripts/_run_python").read_text(encoding="utf-8")

    assert not (PROJECT_ROOT.parents[1] / "requirements.txt").exists()
    assert {"python3-numpy", "python3-opencv"} <= set(dependencies)
    assert ".python_packages" not in runner


def test_workspace_contains_exactly_one_ros_package() -> None:
    workspace_src = PROJECT_ROOT.parent
    manifests = sorted(workspace_src.glob("*/package.xml"))

    assert manifests == [PROJECT_ROOT / "package.xml"]


def test_grasp_orchestrator_is_split_by_responsibility() -> None:
    module_root = PROJECT_ROOT / "x2_grasp"
    for name in (
        "grasp_action_server.py",
        "grasp_executor.py",
        "grasp_parameters.py",
        "grasp_planner.py",
        "perception_coordinator.py",
    ):
        assert (module_root / name).is_file()

    orchestrator_lines = (module_root / "grasp_node.py").read_text(
        encoding="utf-8"
    ).splitlines()
    assert len(orchestrator_lines) < 350


def test_action_client_example_is_installed() -> None:
    cmake_source = (PROJECT_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert (PROJECT_ROOT / "x2_grasp/grasp_action_client.py").is_file()
    assert (PROJECT_ROOT / "scripts/grasp_action_client").is_file()
    assert "grasp_action_client" in cmake_source


def test_workspace_has_public_demo_and_topic_documentation() -> None:
    workspace_root = PROJECT_ROOT.parents[1]
    readme = (workspace_root / "README.md").read_text(encoding="utf-8")
    demo = (workspace_root / "example/send_grasp_goal.py").read_text(
        encoding="utf-8"
    )

    for document in (
        "INSTALLATION.md",
        "USAGE.md",
        "INTERFACES.md",
        "CONFIGURATION.md",
        "ARCHITECTURE.md",
        "PERFORMANCE.md",
        "PUBLISHING.md",
        "REMOTE_API_AND_URDF.md",
        "TROUBLESHOOTING.md",
    ):
        assert (workspace_root / "docs" / document).is_file()
        assert f"docs/{document}" in readme
    assert "example/README.md" in readme
    assert "ActionClient" in demo
    assert "cancel_goal_async" in demo


def test_documentation_relative_links_resolve() -> None:
    workspace_root = PROJECT_ROOT.parents[1]
    markdown_files = [workspace_root / "README.md"]
    markdown_files.extend((workspace_root / "docs").glob("*.md"))
    link_pattern = re.compile(r"\[[^]]*\]\(([^)]+)\)")

    for document in markdown_files:
        source = document.read_text(encoding="utf-8")
        for target in link_pattern.findall(source):
            path_text = target.split("#", 1)[0]
            if not path_text or "://" in path_text or path_text.startswith("mailto:"):
                continue
            target_path = (document.parent / path_text).resolve()
            assert target_path.exists(), f"broken link in {document}: {target}"


def test_unified_launch_exposes_native_ik_backend_selection() -> None:
    launch_source = (PROJECT_ROOT / "launch/unified_grasp.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument(\n                "ik_backend"' in launch_source
    assert 'choices=["auto", "native", "python"]' in launch_source
    assert '"ik_backend": ik_backend' in launch_source


def test_workspace_has_open_source_project_metadata() -> None:
    workspace_root = PROJECT_ROOT.parents[1]
    package = ET.parse(PROJECT_ROOT / "package.xml").getroot()
    license_text = (workspace_root / "LICENSE").read_text(encoding="utf-8")

    assert package.findtext("license") == "Apache-2.0"
    assert "Apache License" in license_text
    assert "Version 2.0, January 2004" in license_text
