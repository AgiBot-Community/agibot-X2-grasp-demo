from pathlib import Path
import xml.etree.ElementTree as ET


def test_cmake_installs_only_public_grasp_config():
    root = Path(__file__).resolve().parents[1]
    cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "install(DIRECTORY audio config" not in cmake
    assert "install(FILES config/unified_grasp.yaml" in cmake
    assert "api_key.yaml" not in cmake


def test_pinocchio_is_managed_as_a_ros_dependency():
    root = Path(__file__).resolve().parents[1]
    package = ET.parse(root / "package.xml").getroot()
    dependencies = [item.text for item in package.findall("exec_depend")]
    install_script = (root.parents[1] / "scripts/install_dependencies.sh").read_text(
        encoding="utf-8"
    )

    assert "pinocchio" in dependencies
    assert '"ros-${ros_distro}-pinocchio"' in install_script
    assert '"${apt[@]}" install -y "${packages[@]}"' in install_script
    for system_dependency in ("libopencv-dev", "python3-numpy", "python3-opencv"):
        assert system_dependency in install_script


def test_apriltag_uses_typed_perception_status():
    root = Path(__file__).resolve().parents[1]
    source = (root / "x2_grasp/apriltag_detector.py").read_text(encoding="utf-8")

    assert "from x2_grasp.msg import PerceptionStatus" in source
    assert "create_publisher(\n            PerceptionStatus" in source
    assert "std_msgs.msg import String" not in source
