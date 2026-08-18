import math
from pathlib import Path
from xml.etree import ElementTree

import pytest

from x2_arm.config import ARM_POS_ORDER, X2IKConfig
from x2_arm.solver_node import (
    DEFAULT_STATUS_TOPIC,
    DEFAULT_TOPICS,
    ordered_arm_pos,
    quaternion_to_rpy,
)
from x2_arm import ArmSide
from x2_arm.hardware_node import joint_states_to_arm_pos


def test_public_topic_defaults_are_stable():
    assert DEFAULT_TOPICS[ArmSide.LEFT]["target"] == "/x2_ik/left/target_pose"
    assert DEFAULT_TOPICS[ArmSide.RIGHT]["target"] == "/x2_ik/right/target_pose"
    assert DEFAULT_TOPICS[ArmSide.LEFT]["solution"] == "/x2_ik/left/solution"
    assert DEFAULT_TOPICS[ArmSide.RIGHT]["solution"] == "/x2_ik/right/solution"
    assert DEFAULT_TOPICS[ArmSide.LEFT]["target_6d"] == "/x2_arm/left/target_6d"
    assert DEFAULT_TOPICS[ArmSide.RIGHT]["target_6d"] == "/x2_arm/right/target_6d"
    assert DEFAULT_STATUS_TOPIC == "/x2_ik/status"


def test_default_joint_margin_is_a_real_operational_margin():
    assert X2IKConfig.default_omnipicker().joint_margin == pytest.approx(0.02)


def test_ordered_arm_pos_reorders_complete_joint_state():
    names = list(reversed(ARM_POS_ORDER))
    value_by_name = {name: float(index) for index, name in enumerate(ARM_POS_ORDER)}
    positions = [value_by_name[name] for name in names]
    assert ordered_arm_pos(names, positions) == [float(i) for i in range(14)]


def test_ordered_arm_pos_rejects_incomplete_joint_state():
    assert ordered_arm_pos(ARM_POS_ORDER[:-1], range(13)) is None
    assert ordered_arm_pos(ARM_POS_ORDER, [0.0] * 13 + [float("nan")]) is None


def test_quaternion_to_rpy_handles_identity_and_yaw():
    assert quaternion_to_rpy(0.0, 0.0, 0.0, 1.0) == pytest.approx([0.0, 0.0, 0.0])
    yaw = math.pi / 2.0
    assert quaternion_to_rpy(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(
        [0.0, 0.0, yaw]
    )


def test_quaternion_to_rpy_rejects_zero_norm():
    with pytest.raises(ValueError, match="zero norm"):
        quaternion_to_rpy(0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="finite"):
        quaternion_to_rpy(0.0, 0.0, float("nan"), 1.0)


def test_package_manifest_declares_ament_cmake():
    root = Path(__file__).resolve().parents[1]
    manifest = ElementTree.parse(root / "package.xml").getroot()
    assert manifest.findtext("name") == "x2_grasp"
    export = manifest.find("export")
    assert export is not None
    assert export.findtext("build_type") == "ament_cmake"
    assert (root / "CMakeLists.txt").is_file()


def test_hardware_joint_state_mapping_is_strict():
    class State:
        def __init__(self, name, position):
            self.name = name
            self.position = position

    states = [State(name, index) for index, name in enumerate(reversed(ARM_POS_ORDER))]
    by_name = {state.name: float(state.position) for state in states}
    assert joint_states_to_arm_pos(states) == [by_name[name] for name in ARM_POS_ORDER]

    with pytest.raises(ValueError, match="missing arm joints"):
        joint_states_to_arm_pos(states[:-1])

    states[-1].position = float("inf")
    with pytest.raises(ValueError, match="non-finite"):
        joint_states_to_arm_pos(states)
