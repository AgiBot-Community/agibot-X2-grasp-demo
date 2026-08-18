from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from x2_common import package_file


class ArmSide(str, Enum):
    LEFT = "left"
    RIGHT = "right"


LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
]

RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
]

ARM_POS_ORDER = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

DEFAULT_URDF_RELATIVE_PATH = (
    Path("urdf") / "x2_ultra_plus_omnipicker_omnipicker.urdf"
)


def default_urdf_path() -> Path:
    return package_file(
        "x2_grasp",
        DEFAULT_URDF_RELATIVE_PATH,
        source_root=Path(__file__).resolve().parents[1],
    )


@dataclass(frozen=True)
class X2IKConfig:
    urdf_path: Path
    left_ee_frame: str = "L_omnipicker_base_link"
    right_ee_frame: str = "R_omnipicker_base_link"
    left_ready_arm: list[float] = field(
        default_factory=lambda: [-0.35, 0.45, 0.0, -1.0, 0.0, 0.15, 0.0]
    )
    right_ready_arm: list[float] = field(
        default_factory=lambda: [-0.35, -0.45, 0.0, -1.0, 0.0, 0.15, 0.0]
    )
    eps: float = 1e-4
    max_iters: int = 1000
    dt: float = 0.1
    damping: float = 1e-4
    max_step_norm: float = 0.05
    # Keep arm IK away from the URDF mechanical hard stops. This is an
    # operational margin, not a replacement for the robot controller limits.
    joint_margin: float = 0.02

    def __post_init__(self) -> None:
        positive_values = {
            "eps": self.eps,
            "dt": self.dt,
            "damping": self.damping,
            "max_step_norm": self.max_step_norm,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be a positive finite value")
        if not isinstance(self.max_iters, int) or self.max_iters < 1:
            raise ValueError("max_iters must be a positive integer")
        if not math.isfinite(self.joint_margin) or self.joint_margin < 0.0:
            raise ValueError("joint_margin must be a finite non-negative value")
        for name, values in (
            ("left_ready_arm", self.left_ready_arm),
            ("right_ready_arm", self.right_ready_arm),
        ):
            if len(values) != 7 or not all(math.isfinite(value) for value in values):
                raise ValueError(f"{name} must contain seven finite values")
        if not self.left_ee_frame.strip() or not self.right_ee_frame.strip():
            raise ValueError("end-effector frame names must not be empty")

    @staticmethod
    def default_omnipicker() -> "X2IKConfig":
        return X2IKConfig(urdf_path=default_urdf_path())

    def frame_for_side(self, side: ArmSide) -> str:
        return self.left_ee_frame if side == ArmSide.LEFT else self.right_ee_frame

    def active_joints_for_side(self, side: ArmSide) -> list[str]:
        return LEFT_ARM_JOINTS if side == ArmSide.LEFT else RIGHT_ARM_JOINTS

    def ready_arm_pos(self) -> list[float]:
        return self.left_ready_arm + self.right_ready_arm
