from .config import ArmSide, X2IKConfig
from .standalone_hand_api import (
    COMMAND_TOPIC,
    CleanHandAPI,
    GRIPPER_HAND_TYPE,
    HandControlAPI,
    HandControlError,
    HandTarget,
    LEFT_JOINT_NAME,
    NONE_HAND_TYPE,
    RIGHT_JOINT_NAME,
    StandaloneHandAPI,
    attach_hand_controls,
    load_ros_bindings,
)


_SOLVER_EXPORTS = {
    "TARGET_VECTOR_LENGTH",
    "IKResult",
    "X2ArmIKSolver",
    "split_target_vector",
}


def __getattr__(name):
    """Load numerical IK dependencies only when the solver API is used."""

    if name in _SOLVER_EXPORTS:
        from . import solver

        value = getattr(solver, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "ArmSide",
    "COMMAND_TOPIC",
    "CleanHandAPI",
    "GRIPPER_HAND_TYPE",
    "HandControlAPI",
    "HandControlError",
    "HandTarget",
    "IKResult",
    "LEFT_JOINT_NAME",
    "NONE_HAND_TYPE",
    "RIGHT_JOINT_NAME",
    "StandaloneHandAPI",
    "TARGET_VECTOR_LENGTH",
    "X2ArmIKSolver",
    "X2IKConfig",
    "attach_hand_controls",
    "load_ros_bindings",
    "split_target_vector",
]
