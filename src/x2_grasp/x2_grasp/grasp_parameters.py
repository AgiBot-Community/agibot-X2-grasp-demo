"""Central parameter declaration, CLI parsing, and validation for grasping."""

from __future__ import annotations

import argparse
import math
from types import SimpleNamespace

from .grasp_constants import (
    APPROACH_STANDOFF_M,
    APRILTAG_VECTOR_TOPIC,
    BACKWARD_M,
    GROUNDING_VECTOR_TOPIC,
    GRASP_PLANE_Z_M,
    GRASP_X_OFFSET_M,
    GRIPPER_REACH_M,
    INITIAL_UPWARD_M,
    TAG_TO_OBJECT_DEPTH_M,
    UPWARD_AFTER_GRASP_M,
)
from .target_catalog import (
    DEFAULT_TARGET_ALIASES,
    DEFAULT_TARGET_DESCRIPTIONS,
    DEFAULT_TARGET_GRIP_POSITIONS,
    DEFAULT_TARGET_NAMES,
    DEFAULT_TARGET_PCM_PATHS,
    build_target_catalog,
)


DEFAULTS = {
    "action_name": "/x2_grasp/grasp",
    "execute": False,
    "duration": 4.0,
    "approach_duration": 3.0,
    "timeout": 300.0,
    "standoff": APPROACH_STANDOFF_M,
    "gripper_reach": GRIPPER_REACH_M,
    "tag_depth": TAG_TO_OBJECT_DEPTH_M,
    "source": "grounding",
    "backward": BACKWARD_M,
    "initial_upward": INITIAL_UPWARD_M,
    "grasp_x_offset": GRASP_X_OFFSET_M,
    "grasp_plane_z": GRASP_PLANE_Z_M,
    "post_grasp_lift": UPWARD_AFTER_GRASP_M,
    "min_post_grasp_lift": 0.04,
    "lift_step": 0.02,
    "cartesian_step": 0.015,
    "high_retract_position_tolerance": 0.005,
    "high_retract_axis_tolerance": 0.05,
    "grasp_axis_orientation_weight": 0.5,
    "grasp_axis_orientation_eps": 0.087,
    "ik_seed_perturbation": 0.12,
    "target_names": DEFAULT_TARGET_NAMES,
    "target_descriptions": DEFAULT_TARGET_DESCRIPTIONS,
    "target_aliases": DEFAULT_TARGET_ALIASES,
    "target_grip_close_positions": DEFAULT_TARGET_GRIP_POSITIONS,
    "target_pcm_paths": DEFAULT_TARGET_PCM_PATHS,
    "default_pcm_path": "grasp_complete.pcm",
    "initial_close_seconds": 2.0,
    "open_seconds": 0.5,
    "grip_close_seconds": 2.0,
    "audio_playback_topic": "/aima/hal/audio/playback",
    "audio_focus_response_topic": "/aima/hal/audio/focus_response",
    "audio_focus_request_service": "/aimdk_5Fmsgs/srv/RequestAudioFocus",
    "audio_focus_release_service": "/aimdk_5Fmsgs/srv/AbandonAudioFocus",
    "audio_pkg_name": "x2_grasp",
    "audio_focus_priority": 6,
    "audio_chunk_ms": 50,
    "vector_topic": GROUNDING_VECTOR_TOPIC,
    "apriltag_topic": APRILTAG_VECTOR_TOPIC,
    "skip_mode_switch": False,
    "grounding_command_topic": "/x2_grasp/grounding_target",
    "arbitration_probe_timeout": 0.5,
    "arbitration_tag_freshness": 1.0,
    "grounding_result_topic": "/x2_grasp/grounding_result",
    "localizer_status_topic": "/x2_rgbd_localizer/status",
    "grounding_retry_attempts": 3,
    "grounding_retry_delay": 1.0,
    "grounding_attempt_timeout": 35.0,
    "localization_result_timeout": 5.0,
}

BOOLEAN_PARAMETERS = {"execute", "skip_mode_switch"}
INTEGER_PARAMETERS = {
    "audio_focus_priority",
    "audio_chunk_ms",
    "grounding_retry_attempts",
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="X2 grasp Action server")
    for name, default in DEFAULTS.items():
        option = "--" + name.replace("_", "-")
        if name in BOOLEAN_PARAMETERS:
            parser.add_argument(option, action="store_true", default=default)
        elif name == "source":
            parser.add_argument(
                option,
                choices=("grounding", "apriltag", "auto"),
                default=default,
            )
        elif isinstance(default, list):
            value_type = float if default and isinstance(default[0], float) else str
            parser.add_argument(option, nargs="+", type=value_type, default=list(default))
        else:
            value_type = int if name in INTEGER_PARAMETERS else type(default)
            parser.add_argument(option, type=value_type, default=default)
    return parser


def parse_ros_params_into_args(node):
    for name, value in DEFAULTS.items():
        node.declare_parameter(name, value)
    values = {}
    for name, default in DEFAULTS.items():
        value = node.get_parameter(name).value
        if isinstance(default, list):
            values[name] = list(value)
        elif name in BOOLEAN_PARAMETERS:
            values[name] = bool(value)
        elif name in INTEGER_PARAMETERS:
            values[name] = int(value)
        elif isinstance(default, float):
            values[name] = float(value)
        else:
            values[name] = str(value)
    return SimpleNamespace(**values)


def validate_parameters(args) -> None:
    if args.source not in {"grounding", "apriltag", "auto"}:
        raise ValueError("source must be grounding, apriltag, or auto")
    numeric_names = [
        name for name, default in DEFAULTS.items() if isinstance(default, float)
    ]
    for name in numeric_names:
        if not math.isfinite(getattr(args, name)):
            raise ValueError(f"{name} must be finite")

    positive = (
        "standoff",
        "duration",
        "approach_duration",
        "timeout",
        "min_post_grasp_lift",
        "lift_step",
        "cartesian_step",
        "high_retract_position_tolerance",
        "high_retract_axis_tolerance",
        "grasp_axis_orientation_weight",
        "grasp_axis_orientation_eps",
        "ik_seed_perturbation",
        "initial_close_seconds",
        "open_seconds",
        "grip_close_seconds",
        "arbitration_tag_freshness",
        "grounding_attempt_timeout",
        "localization_result_timeout",
    )
    for name in positive:
        if getattr(args, name) <= 0.0:
            raise ValueError(f"{name} must be positive")
    non_negative = (
        "gripper_reach",
        "tag_depth",
        "backward",
        "initial_upward",
        "post_grasp_lift",
        "grounding_retry_delay",
        "arbitration_probe_timeout",
    )
    for name in non_negative:
        if getattr(args, name) < 0.0:
            raise ValueError(f"{name} cannot be negative")
    if args.min_post_grasp_lift > args.post_grasp_lift:
        raise ValueError("min_post_grasp_lift cannot exceed post_grasp_lift")
    args.target_catalog = build_target_catalog(
        args.target_names,
        args.target_descriptions,
        args.target_aliases,
        args.target_grip_close_positions,
        args.target_pcm_paths,
    )
    if not 1 <= args.audio_focus_priority <= 10:
        raise ValueError("audio_focus_priority must be in [1, 10]")
    if args.audio_chunk_ms < 1 or args.grounding_retry_attempts < 1:
        raise ValueError("audio_chunk_ms and grounding_retry_attempts must be positive")
    for name in (
        "action_name",
        "grounding_command_topic",
        "grounding_result_topic",
        "localizer_status_topic",
        "default_pcm_path",
    ):
        if not getattr(args, name).strip():
            raise ValueError(f"{name} must not be empty")
