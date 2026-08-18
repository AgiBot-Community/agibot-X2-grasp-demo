from types import SimpleNamespace

import pytest

from x2_grasp.grasp_parameters import DEFAULTS, validate_parameters


def _args(**overrides):
    values = dict(DEFAULTS)
    values.update(overrides)
    return SimpleNamespace(**values)


def test_action_is_the_only_public_grasp_command_parameter() -> None:
    assert DEFAULTS["action_name"] == "/x2_grasp/grasp"
    assert "command_topic" not in DEFAULTS
    assert "status_topic" not in DEFAULTS


def test_parameter_validation_rejects_unsafe_motion_values() -> None:
    validate_parameters(_args())

    with pytest.raises(ValueError, match="cartesian_step"):
        validate_parameters(_args(cartesian_step=0.0))
    with pytest.raises(ValueError, match="grip close position"):
        validate_parameters(_args(target_grip_close_positions=[1.1, 0.1, 0.1]))
    with pytest.raises(ValueError, match="action_name"):
        validate_parameters(_args(action_name=""))


def test_target_catalog_is_built_from_parallel_config_arrays() -> None:
    args = _args(
        target_names=["apple"],
        target_descriptions=["红色苹果"],
        target_aliases=["fruit=apple"],
        target_grip_close_positions=[0.25],
        target_pcm_paths=[""],
    )

    validate_parameters(args)

    assert args.target_catalog.normalize("fruit") == "apple"
    assert args.target_catalog.spec("apple").description == "红色苹果"
    assert args.target_catalog.spec("apple").grip_close_position == 0.25
    assert args.target_catalog.spec("apple").pcm_path == ""
    assert args.default_pcm_path == "grasp_complete.pcm"


def test_target_catalog_rejects_misaligned_arrays() -> None:
    with pytest.raises(ValueError, match="same length"):
        validate_parameters(_args(target_descriptions=["only one"]))
