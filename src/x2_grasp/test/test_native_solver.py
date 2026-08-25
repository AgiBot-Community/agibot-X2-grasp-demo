from concurrent.futures import ThreadPoolExecutor

import pytest

from x2_arm import (
    ArmSide,
    NativeX2ArmIKSolver,
    X2ArmIKSolver,
    X2IKConfig,
    create_ik_solver,
    native_backend_available,
)


pytestmark = pytest.mark.skipif(
    not native_backend_available(), reason="native IK extension is not built"
)


@pytest.fixture(scope="module")
def solvers():
    config = X2IKConfig.default_omnipicker()
    return X2ArmIKSolver(config), NativeX2ArmIKSolver(config)


@pytest.mark.parametrize("side", list(ArmSide))
def test_native_fk_matches_python_for_both_arms(solvers, side):
    python_solver, native_solver = solvers
    seed = python_solver.ready_arm_pos()

    assert native_solver.fk_xyz(side, seed) == pytest.approx(
        python_solver.fk_xyz(side, seed), abs=1e-11
    )
    assert native_solver.fk_axis(side, seed) == pytest.approx(
        python_solver.fk_axis(side, seed), abs=1e-11
    )
    assert native_solver.fk_rpy(side, seed) == pytest.approx(
        python_solver.fk_rpy(side, seed), abs=1e-11
    )


def test_native_configuration_mapping_matches_python(solvers):
    python_solver, native_solver = solvers
    seed = python_solver.ready_arm_pos()

    native_q = native_solver.q_from_arm_pos(seed)

    assert native_q == pytest.approx(python_solver.q_from_arm_pos(seed), abs=1e-12)
    assert native_solver.arm_pos_from_q(native_q) == pytest.approx(seed, abs=1e-12)


def test_native_clipping_preserves_sdk_joint_order(solvers):
    python_solver, native_solver = solvers
    values = python_solver.ready_arm_pos()
    raw_limits = python_solver.joint_limits_for_arm_pos()
    values[1] = raw_limits[1][1] - 1.0
    values[12] = raw_limits[12][2] + 1.0

    native_clipped = native_solver.clip_arm_pos(values)

    assert len(native_clipped) == 14
    assert native_clipped == pytest.approx(
        python_solver.clip_arm_pos(values), abs=1e-12
    )
    for native_limit, python_limit in zip(
        native_solver.joint_limits_for_arm_pos(), raw_limits
    ):
        assert native_limit[0] == python_limit[0]
        assert native_limit[1:] == pytest.approx(python_limit[1:])
    for native_limit, python_limit in zip(
        native_solver.effective_joint_limits_for_arm_pos(),
        python_solver.effective_joint_limits_for_arm_pos(),
    ):
        assert native_limit[0] == python_limit[0]
        assert native_limit[1:] == pytest.approx(python_limit[1:])


@pytest.mark.parametrize("side", list(ArmSide))
def test_native_axis_ik_matches_python(solvers, side):
    python_solver, native_solver = solvers
    seed = python_solver.ready_arm_pos()
    current_xyz = python_solver.fk_xyz(side, seed)
    current_axis = python_solver.fk_axis(side, seed)
    target_xyz = [
        current_xyz[0] + 0.005,
        current_xyz[1],
        current_xyz[2] + 0.005,
    ]

    expected = python_solver.solve_axis(side, target_xyz, current_axis, seed)
    actual = native_solver.solve_axis(side, target_xyz, current_axis, seed)

    assert actual.success == expected.success
    assert actual.iterations == expected.iterations
    assert actual.arm_pos == pytest.approx(expected.arm_pos, abs=2e-8)
    assert actual.final_xyz == pytest.approx(expected.final_xyz, abs=2e-9)
    assert actual.final_axis == pytest.approx(expected.final_axis, abs=2e-9)
    assert actual.position_error_norm == pytest.approx(
        expected.position_error_norm, abs=2e-9
    )
    assert actual.orientation_error_norm == pytest.approx(
        expected.orientation_error_norm, abs=2e-9
    )


@pytest.mark.parametrize("side", list(ArmSide))
def test_native_position_ik_matches_python(solvers, side):
    python_solver, native_solver = solvers
    seed = python_solver.ready_arm_pos()
    current_xyz = python_solver.fk_xyz(side, seed)
    target_xyz = [current_xyz[0] + 0.005, current_xyz[1], current_xyz[2] + 0.005]

    expected = python_solver.solve_position(side, target_xyz, seed)
    actual = native_solver.solve_position(side, target_xyz, seed)

    assert actual.success == expected.success
    assert actual.iterations == expected.iterations
    assert actual.arm_pos == pytest.approx(expected.arm_pos, abs=2e-8)
    assert actual.final_xyz == pytest.approx(expected.final_xyz, abs=2e-9)
    assert actual.error_norm == pytest.approx(expected.error_norm, abs=2e-9)


@pytest.mark.parametrize("side", list(ArmSide))
def test_native_pose_and_6d_ik_match_python(solvers, side):
    python_solver, native_solver = solvers
    seed = python_solver.ready_arm_pos()
    current_xyz = python_solver.fk_xyz(side, seed)
    current_rpy = python_solver.fk_rpy(side, seed)
    target_xyz = [current_xyz[0] + 0.005, current_xyz[1], current_xyz[2] + 0.005]

    expected = python_solver.solve_pose(side, target_xyz, current_rpy, seed)
    actual = native_solver.solve_pose(side, target_xyz, current_rpy, seed)
    actual_6d = native_solver.solve_6d(side, target_xyz + current_rpy, seed)

    for result in (actual, actual_6d):
        assert result.success == expected.success
        assert result.iterations == expected.iterations
        assert result.arm_pos == pytest.approx(expected.arm_pos, abs=2e-8)
        assert result.final_xyz == pytest.approx(expected.final_xyz, abs=2e-9)
        assert result.final_rpy == pytest.approx(expected.final_rpy, abs=2e-9)
        assert result.position_error_norm == pytest.approx(
            expected.position_error_norm, abs=2e-9
        )
        assert result.orientation_error_norm == pytest.approx(
            expected.orientation_error_norm, abs=2e-9
        )


def test_native_special_seed_inputs_use_compatible_python_path(solvers):
    python_solver, native_solver = solvers
    seed = python_solver.ready_arm_pos()
    q_seed = python_solver.q_from_arm_pos(seed)
    target = python_solver.fk_xyz(ArmSide.RIGHT, seed)

    assert native_solver.fk_xyz(
        ArmSide.RIGHT, q_seed=q_seed
    ) == pytest.approx(target)
    assert native_solver.solve_position(
        ArmSide.RIGHT, target, q_seed=q_seed
    ).arm_pos == pytest.approx(seed)
    assert native_solver.q_from_arm_pos(seed, [0.1, -0.1]) == pytest.approx(
        python_solver.q_from_arm_pos(seed, [0.1, -0.1])
    )


def test_native_backend_builds_python_model_only_for_special_inputs():
    solver = NativeX2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()

    solver.fk_xyz(ArmSide.RIGHT, seed)
    solver.solve_position(ArmSide.RIGHT, solver.fk_xyz(ArmSide.RIGHT, seed), seed)
    assert solver._python_fallback is None

    solver.q_from_arm_pos(seed, [0.0, 0.0])
    assert solver._python_fallback is not None


def test_native_failure_result_matches_python():
    config = X2IKConfig(
        urdf_path=X2IKConfig.default_omnipicker().urdf_path,
        max_iters=1,
    )
    python_solver = X2ArmIKSolver(config)
    native_solver = NativeX2ArmIKSolver(config)
    seed = python_solver.ready_arm_pos()
    target = [2.0, -1.0, 2.0]

    expected = python_solver.solve_position(ArmSide.RIGHT, target, seed)
    actual = native_solver.solve_position(ArmSide.RIGHT, target, seed)

    assert not actual.success
    assert actual.iterations == expected.iterations == 1
    assert actual.arm_pos == pytest.approx(expected.arm_pos, abs=2e-8)
    assert actual.message == expected.message


def test_native_solver_serializes_shared_pinocchio_data(solvers):
    python_solver, native_solver = solvers
    seed = python_solver.ready_arm_pos()
    targets = {side: python_solver.fk_xyz(side, seed) for side in ArmSide}

    with ThreadPoolExecutor(max_workers=4) as executor:
        futures = [
            executor.submit(native_solver.solve_position, side, targets[side], seed)
            for _ in range(20)
            for side in ArmSide
        ]

    for index, future in enumerate(futures):
        side = list(ArmSide)[index % 2]
        result = future.result()
        assert result.success
        assert result.final_xyz == pytest.approx(targets[side], abs=1e-11)


def test_native_solver_keeps_python_validation_contract(solvers):
    _, native_solver = solvers
    seed = native_solver.ready_arm_pos()

    with pytest.raises(ValueError, match="three finite"):
        native_solver.solve_position(ArmSide.RIGHT, [0.0, float("nan"), 0.0], seed)
    with pytest.raises(ValueError, match="positive finite"):
        native_solver.solve_pose(
            ArmSide.RIGHT,
            native_solver.fk_xyz(ArmSide.RIGHT, seed),
            native_solver.fk_rpy(ArmSide.RIGHT, seed),
            seed,
            orientation_weight=0.0,
        )
    with pytest.raises(ValueError, match="non-zero"):
        native_solver.fk_axis(ArmSide.RIGHT, seed, local_axis=[0.0, 0.0, 0.0])


def test_backend_factory_is_explicit_and_auto_prefers_native():
    config = X2IKConfig.default_omnipicker()

    assert create_ik_solver(config, "python").backend == "python"
    assert create_ik_solver(config, "native").backend == "native"
    assert create_ik_solver(config, "auto").backend == "native"
    with pytest.raises(ValueError, match="backend"):
        create_ik_solver(config, "cuda")


def test_ik_command_entrypoints_default_to_auto_backend():
    from x2_arm.cli import build_parser as build_cli_parser
    from x2_arm.hardware_node import build_parser as build_hardware_parser

    assert build_cli_parser().parse_args([]).ik_backend == "auto"
    assert build_hardware_parser().parse_args([]).ik_backend == "auto"


def test_auto_backend_falls_back_when_extension_is_unavailable(monkeypatch):
    import x2_arm.native_solver as native_module

    config = X2IKConfig.default_omnipicker()
    monkeypatch.setattr(native_module, "_NativeIKSolver", None)

    assert native_module.create_ik_solver(config, "auto").backend == "python"
    with pytest.raises(native_module.NativeBackendUnavailable):
        native_module.create_ik_solver(config, "native")
