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


def test_backend_factory_is_explicit_and_auto_prefers_native():
    config = X2IKConfig.default_omnipicker()

    assert create_ik_solver(config, "python").backend == "python"
    assert create_ik_solver(config, "native").backend == "native"
    assert create_ik_solver(config, "auto").backend == "native"
    with pytest.raises(ValueError, match="backend"):
        create_ik_solver(config, "cuda")


def test_auto_backend_falls_back_when_extension_is_unavailable(monkeypatch):
    import x2_arm.native_solver as native_module

    config = X2IKConfig.default_omnipicker()
    monkeypatch.setattr(native_module, "_NativeIKSolver", None)

    assert native_module.create_ik_solver(config, "auto").backend == "python"
    with pytest.raises(native_module.NativeBackendUnavailable):
        native_module.create_ik_solver(config, "native")
