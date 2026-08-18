import pytest
import pinocchio as pin

from x2_arm import ArmSide, X2ArmIKSolver, X2IKConfig, split_target_vector


def test_right_arm_offset_ik():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    current = solver.fk_xyz(ArmSide.RIGHT, seed)
    target = [current[0] + 0.01, current[1], current[2] + 0.01]
    result = solver.solve_position(ArmSide.RIGHT, target, seed)
    assert result.success
    assert result.error_norm < 2e-4
    assert len(result.arm_pos) == 14


def test_left_arm_offset_ik():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    current = solver.fk_xyz(ArmSide.LEFT, seed)
    target = [current[0] + 0.01, current[1], current[2] + 0.01]
    result = solver.solve_position(ArmSide.LEFT, target, seed)
    assert result.success
    assert result.error_norm < 2e-4
    assert len(result.arm_pos) == 14


def test_right_arm_pose_ik_keeps_current_orientation():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    current_xyz = solver.fk_xyz(ArmSide.RIGHT, seed)
    current_rpy = solver.fk_rpy(ArmSide.RIGHT, seed)
    target_xyz = [current_xyz[0] + 0.005, current_xyz[1], current_xyz[2] + 0.005]
    result = solver.solve_pose(ArmSide.RIGHT, target_xyz, current_rpy, seed)
    assert result.success
    assert result.position_error_norm is not None
    assert result.orientation_error_norm is not None
    assert result.position_error_norm < 2e-4
    assert result.orientation_error_norm < 1e-3
    assert len(result.arm_pos) == 14


def test_right_arm_axis_ik_allows_roll_but_keeps_tool_axis():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    current_xyz = solver.fk_xyz(ArmSide.RIGHT, seed)
    current_rpy = solver.fk_rpy(ArmSide.RIGHT, seed)
    current_rotation = pin.rpy.rpyToMatrix(*current_rpy)
    target_axis = current_rotation[:, 2].tolist()
    target_xyz = [current_xyz[0] + 0.005, current_xyz[1], current_xyz[2] + 0.005]

    result = solver.solve_axis(
        ArmSide.RIGHT, target_xyz, target_axis, seed, orientation_eps=0.02
    )

    assert result.success
    assert result.position_error_norm < 2e-4
    assert result.orientation_error_norm < 0.02
    assert result.target_axis == pytest.approx(target_axis)
    assert solver.fk_axis(ArmSide.RIGHT, result.arm_pos) == pytest.approx(
        result.final_axis
    )


def test_six_dimension_target_vector_solves_pose():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    current_xyz = solver.fk_xyz(ArmSide.RIGHT, seed)
    current_rpy = solver.fk_rpy(ArmSide.RIGHT, seed)
    target = [current_xyz[0] + 0.005, current_xyz[1], current_xyz[2] + 0.005]

    result = solver.solve_6d(ArmSide.RIGHT, target + current_rpy, seed)

    assert result.success
    assert result.target_xyz == pytest.approx(target)
    assert result.target_rpy == pytest.approx(current_rpy)
    assert result.position_error_norm is not None
    assert result.orientation_error_norm is not None
    assert result.position_error_norm < 2e-4
    assert result.orientation_error_norm < 1e-3


def test_six_dimension_target_vector_is_strictly_validated():
    assert split_target_vector(range(6)) == ([0.0, 1.0, 2.0], [3.0, 4.0, 5.0])

    with pytest.raises(ValueError, match="length 6"):
        split_target_vector([0.0] * 5)
    with pytest.raises(ValueError, match="finite"):
        split_target_vector([0.0, 0.0, 0.0, 0.0, 0.0, float("nan")])


def test_arm_position_clipping_keeps_trajectory_inside_operational_limits():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    raw_limits = solver.joint_limits_for_arm_pos()
    near_limit = solver.ready_arm_pos()
    near_limit[8] = raw_limits[8][2] - 1e-6

    clipped = solver.clip_arm_pos(near_limit)
    effective_limits = solver.effective_joint_limits_for_arm_pos()

    assert clipped[8] == pytest.approx(effective_limits[8][2])
    assert clipped[8] < near_limit[8]


def test_solver_rejects_non_finite_motion_inputs():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()

    with pytest.raises(ValueError, match="finite"):
        solver.solve_position(ArmSide.RIGHT, [0.2, float("nan"), 0.3], seed)
    with pytest.raises(ValueError, match="finite"):
        solver.solve_pose(
            ArmSide.RIGHT, [0.2, -0.2, 0.3], [0.0, float("inf"), 0.0], seed
        )
    with pytest.raises(ValueError, match="finite"):
        solver.clip_arm_pos([0.0] * 13 + [float("nan")])
