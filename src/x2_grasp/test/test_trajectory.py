import pytest

from x2_arm.trajectory import interpolate_arm_pos


def test_trajectory_rejects_unsafe_numeric_inputs() -> None:
    zeros = [0.0] * 14

    for kwargs in (
        {"duration": 0.0},
        {"rate_hz": 0.0},
        {"max_delta_per_step": 0.0},
    ):
        with pytest.raises(ValueError, match="positive finite"):
            interpolate_arm_pos(zeros, zeros, **kwargs)

    with pytest.raises(ValueError, match="finite"):
        interpolate_arm_pos(zeros, [0.0] * 13 + [float("nan")])


def test_trajectory_keeps_endpoints_and_step_limit() -> None:
    start = [0.0] * 14
    goal = [0.0] * 13 + [0.2]

    waypoints = interpolate_arm_pos(
        start, goal, duration=1.0, rate_hz=10.0, max_delta_per_step=0.03
    )

    assert waypoints[0].arm_pos == pytest.approx(start)
    assert waypoints[-1].arm_pos == pytest.approx(goal)
    assert max(
        abs(right.arm_pos[-1] - left.arm_pos[-1])
        for left, right in zip(waypoints, waypoints[1:])
    ) <= 0.03
