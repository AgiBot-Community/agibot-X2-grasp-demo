from types import SimpleNamespace

import pytest

from x2_arm.config import ArmSide
from x2_grasp.grasp_errors import GraspCancelled
from x2_grasp.grasp_planner import plan_holding_return


def _setup(monkeypatch, *, bow=False, fail=False):
    calls = []

    def fk(_side, joints):
        xyz = list(joints[:3])
        if bow:
            xyz[2] -= 0.10 * max(0.0, 1.0 - abs(xyz[0] - 0.3) / 0.1)
        return xyz

    def segment(_node, start, target, _seed, _retract, _args, stage, *_rest):
        calls.append((stage, list(start), list(target)))
        if fail and stage == "returning_high":
            raise RuntimeError("unreachable return")
        return [SimpleNamespace(arm_pos=list(target) + [0.0] * 11)]

    monkeypatch.setattr("x2_grasp.grasp_planner.solve_cartesian_segment", segment)
    node = SimpleNamespace(solver=SimpleNamespace(fk_xyz=fk))
    args = SimpleNamespace(initial_upward=0.15, high_retract_position_tolerance=0.005)
    return node, args, calls


@pytest.mark.parametrize("start_height,height", [(0.32, 0.55), (0.65, 0.65)])
def test_return_raises_then_moves_above_initial_then_descends(monkeypatch, start_height, height):
    node, args, calls = _setup(monkeypatch)
    initial = [0.2, -0.2, 0.4] + [0.0] * 11
    start = SimpleNamespace(arm_pos=[0.4, -0.3, start_height] + [0.0] * 11)
    segments = plan_holding_return(node, start, initial, initial, args, ArmSide.RIGHT)
    assert [stage for stage, _results in segments] == [
        "return_raising", "returning_high", "return_lowering",
    ]
    assert calls[0][1] == [0.4, -0.3, start_height]
    assert calls[0][2] == pytest.approx([0.4, -0.3, height])
    assert calls[1][2] == pytest.approx([0.2, -0.2, height])
    assert calls[2][2] == initial[:3]


def test_return_rejects_downward_bowing_between_valid_endpoints(monkeypatch):
    node, args, _calls = _setup(monkeypatch, bow=True)
    initial = [0.2, -0.2, 0.4] + [0.0] * 11
    start = SimpleNamespace(arm_pos=[0.4, -0.2, 0.55] + [0.0] * 11)
    with pytest.raises(RuntimeError, match="Cartesian tolerance"):
        plan_holding_return(node, start, initial, initial, args, ArmSide.RIGHT)


def test_return_does_not_ignore_unreachable_segment(monkeypatch):
    node, args, _calls = _setup(monkeypatch, fail=True)
    initial = [0.2, -0.2, 0.4] + [0.0] * 11
    start = SimpleNamespace(arm_pos=[0.4, -0.2, 0.55] + [0.0] * 11)
    with pytest.raises(RuntimeError, match="unreachable return"):
        plan_holding_return(node, start, initial, initial, args, ArmSide.RIGHT)


def test_return_validation_honors_cancel(monkeypatch):
    node, args, _calls = _setup(monkeypatch)
    initial = [0.2, -0.2, 0.4] + [0.0] * 11
    start = SimpleNamespace(arm_pos=[0.4, -0.2, 0.55] + [0.0] * 11)
    with pytest.raises(GraspCancelled):
        plan_holding_return(node, start, initial, initial, args, ArmSide.RIGHT, lambda: True)
