from types import SimpleNamespace

from x2_arm.config import ArmSide
import pytest

from x2_grasp.grasp_planner import (
    check_reachable,
    ik_seed_candidates,
    plan_grasp,
    validate_target,
)


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class _Solver:
    @staticmethod
    def clip_arm_pos(arm_pos):
        return list(arm_pos)


class _Node:
    def __init__(self):
        self.solver = _Solver()
        self.node = SimpleNamespace(get_logger=lambda: _Logger())


def _plan(side, state):
    result = SimpleNamespace(arm_pos=state)
    return SimpleNamespace(
        side=side,
        retract_arm_pos=state,
        pre_grasp=result,
        approach_steps=[result],
        lift_steps=[result],
        high_retract_steps=[],
    )


def test_ik_seed_candidates_only_perturb_selected_arm():
    node = _Node()
    primary = [float(value) for value in range(14)]

    for side, unchanged in (
        (ArmSide.LEFT, slice(7, 14)),
        (ArmSide.RIGHT, slice(0, 7)),
    ):
        candidates = list(
            ik_seed_candidates(node, primary, primary, 0.12, side)
        )

        assert len(candidates) == 7
        assert all(candidate[unchanged] == primary[unchanged] for candidate in candidates)


def test_ik_seed_candidates_are_generated_lazily():
    node = _Node()
    calls = []
    node.solver.clip_arm_pos = lambda candidate: calls.append(candidate) or candidate
    primary = [0.0] * 14

    candidates = iter(
        ik_seed_candidates(node, primary, primary, 0.12, ArmSide.RIGHT)
    )

    assert next(candidates) == primary
    assert calls == []
    next(candidates)
    assert len(calls) == 1


def test_reachability_is_mirrored_for_left_and_right_arms():
    args = SimpleNamespace()
    check_reachable([0.3, -0.2, 0.3], [0.3, -0.2, 0.2], args, "test", ArmSide.RIGHT)
    check_reachable([0.3, 0.2, 0.3], [0.3, 0.2, 0.2], args, "test", ArmSide.LEFT)

    with pytest.raises(RuntimeError, match="右臂"):
        check_reachable(
            [0.3, 0.2, 0.3], [0.3, 0.2, 0.2], args, "test", ArmSide.RIGHT
        )
    with pytest.raises(RuntimeError, match="左臂"):
        check_reachable(
            [0.3, -0.2, 0.3], [0.3, -0.2, 0.2], args, "test", ArmSide.LEFT
        )


def test_target_validation_accepts_left_workspace():
    validate_target([0.3, 0.4, 0.2], "base_link", "test")


def test_auto_arm_selection_uses_plan_with_less_joint_travel(monkeypatch):
    current = [0.0] * 14

    def fake_plan(_node, _current, _target, _args, _source, side, *_rest):
        distance = 0.1 if side == ArmSide.LEFT else 0.4
        state = list(current)
        active = slice(0, 7) if side == ArmSide.LEFT else slice(7, 14)
        state[active] = [distance] * 7
        return _plan(side, state)

    monkeypatch.setattr(
        "x2_grasp.grasp_planner._plan_grasp_for_side", fake_plan
    )

    plan = plan_grasp(
        _Node(),
        current,
        [0.3, 0.2, 0.2],
        SimpleNamespace(arm_side="auto"),
        "grounding",
    )

    assert plan.side == ArmSide.LEFT
    assert plan.selection_cost < 1.0


def test_auto_arm_selection_falls_back_when_preferred_side_fails(monkeypatch):
    current = [0.0] * 14

    def fake_plan(_node, _current, _target, _args, _source, side, *_rest):
        if side == ArmSide.LEFT:
            raise RuntimeError("left unreachable")
        return _plan(side, list(current))

    monkeypatch.setattr(
        "x2_grasp.grasp_planner._plan_grasp_for_side", fake_plan
    )

    plan = plan_grasp(
        _Node(),
        current,
        [0.3, 0.2, 0.2],
        SimpleNamespace(arm_side="auto"),
        "grounding",
    )

    assert plan.side == ArmSide.RIGHT
