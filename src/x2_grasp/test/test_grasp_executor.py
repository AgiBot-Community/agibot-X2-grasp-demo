from types import SimpleNamespace

import pytest

from x2_arm.config import ArmSide
from x2_grasp.grasp_errors import GraspCancelled
from x2_grasp.grasp_executor import execute_grasp


@pytest.mark.parametrize("retracted", [False, True])
@pytest.mark.parametrize("stop", [None, "cancel", "failure"])
def test_holding_return_finishes_before_completion(retracted, stop):
    initial = [0.0] * 14
    clearance = [0.1] * 14
    result = lambda value: SimpleNamespace(arm_pos=[value] * 14)
    plan = SimpleNamespace(
        side=ArmSide.RIGHT, retract_arm_pos=clearance,
        pre_grasp=result(0.2), approach_steps=[result(0.3)],
        grasp=result(0.3), lift_steps=[result(0.4)],
        post_grasp=result(0.4), achieved_lift=0.04,
        high_retract_steps=[result(0.5)] if retracted else [],
        return_segments=[("returning_high", [result(0.6)]),
                         ("return_lowering", [result(0.1)])],
    )
    args = SimpleNamespace(
        duration=4.0, approach_duration=3.0, initial_close_seconds=2.0,
        open_seconds=0.5, grip_close_seconds=2.0, lift_step=0.02,
        cup_grip_close_position=0.1,
    )
    events = []
    canceled = False

    def feedback(stage, detail):
        nonlocal canceled
        events.append((stage, detail))
        if stage.startswith("return") and stop == "cancel":
            canceled = True

    def trajectory(start, goal, duration, **kwargs):
        if stop == "failure" and events[-1][0].startswith("return"):
            raise RuntimeError("return failed")
        events.append(("move", list(start), list(goal)))

    node = SimpleNamespace(
        publish_trajectory=trajectory,
        close_gripper=lambda *a, **kw: events.append(("close",)),
        open_gripper=lambda *a, **kw: events.append(("open",)),
        set_gripper_position=lambda *a, **kw: events.append(("grip",)),
    )
    if stop:
        with pytest.raises(GraspCancelled if stop == "cancel" else RuntimeError):
            execute_grasp(node, initial, plan, args, "cup", lambda: canceled, feedback)
        assert not any(event[0] == "completed" for event in events)
    else:
        execute_grasp(node, initial, plan, args, "cup", lambda: canceled, feedback)
        moves = [event for event in events if event[0] == "move"]
        assert moves[-3:] == [
            ("move", [0.5 if retracted else 0.4] * 14, [0.6] * 14),
            ("move", [0.6] * 14, clearance),
            ("move", clearance, initial),
        ]
        assert events[-1][0] == "completed"
    grip_index = next(i for i, event in enumerate(events) if event[0] == "grip")
    assert not any(event[0] in {"open", "close", "grip"} for event in events[grip_index + 1:])
