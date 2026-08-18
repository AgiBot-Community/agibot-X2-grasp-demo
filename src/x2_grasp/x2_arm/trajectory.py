from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class ArmWaypoint:
    arm_pos: list[float]
    duration: float


def interpolate_arm_pos(
    start: Iterable[float],
    goal: Iterable[float],
    *,
    duration: float = 2.0,
    rate_hz: float = 50.0,
    max_delta_per_step: float = 0.03,
) -> list[ArmWaypoint]:
    for name, value in (
        ("duration", duration),
        ("rate_hz", rate_hz),
        ("max_delta_per_step", max_delta_per_step),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a positive finite value")
    start_arr = np.asarray(list(start), dtype=float)
    goal_arr = np.asarray(list(goal), dtype=float)
    if start_arr.shape != (14,) or goal_arr.shape != (14,):
        raise ValueError("start and goal must be length-14 arm_pos arrays")
    if not np.all(np.isfinite(start_arr)) or not np.all(np.isfinite(goal_arr)):
        raise ValueError("start and goal must contain only finite values")

    delta = goal_arr - start_arr
    min_steps_by_time = max(2, int(np.ceil(duration * rate_hz)) + 1)
    # smoothstep has a maximum derivative of 1.5, so a linear step count would
    # violate max_delta_per_step around the middle of the trajectory.
    min_steps_by_delta = max(
        2,
        int(
            np.ceil(1.5 * float(np.max(np.abs(delta))) / max_delta_per_step)
        )
        + 1,
    )
    steps = max(min_steps_by_time, min_steps_by_delta)
    dt = duration / max(1, steps - 1)

    waypoints = []
    for i in range(steps):
        s = i / max(1, steps - 1)
        smooth = s * s * (3.0 - 2.0 * s)
        q = start_arr + delta * smooth
        waypoints.append(ArmWaypoint(arm_pos=q.tolist(), duration=dt))
    return waypoints
