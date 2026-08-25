#!/usr/bin/env python3
"""Measure hot-path X2 IK operations without requiring ROS or AimDK."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import sys
import time


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "x2_grasp"
sys.path.insert(0, str(PACKAGE_ROOT))

import pinocchio  # noqa: E402

from x2_arm import ArmSide, X2IKConfig, create_ik_solver  # noqa: E402
from x2_grasp.grasp_planner import ik_seed_candidates  # noqa: E402


def _measure(operation, iterations: int, warmup: int) -> dict[str, float]:
    for _ in range(warmup):
        operation()
    samples = []
    for _ in range(iterations):
        started = time.perf_counter_ns()
        operation()
        samples.append(time.perf_counter_ns() - started)
    samples.sort()
    p95_index = min(len(samples) - 1, int(len(samples) * 0.95))
    return {
        "iterations": iterations,
        "mean_us": statistics.fmean(samples) / 1_000.0,
        "median_us": statistics.median(samples) / 1_000.0,
        "p95_us": samples[p95_index] / 1_000.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--backend", choices=("python", "native", "auto"), default="python"
    )
    parser.add_argument("--micro-iterations", type=int, default=5_000)
    parser.add_argument("--solve-iterations", type=int, default=100)
    parser.add_argument("--warmup", type=int, default=10)
    args = parser.parse_args()

    side = ArmSide(args.side)
    solver = create_ik_solver(X2IKConfig.default_omnipicker(), args.backend)
    seed = solver.ready_arm_pos()
    current_xyz = solver.fk_xyz(side, seed)
    current_axis = solver.fk_axis(side, seed)
    target_xyz = [current_xyz[0] + 0.005, current_xyz[1], current_xyz[2] + 0.005]

    class PlannerNode:
        def __init__(self, value):
            self.solver = value

    planner_node = PlannerNode(solver)

    def first_seed():
        return next(
            iter(ik_seed_candidates(planner_node, seed, seed, 0.12, side))
        )

    solve_iterations = []

    def solve_axis():
        result = solver.solve_axis(side, target_xyz, current_axis, seed)
        if not result.success:
            raise RuntimeError(result.message)
        solve_iterations.append(result.iterations)

    result = {
        "pinocchio_version": pinocchio.__version__,
        "requested_backend": args.backend,
        "backend": solver.backend,
        "side": side.value,
        "operations": {
            "clip_arm_pos": _measure(
                lambda: solver.clip_arm_pos(seed),
                args.micro_iterations,
                args.warmup,
            ),
            "fk_xyz": _measure(
                lambda: solver.fk_xyz(side, seed),
                args.micro_iterations,
                args.warmup,
            ),
            "first_ik_seed": _measure(
                first_seed,
                args.micro_iterations,
                args.warmup,
            ),
            "solve_axis": _measure(
                solve_axis,
                args.solve_iterations,
                args.warmup,
            ),
        },
    }
    result["solve_axis_iterations_mean"] = statistics.fmean(solve_iterations)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
