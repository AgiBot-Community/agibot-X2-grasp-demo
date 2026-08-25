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
from x2_arm.trajectory import interpolate_arm_pos  # noqa: E402
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
    parser.add_argument("--init-iterations", type=int, default=10)
    parser.add_argument("--warmup", type=int, default=10)
    args = parser.parse_args()

    side = ArmSide(args.side)
    config = X2IKConfig.default_omnipicker()
    solver = create_ik_solver(config, args.backend)
    seed = solver.ready_arm_pos()
    current_xyz = solver.fk_xyz(side, seed)
    current_rpy = solver.fk_rpy(side, seed)
    current_axis = solver.fk_axis(side, seed)
    seed_q = solver.q_from_arm_pos(seed)
    target_xyz = [current_xyz[0] + 0.005, current_xyz[1], current_xyz[2] + 0.005]

    class PlannerNode:
        def __init__(self, value):
            self.solver = value

    planner_node = PlannerNode(solver)

    def first_seed():
        return next(
            iter(ik_seed_candidates(planner_node, seed, seed, 0.12, side))
        )

    solve_iterations = {"position": [], "pose": [], "pose_6d": [], "axis": []}

    def solve_position():
        result = solver.solve_position(side, target_xyz, seed)
        if not result.success:
            raise RuntimeError(result.message)
        solve_iterations["position"].append(result.iterations)

    def solve_pose():
        result = solver.solve_pose(side, target_xyz, current_rpy, seed)
        if not result.success:
            raise RuntimeError(result.message)
        solve_iterations["pose"].append(result.iterations)

    def solve_6d():
        result = solver.solve_6d(side, target_xyz + current_rpy, seed)
        if not result.success:
            raise RuntimeError(result.message)
        solve_iterations["pose_6d"].append(result.iterations)

    def solve_axis():
        result = solver.solve_axis(side, target_xyz, current_axis, seed)
        if not result.success:
            raise RuntimeError(result.message)
        solve_iterations["axis"].append(result.iterations)

    chain_waypoints = [
        [
            current_xyz[0] + 0.01 * step / 8,
            current_xyz[1],
            current_xyz[2] + 0.01 * step / 8,
        ]
        for step in range(1, 9)
    ]
    trajectory_goal = [value + 0.5 for value in seed]

    def solve_cartesian_chain():
        chain_seed = seed
        for waypoint in chain_waypoints:
            result = solver.solve_axis(side, waypoint, current_axis, chain_seed)
            if not result.success:
                raise RuntimeError(result.message)
            chain_seed = result.arm_pos

    result = {
        "pinocchio_version": pinocchio.__version__,
        "requested_backend": args.backend,
        "backend": solver.backend,
        "side": side.value,
        "solver_init": _measure(
            lambda: create_ik_solver(config, args.backend),
            args.init_iterations,
            min(args.warmup, 2),
        ),
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
            "fk_rpy": _measure(
                lambda: solver.fk_rpy(side, seed),
                args.micro_iterations,
                args.warmup,
            ),
            "fk_axis": _measure(
                lambda: solver.fk_axis(side, seed),
                args.micro_iterations,
                args.warmup,
            ),
            "q_from_arm_pos": _measure(
                lambda: solver.q_from_arm_pos(seed),
                args.micro_iterations,
                args.warmup,
            ),
            "arm_pos_from_q": _measure(
                lambda: solver.arm_pos_from_q(seed_q),
                args.micro_iterations,
                args.warmup,
            ),
            "first_ik_seed": _measure(
                first_seed,
                args.micro_iterations,
                args.warmup,
            ),
            "trajectory_interpolation_101": _measure(
                lambda: interpolate_arm_pos(seed, trajectory_goal),
                args.micro_iterations,
                args.warmup,
            ),
            "solve_axis": _measure(
                solve_axis,
                args.solve_iterations,
                args.warmup,
            ),
            "solve_position": _measure(
                solve_position,
                args.solve_iterations,
                args.warmup,
            ),
            "solve_pose": _measure(
                solve_pose,
                args.solve_iterations,
                args.warmup,
            ),
            "solve_6d": _measure(
                solve_6d,
                args.solve_iterations,
                args.warmup,
            ),
            "solve_cartesian_chain_8": _measure(
                solve_cartesian_chain,
                args.solve_iterations,
                args.warmup,
            ),
        },
    }
    result["solver_iterations_mean"] = {
        name: statistics.fmean(values) for name, values in solve_iterations.items()
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
