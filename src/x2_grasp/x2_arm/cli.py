from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .config import ArmSide, X2IKConfig
from .solver import X2ArmIKSolver


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline X2 arm IK demo")
    parser.add_argument("--urdf", type=Path, help="Override URDF path")
    parser.add_argument("--side", choices=[side.value for side in ArmSide], default="right")
    parser.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument(
        "--target-6d",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Target [x, y, z, roll, pitch, yaw]; position is meters and RPY is radians",
    )
    parser.add_argument("--target-offset", nargs=3, type=float, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--target-rpy", nargs=3, type=float, metavar=("ROLL", "PITCH", "YAW"))
    parser.add_argument("--keep-current-rpy", action="store_true")
    parser.add_argument("--orientation-weight", type=float, default=1.0)
    parser.add_argument("--orientation-eps", type=float, default=1e-3)
    parser.add_argument(
        "--joint-margin",
        type=float,
        default=0.02,
        help="Keep arm joints this far (rad) inside URDF mechanical limits",
    )
    parser.add_argument("--arm-pos", nargs=14, type=float, help="Seed arm_pos[14]. Default is ready pose.")
    parser.add_argument("--print-json", action="store_true")
    parser.add_argument("--limits", action="store_true", help="Print arm joint limits and exit")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    default_cfg = X2IKConfig.default_omnipicker()
    cfg = X2IKConfig(
        urdf_path=args.urdf if args.urdf else default_cfg.urdf_path,
        joint_margin=args.joint_margin,
    )

    solver = X2ArmIKSolver(cfg)
    if args.limits:
        for name, lower, upper in solver.effective_joint_limits_for_arm_pos():
            print(f"{name:35s} {lower: .6f} {upper: .6f}")
        return

    side = ArmSide(args.side)
    arm_pos = args.arm_pos if args.arm_pos is not None else solver.ready_arm_pos()
    target_inputs = [
        args.target is not None,
        args.target_6d is not None,
        args.target_offset is not None,
    ]
    if sum(target_inputs) > 1:
        raise SystemExit(
            "Use exactly one of --target, --target-6d, or --target-offset"
        )
    if args.target_6d is not None:
        target = np.asarray(args.target_6d[:3], dtype=float)
        target_rpy = args.target_6d[3:]
    elif args.target is not None:
        target = np.asarray(args.target, dtype=float)
    elif args.target_offset is not None:
        current = np.asarray(solver.fk_xyz(side, arm_pos), dtype=float)
        target = current + np.asarray(args.target_offset, dtype=float)
    else:
        raise SystemExit("Provide --target or --target-offset")

    if args.target_6d is not None and (args.target_rpy is not None or args.keep_current_rpy):
        raise SystemExit("--target-6d already includes orientation")
    if args.target_rpy is not None and args.keep_current_rpy:
        raise SystemExit("Use either --target-rpy or --keep-current-rpy, not both")

    if args.target_6d is None:
        target_rpy = None
        if args.keep_current_rpy:
            target_rpy = solver.fk_rpy(side, arm_pos)
        elif args.target_rpy is not None:
            target_rpy = args.target_rpy

    if args.target_6d is not None:
        result = solver.solve_6d(
            side=side,
            target_6d=args.target_6d,
            current_arm_pos=arm_pos,
            orientation_weight=args.orientation_weight,
            orientation_eps=args.orientation_eps,
        )
    elif target_rpy is not None:
        result = solver.solve_pose(
            side=side,
            target_xyz=target,
            target_rpy=target_rpy,
            current_arm_pos=arm_pos,
            orientation_weight=args.orientation_weight,
            orientation_eps=args.orientation_eps,
        )
    else:
        result = solver.solve_position(side=side, target_xyz=target, current_arm_pos=arm_pos)
    if args.print_json:
        payload = asdict(result)
        payload["side"] = result.side.value
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return

    print(f"success: {result.success}")
    print(f"message: {result.message}")
    print(f"side: {result.side.value}")
    print(f"ee_frame: {result.ee_frame}")
    print(f"iterations: {result.iterations}")
    print(f"error_norm: {result.error_norm:.8f}")
    if result.position_error_norm is not None:
        print(f"position_error_norm: {result.position_error_norm:.8f}")
    if result.orientation_error_norm is not None:
        print(f"orientation_error_norm: {result.orientation_error_norm:.8f}")
    print(f"target_xyz: {result.target_xyz}")
    print(f"final_xyz:  {result.final_xyz}")
    if result.target_rpy is not None:
        print(f"target_rpy: {result.target_rpy}")
    if result.final_rpy is not None:
        print(f"final_rpy:  {result.final_rpy}")
    print("arm_pos[14]:")
    print(np.array2string(np.asarray(result.arm_pos), precision=6, suppress_small=True, separator=", "))


if __name__ == "__main__":
    main()
