from __future__ import annotations

from typing import Iterable

import numpy as np

from .config import ArmSide, X2IKConfig
from .solver import IKResult, X2ArmIKSolver

try:
    from ._x2_ik_native import NativeIKSolver as _NativeIKSolver
except ImportError as exc:
    try:
        from _x2_ik_native import NativeIKSolver as _NativeIKSolver
    except ImportError:
        _NATIVE_IMPORT_ERROR: ImportError | None = exc
        _NativeIKSolver = None
    else:
        _NATIVE_IMPORT_ERROR = None
else:
    _NATIVE_IMPORT_ERROR = None


class NativeBackendUnavailable(ImportError):
    pass


def native_backend_available() -> bool:
    return _NativeIKSolver is not None


class NativeX2ArmIKSolver(X2ArmIKSolver):
    """API-compatible solver using C++ for the measured IK hot path."""

    backend = "native"

    def __init__(self, config: X2IKConfig):
        if _NativeIKSolver is None:
            raise NativeBackendUnavailable(
                "the x2_arm native extension is not installed; build x2_grasp with colcon"
            ) from _NATIVE_IMPORT_ERROR
        super().__init__(config)
        self._native = _NativeIKSolver(
            str(config.urdf_path),
            config.left_ee_frame,
            config.right_ee_frame,
            config.eps,
            config.max_iters,
            config.dt,
            config.damping,
            config.max_step_norm,
            config.joint_margin,
        )

    @staticmethod
    def _native_seed_supported(current_head_pos, q_seed) -> bool:
        return current_head_pos is None and q_seed is None

    def clip_arm_pos(self, arm_pos: Iterable[float]) -> list[float]:
        values = list(arm_pos)
        return self._native.clip_arm_pos(values)

    def fk_xyz(
        self,
        side: ArmSide | str,
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> list[float]:
        if not self._native_seed_supported(current_head_pos, q_seed):
            return super().fk_xyz(
                side,
                current_arm_pos,
                current_head_pos=current_head_pos,
                q_seed=q_seed,
            )
        arm_pos = self.ready_arm_pos() if current_arm_pos is None else list(current_arm_pos)
        return self._native.fk_xyz(ArmSide(side).value, arm_pos)

    def fk_axis(
        self,
        side: ArmSide | str,
        current_arm_pos: Iterable[float] | None = None,
        *,
        local_axis: Iterable[float] = (0.0, 0.0, 1.0),
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> list[float]:
        if not self._native_seed_supported(current_head_pos, q_seed):
            return super().fk_axis(
                side,
                current_arm_pos,
                local_axis=local_axis,
                current_head_pos=current_head_pos,
                q_seed=q_seed,
            )
        arm_pos = self.ready_arm_pos() if current_arm_pos is None else list(current_arm_pos)
        return self._native.fk_axis(
            ArmSide(side).value, arm_pos, list(local_axis)
        )

    def solve_axis(
        self,
        side: ArmSide | str,
        target_xyz: Iterable[float],
        target_axis: Iterable[float],
        current_arm_pos: Iterable[float] | None = None,
        *,
        local_axis: Iterable[float] = (0.0, 0.0, 1.0),
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
        orientation_weight: float = 1.0,
        orientation_eps: float = 0.05,
    ) -> IKResult:
        if not self._native_seed_supported(current_head_pos, q_seed):
            return super().solve_axis(
                side,
                target_xyz,
                target_axis,
                current_arm_pos,
                local_axis=local_axis,
                current_head_pos=current_head_pos,
                q_seed=q_seed,
                orientation_weight=orientation_weight,
                orientation_eps=orientation_eps,
            )
        side = ArmSide(side)
        arm_pos = self.ready_arm_pos() if current_arm_pos is None else list(current_arm_pos)
        target = list(target_xyz)
        axis = list(target_axis)
        native = self._native.solve_axis(
            side.value,
            target,
            axis,
            arm_pos,
            list(local_axis),
            orientation_weight,
            orientation_eps,
        )
        normalized_axis = np.asarray(axis, dtype=float)
        normalized_axis /= np.linalg.norm(normalized_axis)
        active_arm = native.arm_pos[:7] if side == ArmSide.LEFT else native.arm_pos[7:]
        return IKResult(
            success=native.success,
            side=side,
            arm_pos=native.arm_pos,
            active_arm=active_arm,
            target_xyz=target,
            final_xyz=list(native.final_xyz),
            error_norm=native.error_norm,
            iterations=native.iterations,
            ee_frame=self.config.frame_for_side(side),
            message="converged" if native.success else "max iterations reached",
            final_rpy=list(native.final_rpy),
            position_error_norm=native.position_error_norm,
            orientation_error_norm=native.orientation_error_norm,
            target_axis=normalized_axis.tolist(),
            final_axis=list(native.final_axis),
        )


def create_ik_solver(
    config: X2IKConfig, backend: str = "auto"
) -> X2ArmIKSolver:
    if backend not in {"auto", "native", "python"}:
        raise ValueError("IK backend must be auto, native, or python")
    if backend == "python":
        solver = X2ArmIKSolver(config)
        solver.backend = "python"
        return solver
    if backend == "native" or native_backend_available():
        return NativeX2ArmIKSolver(config)
    solver = X2ArmIKSolver(config)
    solver.backend = "python"
    return solver
