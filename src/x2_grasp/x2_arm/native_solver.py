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
    """API-compatible solver using C++ for Pinocchio numerical operations."""

    backend = "native"

    def __init__(self, config: X2IKConfig):
        if _NativeIKSolver is None:
            raise NativeBackendUnavailable(
                "the x2_arm native extension is not installed; build x2_grasp with colcon"
            ) from _NATIVE_IMPORT_ERROR
        if not config.urdf_path.exists():
            raise FileNotFoundError(f"URDF not found: {config.urdf_path}")
        self.config = config
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
        self._arm_q_idxs = np.asarray(self._native.arm_q_indices(), dtype=int)
        self._python_fallback: X2ArmIKSolver | None = None

    def _python_solver(self) -> X2ArmIKSolver:
        if self._python_fallback is None:
            self._python_fallback = X2ArmIKSolver(self.config)
        return self._python_fallback

    @staticmethod
    def _native_seed_supported(current_head_pos, q_seed) -> bool:
        return current_head_pos is None and q_seed is None

    def clip_arm_pos(self, arm_pos: Iterable[float]) -> list[float]:
        values = list(arm_pos)
        return self._native.clip_arm_pos(values)

    @staticmethod
    def _vector(values: Iterable[float], name: str) -> list[float]:
        vector = np.asarray(list(values), dtype=float)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"{name} must contain three finite values, got {vector}")
        return vector.tolist()

    def q_from_arm_pos(
        self,
        arm_pos: Iterable[float],
        current_head_pos: Iterable[float] | None = None,
    ) -> np.ndarray:
        if current_head_pos is not None:
            return self._python_solver().q_from_arm_pos(arm_pos, current_head_pos)
        return np.asarray(
            self._native.configuration_from_arm_pos(list(arm_pos)), dtype=float
        )

    def arm_pos_from_q(self, q: np.ndarray) -> list[float]:
        return np.asarray(q)[self._arm_q_idxs].astype(float).tolist()

    def ready_arm_pos(self) -> list[float]:
        return list(self.config.ready_arm_pos())

    def joint_limits_for_arm_pos(self) -> list[tuple[str, float, float]]:
        return [tuple(limit) for limit in self._native.joint_limits()]

    def effective_joint_limits_for_arm_pos(self) -> list[tuple[str, float, float]]:
        return [tuple(limit) for limit in self._native.effective_joint_limits()]

    def fk_xyz(
        self,
        side: ArmSide | str,
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> list[float]:
        if not self._native_seed_supported(current_head_pos, q_seed):
            return self._python_solver().fk_xyz(
                side,
                current_arm_pos,
                current_head_pos=current_head_pos,
                q_seed=q_seed,
            )
        arm_pos = self.ready_arm_pos() if current_arm_pos is None else list(current_arm_pos)
        return self._native.fk_xyz(ArmSide(side).value, arm_pos)

    def fk_rpy(
        self,
        side: ArmSide | str,
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> list[float]:
        if not self._native_seed_supported(current_head_pos, q_seed):
            return self._python_solver().fk_rpy(
                side,
                current_arm_pos,
                current_head_pos=current_head_pos,
                q_seed=q_seed,
            )
        arm_pos = self.ready_arm_pos() if current_arm_pos is None else list(current_arm_pos)
        return self._native.fk_rpy(ArmSide(side).value, arm_pos)

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
            return self._python_solver().fk_axis(
                side,
                current_arm_pos,
                local_axis=local_axis,
                current_head_pos=current_head_pos,
                q_seed=q_seed,
            )
        arm_pos = self.ready_arm_pos() if current_arm_pos is None else list(current_arm_pos)
        axis = self._vector(local_axis, "local_axis")
        return self._native.fk_axis(
            ArmSide(side).value, arm_pos, axis
        )

    @staticmethod
    def _active_arm(side: ArmSide, arm_pos: list[float]) -> list[float]:
        return arm_pos[:7] if side == ArmSide.LEFT else arm_pos[7:]

    def solve_position(
        self,
        side: ArmSide | str,
        target_xyz: Iterable[float],
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> IKResult:
        if not self._native_seed_supported(current_head_pos, q_seed):
            return self._python_solver().solve_position(
                side,
                target_xyz,
                current_arm_pos,
                current_head_pos=current_head_pos,
                q_seed=q_seed,
            )
        side = ArmSide(side)
        target = self._vector(target_xyz, "target_xyz")
        arm_pos = self.ready_arm_pos() if current_arm_pos is None else list(current_arm_pos)
        native = self._native.solve_position(side.value, target, arm_pos)
        return IKResult(
            success=native.success,
            side=side,
            arm_pos=native.arm_pos,
            active_arm=self._active_arm(side, native.arm_pos),
            target_xyz=target,
            final_xyz=list(native.final_xyz),
            error_norm=native.error_norm,
            iterations=native.iterations,
            ee_frame=self.config.frame_for_side(side),
            message="converged" if native.success else "max iterations reached",
        )

    def solve_pose(
        self,
        side: ArmSide | str,
        target_xyz: Iterable[float],
        target_rpy: Iterable[float],
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
        orientation_weight: float = 1.0,
        orientation_eps: float = 1e-3,
    ) -> IKResult:
        if not self._native_seed_supported(current_head_pos, q_seed):
            return self._python_solver().solve_pose(
                side,
                target_xyz,
                target_rpy,
                current_arm_pos,
                current_head_pos=current_head_pos,
                q_seed=q_seed,
                orientation_weight=orientation_weight,
                orientation_eps=orientation_eps,
            )
        side = ArmSide(side)
        target = self._vector(target_xyz, "target_xyz")
        rpy = self._vector(target_rpy, "target_rpy")
        arm_pos = self.ready_arm_pos() if current_arm_pos is None else list(current_arm_pos)
        native = self._native.solve_pose(
            side.value,
            target,
            rpy,
            arm_pos,
            orientation_weight,
            orientation_eps,
        )
        return IKResult(
            success=native.success,
            side=side,
            arm_pos=native.arm_pos,
            active_arm=self._active_arm(side, native.arm_pos),
            target_xyz=target,
            final_xyz=list(native.final_xyz),
            error_norm=native.error_norm,
            iterations=native.iterations,
            ee_frame=self.config.frame_for_side(side),
            message="converged" if native.success else "max iterations reached",
            target_rpy=rpy,
            final_rpy=list(native.final_rpy),
            position_error_norm=native.position_error_norm,
            orientation_error_norm=native.orientation_error_norm,
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
            return self._python_solver().solve_axis(
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
        target = self._vector(target_xyz, "target_xyz")
        axis = self._vector(target_axis, "target_axis")
        tool_axis = self._vector(local_axis, "local_axis")
        native = self._native.solve_axis(
            side.value,
            target,
            axis,
            arm_pos,
            tool_axis,
            orientation_weight,
            orientation_eps,
        )
        normalized_axis = np.asarray(axis, dtype=float)
        normalized_axis /= np.linalg.norm(normalized_axis)
        return IKResult(
            success=native.success,
            side=side,
            arm_pos=native.arm_pos,
            active_arm=self._active_arm(side, native.arm_pos),
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
