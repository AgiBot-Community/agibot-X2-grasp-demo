from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np
import pinocchio as pin

from .config import ARM_POS_ORDER, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, ArmSide, X2IKConfig


TARGET_VECTOR_LENGTH = 6


def split_target_vector(target_vector: Iterable[float]) -> tuple[list[float], list[float]]:
    """Validate and split ``[x, y, z, roll, pitch, yaw]`` in radians."""

    try:
        values = np.asarray(list(target_vector), dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "target_vector must contain six finite numeric values "
            "[x, y, z, roll, pitch, yaw]"
        ) from exc
    if values.shape != (TARGET_VECTOR_LENGTH,):
        raise ValueError(
            "target_vector must have length 6 "
            "[x, y, z, roll, pitch, yaw], "
            f"got {values}"
        )
    if not np.all(np.isfinite(values)):
        raise ValueError("target_vector must contain only finite values")
    return values[:3].tolist(), values[3:].tolist()


@dataclass(frozen=True)
class IKResult:
    success: bool
    side: ArmSide
    arm_pos: list[float]
    active_arm: list[float]
    target_xyz: list[float]
    final_xyz: list[float]
    error_norm: float
    iterations: int
    ee_frame: str
    message: str = ""
    target_rpy: list[float] | None = None
    final_rpy: list[float] | None = None
    position_error_norm: float | None = None
    orientation_error_norm: float | None = None
    target_axis: list[float] | None = None
    final_axis: list[float] | None = None


class X2ArmIKSolver:
    def __init__(self, config: X2IKConfig):
        self.config = config
        if not config.urdf_path.exists():
            raise FileNotFoundError(
                "URDF not found: "
                f"{config.urdf_path}\n"
                "The kinematic model must exist at "
                "`urdf/"
                "x2_ultra_plus_omnipicker_omnipicker.urdf`."
            )
        self.model = pin.buildModelFromUrdf(str(config.urdf_path))
        self.data = self.model.createData()
        self._validate_model()
        self._validate_joint_margin()

    def solve_position(
        self,
        side: ArmSide | str,
        target_xyz: Iterable[float],
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> IKResult:
        side = ArmSide(side)
        target = np.asarray(list(target_xyz), dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError(f"target_xyz must contain three finite values, got {target}")

        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_name = self.config.frame_for_side(side)
        frame_id = self.model.getFrameId(frame_name)
        active_v_idxs = self._active_velocity_indices(side)

        err_norm = math.inf
        iterations = 0
        success = False
        for iterations in range(1, self.config.max_iters + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current = self.data.oMf[frame_id].translation
            err = target - current
            err_norm = float(np.linalg.norm(err))
            if err_norm < self.config.eps:
                success = True
                break

            jacobian6 = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            jacobian = jacobian6[:3, :]
            velocity = self._damped_least_squares(jacobian, err, active_v_idxs)
            step = velocity * self.config.dt
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.config.max_step_norm:
                step *= self.config.max_step_norm / step_norm

            q = pin.integrate(self.model, q, step)
            q = self._clip_q(q)

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        final = self.data.oMf[frame_id].translation.copy()
        arm_pos = self.arm_pos_from_q(q)
        active_arm = arm_pos[:7] if side == ArmSide.LEFT else arm_pos[7:]

        msg = "converged" if success else "max iterations reached"
        return IKResult(
            success=success,
            side=side,
            arm_pos=arm_pos,
            active_arm=active_arm,
            target_xyz=target.tolist(),
            final_xyz=final.tolist(),
            error_norm=err_norm,
            iterations=iterations,
            ee_frame=frame_name,
            message=msg,
        )

    def solve_6d(
        self,
        side: ArmSide | str,
        target_6d: Iterable[float],
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
        orientation_weight: float = 1.0,
        orientation_eps: float = 1e-3,
    ) -> IKResult:
        """Solve a full pose from ``[x, y, z, roll, pitch, yaw]``.

        Position is in meters and RPY is in radians, expressed in the URDF
        base frame.  The result keeps the SDK's full 14-joint arm ordering.
        """

        target_xyz, target_rpy = split_target_vector(target_6d)
        return self.solve_pose(
            side=side,
            target_xyz=target_xyz,
            target_rpy=target_rpy,
            current_arm_pos=current_arm_pos,
            current_head_pos=current_head_pos,
            q_seed=q_seed,
            orientation_weight=orientation_weight,
            orientation_eps=orientation_eps,
        )

    def solve_vector(
        self,
        side: ArmSide | str,
        target_vector: Iterable[float],
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
        orientation_weight: float = 1.0,
        orientation_eps: float = 1e-3,
    ) -> IKResult:
        """Alias for :meth:`solve_6d` using the generic vector name."""

        return self.solve_6d(
            side=side,
            target_6d=target_vector,
            current_arm_pos=current_arm_pos,
            current_head_pos=current_head_pos,
            q_seed=q_seed,
            orientation_weight=orientation_weight,
            orientation_eps=orientation_eps,
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
        side = ArmSide(side)
        target = np.asarray(list(target_xyz), dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError(f"target_xyz must contain three finite values, got {target}")
        target_rpy_arr = np.asarray(list(target_rpy), dtype=float)
        if target_rpy_arr.shape != (3,) or not np.all(np.isfinite(target_rpy_arr)):
            raise ValueError(
                f"target_rpy must contain three finite values, got {target_rpy_arr}"
            )
        if (
            not math.isfinite(orientation_weight)
            or not math.isfinite(orientation_eps)
            or orientation_weight <= 0.0
            or orientation_eps <= 0.0
        ):
            raise ValueError("orientation_weight and orientation_eps must be positive finite values")

        target_rotation = pin.rpy.rpyToMatrix(*target_rpy_arr.tolist())
        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_name = self.config.frame_for_side(side)
        frame_id = self.model.getFrameId(frame_name)
        active_v_idxs = self._active_velocity_indices(side)

        err_norm = math.inf
        pos_err_norm = math.inf
        rot_err_norm = math.inf
        iterations = 0
        success = False
        for iterations in range(1, self.config.max_iters + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current_pose = self.data.oMf[frame_id]
            pos_err = target - current_pose.translation
            rot_err = pin.log3(target_rotation @ current_pose.rotation.T)
            pos_err_norm = float(np.linalg.norm(pos_err))
            rot_err_norm = float(np.linalg.norm(rot_err))
            err = np.concatenate([pos_err, orientation_weight * rot_err])
            err_norm = float(np.linalg.norm(err))
            if pos_err_norm < self.config.eps and rot_err_norm < orientation_eps:
                success = True
                break

            jacobian = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            weighted_jacobian = jacobian.copy()
            weighted_jacobian[3:, :] *= orientation_weight
            velocity = self._damped_least_squares(weighted_jacobian, err, active_v_idxs)
            step = velocity * self.config.dt
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.config.max_step_norm:
                step *= self.config.max_step_norm / step_norm

            q = pin.integrate(self.model, q, step)
            q = self._clip_q(q)

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        final_pose = self.data.oMf[frame_id]
        final_xyz = final_pose.translation.copy()
        final_rpy = pin.rpy.matrixToRpy(final_pose.rotation).tolist()
        arm_pos = self.arm_pos_from_q(q)
        active_arm = arm_pos[:7] if side == ArmSide.LEFT else arm_pos[7:]

        msg = "converged" if success else "max iterations reached"
        return IKResult(
            success=success,
            side=side,
            arm_pos=arm_pos,
            active_arm=active_arm,
            target_xyz=target.tolist(),
            final_xyz=final_xyz.tolist(),
            error_norm=err_norm,
            iterations=iterations,
            ee_frame=frame_name,
            message=msg,
            target_rpy=target_rpy_arr.tolist(),
            final_rpy=final_rpy,
            position_error_norm=pos_err_norm,
            orientation_error_norm=rot_err_norm,
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
        """Solve position and one tool-axis direction, leaving roll unconstrained."""

        side = ArmSide(side)
        target = np.asarray(list(target_xyz), dtype=float)
        axis = np.asarray(list(target_axis), dtype=float)
        tool_axis = np.asarray(list(local_axis), dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError(f"target_xyz must contain three finite values, got {target}")
        if axis.shape != (3,) or not np.all(np.isfinite(axis)):
            raise ValueError(f"target_axis must contain three finite values, got {axis}")
        if tool_axis.shape != (3,) or not np.all(np.isfinite(tool_axis)):
            raise ValueError(f"local_axis must contain three finite values, got {tool_axis}")
        axis_norm = float(np.linalg.norm(axis))
        tool_axis_norm = float(np.linalg.norm(tool_axis))
        if axis_norm < 1e-9 or tool_axis_norm < 1e-9:
            raise ValueError("target_axis and local_axis must be non-zero")
        if (
            not math.isfinite(orientation_weight)
            or not math.isfinite(orientation_eps)
            or orientation_weight <= 0.0
            or orientation_eps <= 0.0
        ):
            raise ValueError("orientation_weight and orientation_eps must be positive finite values")
        axis /= axis_norm
        tool_axis /= tool_axis_norm

        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_name = self.config.frame_for_side(side)
        frame_id = self.model.getFrameId(frame_name)
        active_v_idxs = self._active_velocity_indices(side)

        err_norm = math.inf
        pos_err_norm = math.inf
        axis_err_norm = math.inf
        iterations = 0
        success = False
        for iterations in range(1, self.config.max_iters + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current_pose = self.data.oMf[frame_id]
            current_axis = current_pose.rotation @ tool_axis
            current_axis /= np.linalg.norm(current_axis)
            pos_err = target - current_pose.translation
            dot = float(np.clip(np.dot(current_axis, axis), -1.0, 1.0))
            axis_err_norm = math.acos(dot)
            axis_err = np.cross(current_axis, axis)
            pos_err_norm = float(np.linalg.norm(pos_err))
            err = np.concatenate([pos_err, orientation_weight * axis_err])
            err_norm = float(np.linalg.norm(err))
            if pos_err_norm < self.config.eps and axis_err_norm < orientation_eps:
                success = True
                break

            jacobian = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            axis_projection = np.eye(3) - np.outer(current_axis, current_axis)
            axis_jacobian = axis_projection @ jacobian[3:, :]
            constrained_jacobian = np.vstack(
                (jacobian[:3, :], orientation_weight * axis_jacobian)
            )
            velocity = self._damped_least_squares(
                constrained_jacobian, err, active_v_idxs
            )
            step = velocity * self.config.dt
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.config.max_step_norm:
                step *= self.config.max_step_norm / step_norm
            q = self._clip_q(pin.integrate(self.model, q, step))

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        final_pose = self.data.oMf[frame_id]
        final_axis = final_pose.rotation @ tool_axis
        final_axis /= np.linalg.norm(final_axis)
        final_xyz = final_pose.translation.copy()
        final_rpy = pin.rpy.matrixToRpy(final_pose.rotation).tolist()
        arm_pos = self.arm_pos_from_q(q)
        active_arm = arm_pos[:7] if side == ArmSide.LEFT else arm_pos[7:]
        return IKResult(
            success=success,
            side=side,
            arm_pos=arm_pos,
            active_arm=active_arm,
            target_xyz=target.tolist(),
            final_xyz=final_xyz.tolist(),
            error_norm=err_norm,
            iterations=iterations,
            ee_frame=frame_name,
            message="converged" if success else "max iterations reached",
            final_rpy=final_rpy,
            position_error_norm=pos_err_norm,
            orientation_error_norm=axis_err_norm,
            target_axis=axis.tolist(),
            final_axis=final_axis.tolist(),
        )

    def fk_xyz(
        self,
        side: ArmSide | str,
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> list[float]:
        side = ArmSide(side)
        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_id = self.model.getFrameId(self.config.frame_for_side(side))
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[frame_id].translation.copy().tolist()

    def fk_rpy(
        self,
        side: ArmSide | str,
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> list[float]:
        side = ArmSide(side)
        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_id = self.model.getFrameId(self.config.frame_for_side(side))
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return pin.rpy.matrixToRpy(self.data.oMf[frame_id].rotation).tolist()

    def fk_axis(
        self,
        side: ArmSide | str,
        current_arm_pos: Iterable[float] | None = None,
        *,
        local_axis: Iterable[float] = (0.0, 0.0, 1.0),
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> list[float]:
        """Return a normalized tool-local axis expressed in the base frame."""

        side = ArmSide(side)
        axis = np.asarray(list(local_axis), dtype=float)
        if axis.shape != (3,) or not np.all(np.isfinite(axis)):
            raise ValueError(f"local_axis must contain three finite values, got {axis}")
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9:
            raise ValueError("local_axis must be non-zero")
        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_id = self.model.getFrameId(self.config.frame_for_side(side))
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        world_axis = self.data.oMf[frame_id].rotation @ (axis / norm)
        return (world_axis / np.linalg.norm(world_axis)).tolist()

    def q_from_arm_pos(
        self,
        arm_pos: Iterable[float],
        current_head_pos: Iterable[float] | None = None,
    ) -> np.ndarray:
        q = pin.neutral(self.model)
        arm_values = np.asarray(list(arm_pos), dtype=float)
        if arm_values.shape != (14,) or not np.all(np.isfinite(arm_values)):
            raise ValueError("arm_pos must contain 14 finite values")
        for joint_name, value in zip(ARM_POS_ORDER, arm_values):
            self._set_scalar_joint(q, joint_name, value)
        if current_head_pos is not None:
            head = np.asarray(list(current_head_pos), dtype=float)
            if head.shape != (2,) or not np.all(np.isfinite(head)):
                raise ValueError("current_head_pos must contain two finite values")
            self._set_scalar_joint(q, "head_yaw_joint", head[0])
            self._set_scalar_joint(q, "head_pitch_joint", head[1])
        return self._clip_q(q)

    def arm_pos_from_q(self, q: np.ndarray) -> list[float]:
        values = []
        for joint_name in ARM_POS_ORDER:
            jid = self.model.getJointId(joint_name)
            values.append(float(q[self.model.idx_qs[jid]]))
        return values

    def ready_arm_pos(self) -> list[float]:
        return list(self.config.ready_arm_pos())

    def clip_arm_pos(self, arm_pos: Iterable[float]) -> list[float]:
        """Clamp an arm position vector to the operational IK limits."""

        values = np.asarray(list(arm_pos), dtype=float)
        if values.shape != (14,) or not np.all(np.isfinite(values)):
            raise ValueError("arm_pos must contain 14 finite values")
        for index, (_, lower, upper) in enumerate(
            self.effective_joint_limits_for_arm_pos()
        ):
            values[index] = min(max(values[index], lower), upper)
        return values.tolist()

    def joint_limits_for_arm_pos(self) -> list[tuple[str, float, float]]:
        """Return the raw URDF limits in the SDK arm order."""

        limits = []
        for joint_name in ARM_POS_ORDER:
            jid = self.model.getJointId(joint_name)
            qidx = self.model.idx_qs[jid]
            limits.append(
                (
                    joint_name,
                    float(self.model.lowerPositionLimit[qidx]),
                    float(self.model.upperPositionLimit[qidx]),
                )
            )
        return limits

    def effective_joint_limits_for_arm_pos(self) -> list[tuple[str, float, float]]:
        """Return arm limits after applying the configured safety margin."""

        margin = self.config.joint_margin
        return [
            (name, lower + margin, upper - margin)
            for name, lower, upper in self.joint_limits_for_arm_pos()
        ]

    def _seed_q(
        self,
        current_arm_pos: Iterable[float] | None,
        current_head_pos: Iterable[float] | None,
        q_seed: np.ndarray | None,
    ) -> np.ndarray:
        if q_seed is not None:
            seed = np.asarray(q_seed, dtype=float)
            if seed.shape != (self.model.nq,) or not np.all(np.isfinite(seed)):
                raise ValueError(
                    f"q_seed must contain {self.model.nq} finite configuration values"
                )
            return self._clip_q(seed.copy())
        if current_arm_pos is None:
            current_arm_pos = self.ready_arm_pos()
        return self.q_from_arm_pos(current_arm_pos, current_head_pos)

    def _damped_least_squares(
        self,
        jacobian: np.ndarray,
        err: np.ndarray,
        active_v_idxs: list[int],
    ) -> np.ndarray:
        active_jacobian = jacobian[:, active_v_idxs]
        damping = self.config.damping
        active_velocity = active_jacobian.T @ np.linalg.solve(
            active_jacobian @ active_jacobian.T + damping * np.eye(active_jacobian.shape[0]),
            err,
        )
        velocity = np.zeros(self.model.nv)
        velocity[active_v_idxs] = active_velocity
        return velocity

    def _active_velocity_indices(self, side: ArmSide) -> list[int]:
        idxs = []
        for joint_name in self.config.active_joints_for_side(side):
            jid = self.model.getJointId(joint_name)
            idxs.append(self.model.idx_vs[jid])
        return idxs

    def _clip_q(self, q: np.ndarray) -> np.ndarray:
        # Only arm joints are actively controlled by this solver. Applying an
        # arm safety margin to passive joints would move neutral knee/waist
        # joints away from zero just because their URDF lower limit is zero.
        lower = self.model.lowerPositionLimit.copy()
        upper = self.model.upperPositionLimit.copy()
        margin = self.config.joint_margin
        for joint_name in ARM_POS_ORDER:
            jid = self.model.getJointId(joint_name)
            qidx = self.model.idx_qs[jid]
            lower[qidx] += margin
            upper[qidx] -= margin
        return np.minimum(np.maximum(q, lower), upper)

    def _validate_joint_margin(self) -> None:
        for joint_name, lower, upper in self.joint_limits_for_arm_pos():
            if lower + 2.0 * self.config.joint_margin >= upper:
                raise ValueError(
                    f"joint_margin={self.config.joint_margin} leaves no range "
                    f"for {joint_name}: [{lower}, {upper}]"
                )

    def _set_scalar_joint(self, q: np.ndarray, joint_name: str, value: float) -> None:
        if not self.model.existJointName(joint_name):
            raise ValueError(f"URDF is missing joint: {joint_name}")
        jid = self.model.getJointId(joint_name)
        if self.model.joints[jid].nq != 1:
            raise ValueError(f"Joint {joint_name} is not scalar")
        q[self.model.idx_qs[jid]] = float(value)

    def _validate_model(self) -> None:
        for joint_name in ARM_POS_ORDER:
            if not self.model.existJointName(joint_name):
                raise ValueError(f"URDF is missing expected arm joint: {joint_name}")
        for frame_name in [self.config.left_ee_frame, self.config.right_ee_frame]:
            if not self.model.existFrame(frame_name):
                raise ValueError(f"URDF is missing expected end-effector frame: {frame_name}")
        for joint_name in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS:
            jid = self.model.getJointId(joint_name)
            if self.model.joints[jid].nv != 1:
                raise ValueError(f"Expected scalar joint, got {joint_name}")
