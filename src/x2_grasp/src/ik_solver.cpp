#include "x2_grasp/ik_solver.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/math/rpy.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace x2_grasp {
namespace {

const std::array<std::string, 7> kLeftJoints = {
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",   "left_elbow_joint",
    "left_wrist_yaw_joint",      "left_wrist_pitch_joint",
    "left_wrist_roll_joint"};
const std::array<std::string, 7> kRightJoints = {
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",   "right_elbow_joint",
    "right_wrist_yaw_joint",      "right_wrist_pitch_joint",
    "right_wrist_roll_joint"};

Eigen::Vector3d vector3(const std::array<double, 3> &value) {
  return Eigen::Map<const Eigen::Vector3d>(value.data());
}

std::array<double, 3> array3(const Eigen::Vector3d &value) {
  return {value.x(), value.y(), value.z()};
}

void validate_vector(const std::vector<double> &values, std::size_t size,
                     const char *name) {
  if (values.size() != size ||
      !std::all_of(values.begin(), values.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::invalid_argument(std::string(name) + " must contain " +
                                std::to_string(size) + " finite values");
  }
}

void validate_vector3(const Eigen::Vector3d &value, const char *name,
                      bool nonzero = false) {
  if (!value.allFinite()) {
    throw std::invalid_argument(std::string(name) + " must contain three finite values");
  }
  if (nonzero && value.norm() < 1e-9) {
    throw std::invalid_argument(std::string(name) + " must be non-zero");
  }
}

}  // namespace

NativeIKSolver::NativeIKSolver(
    const std::string &urdf_path, const std::string &left_ee_frame,
    const std::string &right_ee_frame, double eps, int max_iters, double dt,
    double damping, double max_step_norm, double joint_margin)
    : data_(model_),
      eps_(eps),
      max_iters_(max_iters),
      dt_(dt),
      damping_(damping),
      max_step_norm_(max_step_norm) {
  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);

  if (!std::isfinite(eps_) || eps_ <= 0.0 || max_iters_ < 1 ||
      !std::isfinite(dt_) || dt_ <= 0.0 || !std::isfinite(damping_) ||
      damping_ <= 0.0 || !std::isfinite(max_step_norm_) ||
      max_step_norm_ <= 0.0 || !std::isfinite(joint_margin) ||
      joint_margin < 0.0) {
    throw std::invalid_argument("invalid IK solver configuration");
  }

  auto populate = [&](SideMetadata &out, const auto &joint_names,
                      const std::string &frame_name, std::size_t offset) {
    if (!model_.existFrame(frame_name)) {
      throw std::invalid_argument("URDF is missing end-effector frame: " + frame_name);
    }
    out.frame_id = model_.getFrameId(frame_name);
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      if (!model_.existJointName(joint_names[i])) {
        throw std::invalid_argument("URDF is missing arm joint: " + joint_names[i]);
      }
      const auto joint_id = model_.getJointId(joint_names[i]);
      if (model_.joints[joint_id].nq() != 1 || model_.joints[joint_id].nv() != 1) {
        throw std::invalid_argument("expected scalar arm joint: " + joint_names[i]);
      }
      out.joint_ids[i] = joint_id;
      out.velocity_indices[i] = model_.idx_vs[joint_id];
      arm_q_indices_[offset + i] = model_.idx_qs[joint_id];
    }
  };
  populate(left_, kLeftJoints, left_ee_frame, 0);
  populate(right_, kRightJoints, right_ee_frame, 7);

  clip_lower_ = model_.lowerPositionLimit;
  clip_upper_ = model_.upperPositionLimit;
  for (const auto q_index : arm_q_indices_) {
    if (clip_lower_[q_index] + 2.0 * joint_margin >= clip_upper_[q_index]) {
      throw std::invalid_argument("joint margin leaves no usable arm range");
    }
    clip_lower_[q_index] += joint_margin;
    clip_upper_[q_index] -= joint_margin;
  }
}

const NativeIKSolver::SideMetadata &NativeIKSolver::metadata(
    const std::string &side) const {
  if (side == "left") return left_;
  if (side == "right") return right_;
  throw std::invalid_argument("side must be left or right");
}

Eigen::VectorXd NativeIKSolver::q_from_arm_pos(
    const std::vector<double> &arm_pos) const {
  validate_vector(arm_pos, 14, "arm_pos");
  Eigen::VectorXd q = pinocchio::neutral(model_);
  for (std::size_t i = 0; i < arm_q_indices_.size(); ++i) {
    q[arm_q_indices_[i]] = arm_pos[i];
  }
  clip_q(q);
  return q;
}

std::vector<double> NativeIKSolver::arm_pos_from_q(
    const Eigen::VectorXd &q) const {
  std::vector<double> values(14);
  for (std::size_t i = 0; i < arm_q_indices_.size(); ++i) {
    values[i] = q[arm_q_indices_[i]];
  }
  return values;
}

void NativeIKSolver::clip_q(Eigen::VectorXd &q) const {
  q = q.cwiseMax(clip_lower_).cwiseMin(clip_upper_);
}

std::vector<double> NativeIKSolver::clip_arm_pos(
    const std::vector<double> &arm_pos) const {
  return arm_pos_from_q(q_from_arm_pos(arm_pos));
}

std::array<double, 3> NativeIKSolver::fk_xyz(
    const std::string &side, const std::vector<double> &arm_pos) {
  const auto &meta = metadata(side);
  const Eigen::VectorXd q = q_from_arm_pos(arm_pos);
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return array3(data_.oMf[meta.frame_id].translation());
}

std::array<double, 3> NativeIKSolver::fk_axis(
    const std::string &side, const std::vector<double> &arm_pos,
    const std::array<double, 3> &local_axis) {
  Eigen::Vector3d axis = vector3(local_axis);
  validate_vector3(axis, "local_axis", true);
  axis.normalize();
  const auto &meta = metadata(side);
  const Eigen::VectorXd q = q_from_arm_pos(arm_pos);
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return array3((data_.oMf[meta.frame_id].rotation() * axis).normalized());
}

AxisIKResult NativeIKSolver::solve_axis(
    const std::string &side, const std::array<double, 3> &target_xyz,
    const std::array<double, 3> &target_axis,
    const std::vector<double> &arm_pos,
    const std::array<double, 3> &local_axis, double orientation_weight,
    double orientation_eps) {
  const auto &meta = metadata(side);
  const Eigen::Vector3d target = vector3(target_xyz);
  Eigen::Vector3d axis = vector3(target_axis);
  Eigen::Vector3d tool_axis = vector3(local_axis);
  validate_vector3(target, "target_xyz");
  validate_vector3(axis, "target_axis", true);
  validate_vector3(tool_axis, "local_axis", true);
  if (!std::isfinite(orientation_weight) || orientation_weight <= 0.0 ||
      !std::isfinite(orientation_eps) || orientation_eps <= 0.0) {
    throw std::invalid_argument(
        "orientation_weight and orientation_eps must be positive finite values");
  }
  axis.normalize();
  tool_axis.normalize();
  Eigen::VectorXd q = q_from_arm_pos(arm_pos);
  AxisIKResult result;
  result.error_norm = std::numeric_limits<double>::infinity();
  result.position_error_norm = std::numeric_limits<double>::infinity();
  result.orientation_error_norm = std::numeric_limits<double>::infinity();

  for (int iteration = 1; iteration <= max_iters_; ++iteration) {
    result.iterations = iteration;
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    const auto &pose = data_.oMf[meta.frame_id];
    const Eigen::Vector3d current_axis = (pose.rotation() * tool_axis).normalized();
    const Eigen::Vector3d position_error = target - pose.translation();
    const double dot = std::clamp(current_axis.dot(axis), -1.0, 1.0);
    result.orientation_error_norm = std::acos(dot);
    result.position_error_norm = position_error.norm();
    const Eigen::Vector3d axis_error = current_axis.cross(axis);
    Eigen::Matrix<double, 6, 1> error;
    error << position_error, orientation_weight * axis_error;
    result.error_norm = error.norm();
    if (result.position_error_norm < eps_ &&
        result.orientation_error_norm < orientation_eps) {
      result.success = true;
      break;
    }

    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, model_.nv);
    jacobian.setZero();
    pinocchio::computeFrameJacobian(
        model_, data_, q, meta.frame_id,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
    const Eigen::Matrix3d projection =
        Eigen::Matrix3d::Identity() - current_axis * current_axis.transpose();
    Eigen::Matrix<double, 6, 7> active_jacobian;
    for (std::size_t column = 0; column < meta.velocity_indices.size(); ++column) {
      const auto source = meta.velocity_indices[column];
      active_jacobian.block<3, 1>(0, column) = jacobian.block<3, 1>(0, source);
      active_jacobian.block<3, 1>(3, column) =
          orientation_weight * projection * jacobian.block<3, 1>(3, source);
    }
    const Eigen::Matrix<double, 6, 6> normal =
        active_jacobian * active_jacobian.transpose() +
        damping_ * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 7, 1> active_velocity =
        active_jacobian.transpose() * normal.ldlt().solve(error);
    Eigen::VectorXd velocity = Eigen::VectorXd::Zero(model_.nv);
    for (std::size_t i = 0; i < meta.velocity_indices.size(); ++i) {
      velocity[meta.velocity_indices[i]] = active_velocity[i];
    }
    Eigen::VectorXd step = velocity * dt_;
    if (step.norm() > max_step_norm_) {
      step *= max_step_norm_ / step.norm();
    }
    q = pinocchio::integrate(model_, q, step);
    clip_q(q);
  }

  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  const auto &final_pose = data_.oMf[meta.frame_id];
  result.arm_pos = arm_pos_from_q(q);
  result.final_xyz = array3(final_pose.translation());
  result.final_axis = array3((final_pose.rotation() * tool_axis).normalized());
  const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(final_pose.rotation());
  result.final_rpy = array3(rpy);
  return result;
}

}  // namespace x2_grasp
