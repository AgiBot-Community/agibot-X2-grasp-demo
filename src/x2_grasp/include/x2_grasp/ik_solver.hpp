#pragma once

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <array>
#include <string>
#include <vector>

namespace x2_grasp {

struct AxisIKResult {
  bool success{false};
  std::vector<double> arm_pos;
  std::array<double, 3> final_xyz{};
  std::array<double, 3> final_rpy{};
  std::array<double, 3> final_axis{};
  double error_norm{0.0};
  double position_error_norm{0.0};
  double orientation_error_norm{0.0};
  int iterations{0};
};

class NativeIKSolver {
 public:
  NativeIKSolver(const std::string &urdf_path, const std::string &left_ee_frame,
                 const std::string &right_ee_frame, double eps, int max_iters,
                 double dt, double damping, double max_step_norm,
                 double joint_margin);

  std::vector<double> clip_arm_pos(const std::vector<double> &arm_pos) const;
  std::array<double, 3> fk_xyz(const std::string &side,
                              const std::vector<double> &arm_pos);
  std::array<double, 3> fk_axis(const std::string &side,
                               const std::vector<double> &arm_pos,
                               const std::array<double, 3> &local_axis);
  AxisIKResult solve_axis(const std::string &side,
                         const std::array<double, 3> &target_xyz,
                         const std::array<double, 3> &target_axis,
                         const std::vector<double> &arm_pos,
                         const std::array<double, 3> &local_axis,
                         double orientation_weight, double orientation_eps);

 private:
  struct SideMetadata {
    pinocchio::FrameIndex frame_id;
    std::array<pinocchio::JointIndex, 7> joint_ids;
    std::array<Eigen::Index, 7> velocity_indices;
  };

  const SideMetadata &metadata(const std::string &side) const;
  Eigen::VectorXd q_from_arm_pos(const std::vector<double> &arm_pos) const;
  std::vector<double> arm_pos_from_q(const Eigen::VectorXd &q) const;
  void clip_q(Eigen::VectorXd &q) const;

  pinocchio::Model model_;
  pinocchio::Data data_;
  SideMetadata left_;
  SideMetadata right_;
  std::array<Eigen::Index, 14> arm_q_indices_{};
  Eigen::VectorXd clip_lower_;
  Eigen::VectorXd clip_upper_;
  double eps_;
  int max_iters_;
  double dt_;
  double damping_;
  double max_step_norm_;
};

}  // namespace x2_grasp
