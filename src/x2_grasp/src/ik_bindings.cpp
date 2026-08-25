#include "x2_grasp/ik_solver.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(_x2_ik_native, module) {
  module.doc() = "Native Pinocchio hot path for x2_arm";

  py::class_<x2_grasp::NativeIKResult>(module, "NativeIKResult")
      .def_readonly("success", &x2_grasp::NativeIKResult::success)
      .def_readonly("arm_pos", &x2_grasp::NativeIKResult::arm_pos)
      .def_readonly("final_xyz", &x2_grasp::NativeIKResult::final_xyz)
      .def_readonly("final_rpy", &x2_grasp::NativeIKResult::final_rpy)
      .def_readonly("final_axis", &x2_grasp::NativeIKResult::final_axis)
      .def_readonly("error_norm", &x2_grasp::NativeIKResult::error_norm)
      .def_readonly("position_error_norm",
                    &x2_grasp::NativeIKResult::position_error_norm)
      .def_readonly("orientation_error_norm",
                    &x2_grasp::NativeIKResult::orientation_error_norm)
      .def_readonly("iterations", &x2_grasp::NativeIKResult::iterations);

  py::class_<x2_grasp::NativeIKSolver>(module, "NativeIKSolver")
      .def(py::init<const std::string &, const std::string &,
                    const std::string &, double, int, double, double, double,
                    double>())
      .def("clip_arm_pos", &x2_grasp::NativeIKSolver::clip_arm_pos)
      .def("configuration_from_arm_pos",
           &x2_grasp::NativeIKSolver::configuration_from_arm_pos)
      .def("arm_q_indices", &x2_grasp::NativeIKSolver::arm_q_indices)
      .def("joint_limits", &x2_grasp::NativeIKSolver::joint_limits)
      .def("effective_joint_limits",
           &x2_grasp::NativeIKSolver::effective_joint_limits)
      .def("fk_xyz", &x2_grasp::NativeIKSolver::fk_xyz)
      .def("fk_rpy", &x2_grasp::NativeIKSolver::fk_rpy)
      .def("fk_axis", &x2_grasp::NativeIKSolver::fk_axis)
      .def("solve_position", &x2_grasp::NativeIKSolver::solve_position,
           py::call_guard<py::gil_scoped_release>())
      .def("solve_pose", &x2_grasp::NativeIKSolver::solve_pose,
           py::call_guard<py::gil_scoped_release>())
      .def("solve_axis", &x2_grasp::NativeIKSolver::solve_axis,
           py::call_guard<py::gil_scoped_release>());
}
