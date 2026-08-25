#include "x2_grasp/ik_solver.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(_x2_ik_native, module) {
  module.doc() = "Native Pinocchio hot path for x2_arm";

  py::class_<x2_grasp::AxisIKResult>(module, "AxisIKResult")
      .def_readonly("success", &x2_grasp::AxisIKResult::success)
      .def_readonly("arm_pos", &x2_grasp::AxisIKResult::arm_pos)
      .def_readonly("final_xyz", &x2_grasp::AxisIKResult::final_xyz)
      .def_readonly("final_rpy", &x2_grasp::AxisIKResult::final_rpy)
      .def_readonly("final_axis", &x2_grasp::AxisIKResult::final_axis)
      .def_readonly("error_norm", &x2_grasp::AxisIKResult::error_norm)
      .def_readonly("position_error_norm",
                    &x2_grasp::AxisIKResult::position_error_norm)
      .def_readonly("orientation_error_norm",
                    &x2_grasp::AxisIKResult::orientation_error_norm)
      .def_readonly("iterations", &x2_grasp::AxisIKResult::iterations);

  py::class_<x2_grasp::NativeIKSolver>(module, "NativeIKSolver")
      .def(py::init<const std::string &, const std::string &,
                    const std::string &, double, int, double, double, double,
                    double>())
      .def("clip_arm_pos", &x2_grasp::NativeIKSolver::clip_arm_pos)
      .def("fk_xyz", &x2_grasp::NativeIKSolver::fk_xyz)
      .def("fk_axis", &x2_grasp::NativeIKSolver::fk_axis)
      .def("solve_axis", &x2_grasp::NativeIKSolver::solve_axis,
           py::call_guard<py::gil_scoped_release>());
}
