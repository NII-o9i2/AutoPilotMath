//
// Created by gaoyuhui on 2023/11/29.
//
#include <iostream>
#include "idm.h"
#include "pybind_idm.h"
#include <vector>
namespace py = pybind11;

std::vector<IDM::ReturnTestOutput> return_test() {
  std::vector<IDM::ReturnTestOutput> a;
  IDM::ReturnTestOutput b;
  a.emplace_back(b);
  return a;
}

// refs: https://pybind11.readthedocs.io/en/stable/classes.html
namespace py = pybind11;

PYBIND11_MODULE(pybind_idm, m) {
  m.doc() = "Your module docstring";

  py::class_<IDM::ReturnTestOutput>(m, "ReturnTestOutput")
      .def(py::init<>())
      .def_readonly("a", &IDM::ReturnTestOutput::a);

  // Expose your structs
  py::class_<IDM::IDMOutput>(m, "IDMOutput")
      .def(py::init<>())
      .def_readonly("t", &IDM::IDMOutput::t)
      .def_readonly("s_ref_out", &IDM::IDMOutput::s_ref_out)
      .def_readonly("v_ref_out", &IDM::IDMOutput::v_ref_out)
      .def_readonly("a_ref_out", &IDM::IDMOutput::a_ref_out);

  py::class_<IDM::SpeedPlannerProfile>(m, "SpeedPlannerProfile")
      .def(py::init<double, double, double, double &>())
      .def_readwrite("leadercarlength",
                     &IDM::SpeedPlannerProfile::leadercarlength)
      .def_readwrite("s_leader", &IDM::SpeedPlannerProfile::s_leader)
      .def_readwrite("v_leader", &IDM::SpeedPlannerProfile::v_leader)
      .def_readwrite("exist_leader", &IDM::SpeedPlannerProfile::exist_leader);

  py::class_<IDM::Test>(m, "Test")
      .def(py::init<const double &>())
      .def_readwrite("a", &IDM::Test::a);

  py::class_<IDM::IDMParam>(m, "IDMParam")
      .def(py::init<const double, const double, const double, const double,
                    const double, const double, const double, const double,
                    const double, const double, const std::vector<double>,
                    const std::vector<double>, const std::vector<double>,
                    const std::vector<double>, const std::vector<double>,
                    const std::vector<double>, const double, const double>())
      .def_readwrite("delta_t", &IDM::IDMParam::delta_t)
      .def_readwrite("idm_cosnt_jerk", &IDM::IDMParam::idm_cosnt_jerk)
      .def_readwrite("delta", &IDM::IDMParam::delta)
      .def_readwrite("acc_delta", &IDM::IDMParam::acc_delta)
      .def_readwrite("desired_deacc", &IDM::IDMParam::desired_deacc)
      .def_readwrite("max_acc", &IDM::IDMParam::max_acc)
      .def_readwrite("time_headway", &IDM::IDMParam::time_headway)
      .def_readwrite("follow_car_delta", &IDM::IDMParam::follow_car_delta)
      .def_readwrite("min_spacing", &IDM::IDMParam::min_spacing)
      .def_readwrite("min_dacc", &IDM::IDMParam::min_dacc)
      .def_readwrite("v_std", &IDM::IDMParam::v_std)
      .def_readwrite("vel_error_std", &IDM::IDMParam::vel_error_std)
      .def_readwrite("idm_delta_v", &IDM::IDMParam::idm_delta_v)
      .def_readwrite("idm_min_spacing_list",
                     &IDM::IDMParam::idm_min_spacing_list)
      .def_readwrite("delta_v", &IDM::IDMParam::delta_v)
      .def_readwrite("s_change_rate", &IDM::IDMParam::s_change_rate)
      .def_readwrite("desired_spd", &IDM::IDMParam::desired_spd)
      .def_readwrite("change_rate", &IDM::IDMParam::change_rate);

  py::class_<IDM::EgoInfo>(m, "EgoInfo")
      .def(py::init<const double, const double, const double>())
      .def_readwrite("ego_s", &IDM::EgoInfo::ego_s)
      .def_readwrite("ego_v", &IDM::EgoInfo::ego_v)
      .def_readwrite("ego_a", &IDM::EgoInfo::ego_a);

  // Expose your functions
  m.def("calc_idm_acc", &IDM::calc_idm_acc);
  m.def("get_target_s", &IDM::get_target_s);
  m.def("linear_interpolate", &IDM::linear_interpolate);
  m.def("get_idm_output", &IDM::get_idm_output);
  m.def("return_test", &return_test);
}
