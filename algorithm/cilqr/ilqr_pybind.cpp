//
// Created by SENSETIME\fengxiaotong on 23-12-13.
//

#include "ilqr_pybind.h"
#include "core/ilqr_lon_condition.h"
#include "core/ilqr_tree_solver.h"
#include "lateral_longitudinal_ilqr.h"
namespace py = pybind11;

PYBIND11_MODULE(pybind_ilqr, m) {
  py::class_<ILQR::IDMLongiProfile>(m, "IDMLongiProfile")
      .def(py::init<>())
      .def_readwrite("s", &ILQR::IDMLongiProfile::s)
      .def_readwrite("v", &ILQR::IDMLongiProfile::v)
      .def_readwrite("a", &ILQR::IDMLongiProfile::a);

  py::class_<ILQR::IDMCarInfo>(m, "IDMCarInfo")
      .def(py::init<double, double, double, double>())
      .def_readwrite("car_s", &ILQR::IDMCarInfo::car_s)
      .def_readwrite("car_v", &ILQR::IDMCarInfo::car_v)
      .def_readwrite("car_a", &ILQR::IDMCarInfo::car_a)
      .def_readwrite("car_length", &ILQR::IDMCarInfo::car_length)
      .def("print_car_info", &ILQR::IDMCarInfo::print_car_info);

  py::class_<ILQR::IDMParam>(m, "IDMParam")
      .def(py::init<>())
      .def_readwrite("delta", &ILQR::IDMParam::delta)
      .def_readwrite("acc_delta", &ILQR::IDMParam::acc_delta)
      .def_readwrite("desired_deceleration",
                     &ILQR::IDMParam::desired_deceleration)
      .def_readwrite("max_acc", &ILQR::IDMParam::max_acc)
      .def_readwrite("time_headway", &ILQR::IDMParam::time_headway)
      .def_readwrite("follow_car_delta", &ILQR::IDMParam::follow_car_delta)
      .def_readwrite("min_deceleration", &ILQR::IDMParam::min_deceleration)
      .def_readwrite("v_std", &ILQR::IDMParam::v_std)
      .def_readwrite("vel_error_std", &ILQR::IDMParam::vel_error_std)
      .def_readwrite("idm_delta_v", &ILQR::IDMParam::idm_delta_v)
      .def_readwrite("idm_min_spacing_list",
                     &ILQR::IDMParam::idm_min_spacing_list)
      .def_readwrite("delta_v", &ILQR::IDMParam::delta_v)
      .def_readwrite("s_change_rate", &ILQR::IDMParam::s_change_rate)
      .def_readwrite("desired_spd", &ILQR::IDMParam::desired_spd);

  py::class_<ILQR::LongitudinalOperator>(m, "LongitudinalOperator")
      .def_static("calc_longi_profile",
                  &ILQR::LongitudinalOperator::calc_longi_profile)
      .def_static("calc_idm_acc",
                  static_cast<std::pair<double, double> (*)(
                      const ILQR::IDMParam &, double, double, double, int)>(
                      &ILQR::LongitudinalOperator::calc_idm_acc))
      .def_static(
          "calc_idm_acc_with_limit",
          static_cast<std::pair<double, double> (*)(
              const ILQR::IDMParam &, double, double, double, double, int)>(
              &ILQR::LongitudinalOperator::calc_idm_acc))
      .def_static("linear_interpolate",
                  &ILQR::LongitudinalOperator::linear_interpolate)
      .def_static("calc_cah_acc", &ILQR::LongitudinalOperator::calc_cah_acc)
      .def_static("calc_acc", &ILQR::LongitudinalOperator::calc_acc);

  py::class_<ILQR::ILQRParam>(m, "ILQRParam")
      .def(py::init<>())
      .def_readwrite("delta_t", &ILQR::ILQRParam::delta_t)
      .def_readwrite("horizon", &ILQR::ILQRParam::horizon)
      .def_readwrite("max_iter_num", &ILQR::ILQRParam::max_iter_num)
      .def_readwrite("tol", &ILQR::ILQRParam::tol)
      .def_readwrite("state_size", &ILQR::ILQRParam::state_size)
      .def_readwrite("action_size", &ILQR::ILQRParam::action_size);

  py::class_<ILQR::StepCondition>(m, "StepCondition")
      .def(py::init<>())
      .def_readwrite("use_exp_model_map",
                     &ILQR::StepCondition::use_exp_model_map);

  //  py::class_<ILQR::VehicleModelBicycle>(m, "VehicleModelBicycle")
  //      .def(py::init<>())
  //      .def("update_parameter", &ILQR::VehicleModelBicycle::update_parameter)
  //      .def("step_std", &ILQR::VehicleModelBicycle::step_std)
  //      .def("step_kappa_std", &ILQR::VehicleModelBicycle::step_kappa_std)
  //      .def("step_kappa_fuzzy_std",
  //           &ILQR::VehicleModelBicycle::step_kappa_fuzzy_std);

  py::class_<ILQR::VehicleModelBicyclePlus>(m, "VehicleModelBicyclePlus")
      .def(py::init<>())
      .def("update_parameter", &ILQR::VehicleModelBicyclePlus::update_parameter)
      .def("step_std", &ILQR::VehicleModelBicyclePlus::step_std);

  py::class_<ILQR::LatDebugInfo>(m, "LatDebugInfo")
      .def(py::init<>())
      .def_readwrite("match_point", &ILQR::LatDebugInfo::match_point)
      .def_readwrite("ref_omega", &ILQR::LatDebugInfo::ref_omega)
      .def_readwrite("lat_dis", &ILQR::LatDebugInfo::lat_dis_to_ref_line)
      .def_readwrite("ref_lat_acc", &ILQR::LatDebugInfo::ref_lat_acc);

  py::class_<ILQR::DodgeDebugInfo>(m, "DodgeDebugInfo")
      .def(py::init<>())
      .def_readwrite("overlap_map", &ILQR::DodgeDebugInfo::overlap_map);

  py::class_<ILQR::LonDebugInfo>(m, "LonDebugInfo")
      .def(py::init<>())
      .def_readwrite("a", &ILQR::LonDebugInfo::a)
      .def_readwrite("a_ref", &ILQR::LonDebugInfo::a_ref)
      .def_readwrite("v", &ILQR::LonDebugInfo::v)
      .def_readwrite("v_ref", &ILQR::LonDebugInfo::v_ref);

  py::class_<LateralLongitudinalMotion>(m, "LateralLongitudinalMotion")
      .def(py::init<>())
      .def("init",
           py::overload_cast<const std::string &, const PlanningPoint &, bool>(
               &LateralLongitudinalMotion::init),
           "Init function with enable_dodge", py::arg("file_path"),
           py::arg("planning_point"), py::arg("enable_dodge"))
      .def("init",
           py::overload_cast<const std::string &, const PlanningPoint &>(
               &LateralLongitudinalMotion::init),
           "Init function without enable_dodge", py::arg("file_path"),
           py::arg("planning_point"))
      .def("get_env", &LateralLongitudinalMotion::get_env)
      .def("get_planning_origin",
           &LateralLongitudinalMotion::get_planning_origin)
      .def("get_init_trajectory",
           &LateralLongitudinalMotion::get_init_trajectory)
      .def("get_new_trajectory", &LateralLongitudinalMotion::get_new_trajectory)
      //      .def("get_match_point",
      //      &LateralLongitudinalMotion::get_match_point)
      //      .def("get_ref_kappa", &LateralLongitudinalMotion::get_ref_kappa)
      //      .def("get_ref_omega", &LateralLongitudinalMotion::get_ref_omega)
      //      .def("get_ref_v", &LateralLongitudinalMotion::get_ref_v)
      //      .def("get_ref_a", &LateralLongitudinalMotion::get_ref_a)
      .def("get_iter_stat_list", &LateralLongitudinalMotion::get_iter_stat_list)
      //      .def("get_l_condition",
      //      &LateralLongitudinalMotion::get_l_condition)
      .def("execute_tree", &LateralLongitudinalMotion::execute_tree)
      .def("get_trajectory_tree",
           &LateralLongitudinalMotion::get_trajectory_tree)
      .def("get_init_traj_gen_failed_path_set",
           &LateralLongitudinalMotion::get_init_traj_gen_failed_path_set)
      .def("get_init_traj_gen_success",
           &LateralLongitudinalMotion::get_init_traj_gen_success)
      .def("get_lon_debug_tree", &LateralLongitudinalMotion::get_lon_debug_tree)
      .def("get_lat_debug_tree", &LateralLongitudinalMotion::get_lat_debug_tree)
      .def("get_dodge_debug_tree",
           &LateralLongitudinalMotion::get_dodge_debug_tree);

  py::class_<PlanningPoint>(m, "PlanningPoint")
      .def(py::init<>())
      .def_readwrite("position", &PlanningPoint::position)
      .def_readwrite("theta", &PlanningPoint::theta)
      .def_readwrite("velocity", &PlanningPoint::velocity)
      .def_readwrite("acceleration", &PlanningPoint::acceleration)
      .def_readwrite("omega", &PlanningPoint::omega)
      .def_readwrite("curva", &PlanningPoint::curva)
      .def_readwrite("jerk", &PlanningPoint::jerk)
      .def_readwrite("omega_dot", &PlanningPoint::omega_dot);

  py::class_<LatLongiMotionIterationStatistic>(
      m, "LatLongiMotionIterationStatistic")
      .def(py::init<>())
      .def_readwrite("trajectory",
                     &LatLongiMotionIterationStatistic::trajectory)
      .def_readwrite("total_cost",
                     &LatLongiMotionIterationStatistic::total_cost);

  m.doc() = "pybind11 env simulator";  // optional module docstring
}
