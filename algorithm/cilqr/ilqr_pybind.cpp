//
// Created by SENSETIME\fengxiaotong on 23-12-13.
//

#include "ilqr_pybind.h"
#include "ilqr_lon_condition.h"
#include "ilqr_solver.h"
#include "ilqr_tree_solver.h"
#include "lateral_longitudinal_ilqr.h"
namespace py = pybind11;

PYBIND11_MODULE(pybind_ilqr, m) {
    py::class_<ILQR::IDMParam>(m, "IDMParam")
        .def(py::init<>())
        .def_readwrite("delta", &ILQR::IDMParam::delta)
        .def_readwrite("acc_delta", &ILQR::IDMParam::acc_delta)
        .def_readwrite("max_acc", &ILQR::IDMParam::max_acc)
        .def_readwrite("time_headway", &ILQR::IDMParam::time_headway)
        .def_readwrite("follow_car_delta", &ILQR::IDMParam::follow_car_delta)
        .def_readwrite("min_dacc", &ILQR::IDMParam::min_deceleration)
        .def_readwrite("v_std", &ILQR::IDMParam::v_std)
        .def_readwrite("vel_error_std", &ILQR::IDMParam::vel_error_std)
        .def_readwrite("idm_delta_v", &ILQR::IDMParam::idm_delta_v)
        .def_readwrite("idm_min_spacing_list", &ILQR::IDMParam::idm_min_spacing_list)
        .def_readwrite("delta_v", &ILQR::IDMParam::delta_v)
        .def_readwrite("s_change_rate", &ILQR::IDMParam::s_change_rate)
        .def_readwrite("desired_spd", &ILQR::IDMParam::desired_spd);

    py::class_<ILQR::LongitudinalOperator>(m, "LongitudinalOperator")
        .def_static("calc_idm_acc", &ILQR::LongitudinalOperator::calc_idm_acc)
        .def_static("linear_interpolate", &ILQR::LongitudinalOperator::linear_interpolate)
        .def_static("calc_cah_acc", &ILQR::LongitudinalOperator::calc_cah_acc)
        .def_static("calc_acc", &ILQR::LongitudinalOperator::calc_acc);
    
    py::class_<ILQR::ILQRParam>(m, "ILQRParam")
        .def(py::init<>())
        .def_readwrite("delta_t", &ILQR::ILQRParam::delta_t)
        .def_readwrite("horizon", &ILQR::ILQRParam::horizon)
        .def_readwrite("max_iter_num", &ILQR::ILQRParam::max_iter_num)
        .def_readwrite("tol", &ILQR::ILQRParam::tol)
        .def_readwrite("kappa_thr", &ILQR::ILQRParam::kappa_thr)
        .def_readwrite("state_size", &ILQR::ILQRParam::state_size)
        .def_readwrite("action_size", &ILQR::ILQRParam::action_size);

  py::class_<ILQR::StepCondition>(m, "StepCondition")
      .def(py::init<>())
      .def_readwrite("use_exp_model_map",
                     &ILQR::StepCondition::use_exp_model_map);

  py::class_<ILQR::VehicleModelBicycle>(m, "VehicleModelBicycle")
      .def(py::init<>())
      .def("update_parameter", &ILQR::VehicleModelBicycle::update_parameter)
      .def("step_std", &ILQR::VehicleModelBicycle::step_std)
      .def("step_kappa_std", &ILQR::VehicleModelBicycle::step_kappa_std)
      .def("step_kappa_fuzzy_std",
           &ILQR::VehicleModelBicycle::step_kappa_fuzzy_std);

  py::class_<TreeILQR::VehicleModelBicyclePlus>(m, "VehicleModelBicyclePlus")
      .def(py::init<>())
      .def("update_parameter", &TreeILQR::VehicleModelBicyclePlus::update_parameter)
      .def("step_std", &TreeILQR::VehicleModelBicyclePlus::step_std);

  py::class_<TreeILQR::LatDebugInfo>(m, "LatDebugInfo")
      .def(py::init<>())
      .def_readwrite("match_point", &TreeILQR::LatDebugInfo::match_point)
      .def_readwrite("ref_omega", &TreeILQR::LatDebugInfo::ref_omega)
      .def_readwrite("lat_dis", &TreeILQR::LatDebugInfo::lat_dis_to_ref_line)
      .def_readwrite("ref_lat_acc", &TreeILQR::LatDebugInfo::ref_lat_acc);

  py::class_<TreeILQR::LonDebugInfo>(m, "LonDebugInfo")
      .def(py::init<>())
      .def_readwrite("a", &TreeILQR::LonDebugInfo::a)
      .def_readwrite("a_ref", &TreeILQR::LonDebugInfo::a_ref)
      .def_readwrite("v", &TreeILQR::LonDebugInfo::v)
      .def_readwrite("v_ref", &TreeILQR::LonDebugInfo::v_ref);

  py::class_<LateralLongitudinalMotion>(m, "LateralLongitudinalMotion")
      .def(py::init<>())
      .def("init", &LateralLongitudinalMotion::init)
      .def("get_env", &LateralLongitudinalMotion::get_env)
      .def("get_planning_origin",
           &LateralLongitudinalMotion::get_planning_origin)
      .def("execute", &LateralLongitudinalMotion::execute)
      .def("get_init_trajectory",
           &LateralLongitudinalMotion::get_init_trajectory)
      .def("get_new_trajectory", &LateralLongitudinalMotion::get_new_trajectory)
      .def("get_match_point", &LateralLongitudinalMotion::get_match_point)
      .def("get_ref_kappa", &LateralLongitudinalMotion::get_ref_kappa)
      .def("get_ref_omega", &LateralLongitudinalMotion::get_ref_omega)
      .def("get_ref_v", &LateralLongitudinalMotion::get_ref_v)
      .def("get_ref_a", &LateralLongitudinalMotion::get_ref_a)
      .def("get_iter_stat_list", &LateralLongitudinalMotion::get_iter_stat_list)
      .def("get_l_condition", &LateralLongitudinalMotion::get_l_condition)
      .def("execute_tree", &LateralLongitudinalMotion::execute_tree)
      .def("get_trajectory_tree",
           &LateralLongitudinalMotion::get_trajectory_tree)
      .def("get_init_traj_gen_failed_path_set",
           &LateralLongitudinalMotion::get_init_traj_gen_failed_path_set)
      .def("get_init_traj_gen_success",
           &LateralLongitudinalMotion::get_init_traj_gen_success)
      .def("get_lon_debug_tree",&LateralLongitudinalMotion::get_lon_debug_tree)
      .def("get_lat_debug_tree",&LateralLongitudinalMotion::get_lat_debug_tree);

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

  m.doc() = "pybind11 env simulator"; // optional module docstring
}
