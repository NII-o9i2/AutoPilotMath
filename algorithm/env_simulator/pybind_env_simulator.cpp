//
// Created by SENSETIME\fengxiaotong on 23-12-8.
//

#include "pybind_env_simulator.h"
#include "env_simulator.h"
#include "lane.h"
#include "utils.h"
namespace py = pybind11;

PYBIND11_MODULE(pybind_env_simulator, m) {
  py::class_<EnvSim::Lane>(m, "Lane")
      //    .def(py::init<>())
      .def("get_center_points", &EnvSim::Lane::get_center_points);

  py::class_<EnvSim::EnvSimulator>(m, "EnvSimulator")
      .def(py::init<>())
      .def("update_case_data", &EnvSim::EnvSimulator::update_case_data)
      .def("get_all_center_points",
           &EnvSim::EnvSimulator::get_all_lanes_center_points)
      .def("get_all_obstacle", &EnvSim::EnvSimulator::get_all_obstacle)
      .def("get_all_road_edge_points",
           &EnvSim::EnvSimulator::get_all_road_edge_points)
      .def("get_freespace_points_by_roi",
           &EnvSim::EnvSimulator::get_freespace_points_by_roi)
      .def("get_freespace_ellipse_param",
           &EnvSim::EnvSimulator::get_freespace_ellipse_param)
      .def("get_ego_car", &EnvSim::EnvSimulator::get_ego_car)
      .def("get_freespace_right_points_by_ttc",
           &EnvSim::EnvSimulator::get_freespace_right_points_by_ttc)
      .def("get_freespace_left_points_by_ttc",
           &EnvSim::EnvSimulator::get_freespace_left_points_by_ttc)
      .def("get_right_border_use_ellipse_model",
           &EnvSim::EnvSimulator::get_right_border_use_ellipse_model)
      .def("get_left_border_use_ellipse_model",
           &EnvSim::EnvSimulator::get_left_border_use_ellipse_model)
      .def("get_predict_ego_position_at_time",
           &EnvSim::EnvSimulator::get_predict_ego_position_at_time);

  py::class_<EnvSim::EllipseParam>(m, "EllipseParam")
      .def(py::init<>())
      .def_readwrite("a", &EnvSim::EllipseParam::a)
      .def_readwrite("b", &EnvSim::EllipseParam::b)
      .def_readwrite("center_point", &EnvSim::EllipseParam::center_point);

  py::class_<EnvSim::VehicleInfo>(m, "VehicleInfo")
      .def(py::init<>())
      .def_readwrite("vehicle_speed", &EnvSim::VehicleInfo::vehicle_speed)
      .def_readwrite("theta", &EnvSim::VehicleInfo::theta)
      .def_readwrite("vehicle_position", &EnvSim::VehicleInfo::vehicle_position)
      .def_readwrite("length", &EnvSim::VehicleInfo::length)
      .def_readwrite("width", &EnvSim::VehicleInfo::width);

  py::class_<MathUtils::Point2D>(m, "Point2D")
      .def(py::init<>())
      .def(py::init<double, double>())
      .def_readwrite("x", &MathUtils::Point2D::x)
      .def_readwrite("y", &MathUtils::Point2D::y);

  py::class_<EnvSim::RawTrajectoryPoint>(m, "RawTrajectoryPoint")
      .def(py::init<int, double, double>())
      .def_readwrite("lane_id", &EnvSim::RawTrajectoryPoint::lane_id)
      .def_readwrite("s", &EnvSim::RawTrajectoryPoint::s)
      .def_readwrite("l", &EnvSim::RawTrajectoryPoint::l);

  py::class_<EnvSim::TrajectoryPoint>(m, "TrajectoryPoint")
      .def(py::init<>())
      .def_readwrite("position", &EnvSim::TrajectoryPoint::position)
      .def_readwrite("theta", &EnvSim::TrajectoryPoint::theta);

  py::class_<EnvSim::Obstacle>(m, "Obstacle")
      .def(py::init<int, double, double, double, std::vector<double>,
                    std::vector<double>,
                    std::vector<EnvSim::RawTrajectoryPoint>>())
      .def_readwrite("trajectory_points", &EnvSim::Obstacle::trajectory_points)
      .def("get_length", &EnvSim::Obstacle::get_length)
      .def("get_width", &EnvSim::Obstacle::get_width);

  py::class_<EnvSim::PlanningPoint>(m, "PlanningPoint")
      .def(py::init<>())
      .def_readwrite("position", &EnvSim::PlanningPoint::position)
      .def_readwrite("theta", &EnvSim::PlanningPoint::theta)
      .def_readwrite("velocity", &EnvSim::PlanningPoint::velocity)
      .def_readwrite("acceleration", &EnvSim::PlanningPoint::acceleration)
      .def_readwrite("omega", &EnvSim::PlanningPoint::omega)
      .def_readwrite("curva", &EnvSim::PlanningPoint::curva);

  m.doc() = "pybind11 env simulator"; // optional module docstring
}
