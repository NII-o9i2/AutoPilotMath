//
// Created by SENSETIME\fengxiaotong on 23-12-13.
//

#include "dcp_tree_pybind.h"
namespace py = pybind11;
PYBIND11_MODULE(pybind_dcp_tree, m) {
  py::class_<DCP_TREE::DCPTreeNode>(m, "DCPTreeNode")
      .def(py::init<>())
      .def(py::init<DCP_TREE::DCPTreeNodeData,
                    const std::shared_ptr<DCP_TREE::DCPTreeNode>&>())
      .def(py::init<const DCP_TREE::DCPTreeNode&>())
      .def(py::init<DCP_TREE::DCPTreeNodeData>())
      .def("show_tree_list", &DCP_TREE::DCPTreeNode::show_tree_list)
      .def_readwrite("child_list", &DCP_TREE::DCPTreeNode::child_list)
      .def_readwrite("parent", &DCP_TREE::DCPTreeNode::parent)
      .def_readwrite("data", &DCP_TREE::DCPTreeNode::data)
      .def_readwrite("ongoing_action", &DCP_TREE::DCPTreeNode::ongoing_action)
      .def_readwrite("current_time", &DCP_TREE::DCPTreeNode::current_time)
      .def_readwrite("ego_position", &DCP_TREE::DCPTreeNode::ego_position);

  py::class_<DCP_TREE::DCPTreeNodeDebugInfo>(m, "DCPTreeNodeDebugInfo")
      .def(py::init<>())
      .def_readwrite("cost_lc", &DCP_TREE::DCPTreeNodeDebugInfo::cost_lc)
      .def_readwrite("cost_s", &DCP_TREE::DCPTreeNodeDebugInfo::cost_s)
      .def_readwrite("cost_v", &DCP_TREE::DCPTreeNodeDebugInfo::cost_v)
      .def_readwrite("s", &DCP_TREE::DCPTreeNodeDebugInfo::s)
      .def_readwrite("v", &DCP_TREE::DCPTreeNodeDebugInfo::v)
      .def_readwrite("a", &DCP_TREE::DCPTreeNodeDebugInfo::a);

  py::class_<DCP_TREE::DCPTreeNodeData>(m, "DCPTreeNodeData")
      .def(py::init<>())
      .def_readwrite("obs_pos", &DCP_TREE::DCPTreeNodeData::obs_pos)
      .def_readwrite("ego_pos", &DCP_TREE::DCPTreeNodeData::ego_pos)
      .def_readwrite("ego_lane_id", &DCP_TREE::DCPTreeNodeData::ego_lane_id)
      .def_readwrite("tree_trace", &DCP_TREE::DCPTreeNodeData::tree_trace)
      .def_readwrite("accumulated_s", &DCP_TREE::DCPTreeNodeData::accumulated_s)
      .def_readwrite("cost", &DCP_TREE::DCPTreeNodeData::cost)
      .def_readwrite("debug_info", &DCP_TREE::DCPTreeNodeData::debug_info)
      .def_readwrite("test_data", &DCP_TREE::DCPTreeNodeData::test_data);

  py::class_<DCP_TREE::DCPTree>(m, "DCPTree")
      .def(py::init<>())
      .def("set_root", &DCP_TREE::DCPTree::set_root)
      .def("update_tree", &DCP_TREE::DCPTree::update_tree)
      .def_static("generate_next_node", &DCP_TREE::DCPTree::generate_next_node)
      .def("set_IDM_params", &DCP_TREE::DCPTree::set_IDM_params)
      .def("show", &DCP_TREE::DCPTree::show)
      .def("get_all_path", &DCP_TREE::DCPTree::get_all_path);

  py::enum_<DCP_TREE::DCPActionDir>(m, "DCPActionDir")
      .value("LC_LEFT", DCP_TREE::DCPActionDir::LC_LEFT)
      .value("LK", DCP_TREE::DCPActionDir::LK)
      .value("LC_RIGHT", DCP_TREE::DCPActionDir::LC_RIGHT)
      .export_values();

  py::class_<DCP_TREE::Params>(m, "Params")
      .def(py::init<>())
      .def(py::init<const DCP_TREE::Params&>(), "Copy constructor")
      .def(py::init<const double&, const ILQR::IDMParam&, const int&>(),
           py::arg("delta_t"), py::arg("idm_params"),
           py::arg("consider_lanes_size"), "Constructor with parameters")
      .def_readwrite("delta_t", &DCP_TREE::Params::delta_t)
      .def_readwrite("max_time", &DCP_TREE::Params::max_time)
      .def_readwrite("confidence_th", &DCP_TREE::Params::confidence_th)
      .def_readwrite("idm_params", &DCP_TREE::Params::idm_params)
      .def_readwrite("consider_lanes_size",
                     &DCP_TREE::Params::consider_lanes_size);

  py::enum_<DCP_TREE::DCPLaneDir>(m, "DCPLaneDir")
      .value("ABSOLUTE_LEFT", DCP_TREE::DCPLaneDir::ABSOLUTE_LEFT)
      .value("ABSOLUTE_MIDDLE", DCP_TREE::DCPLaneDir::ABSOLUTE_MIDDLE)
      .value("ABSOLUTE_RIGHT", DCP_TREE::DCPLaneDir::ABSOLUTE_RIGHT)
      .export_values();

  py::class_<DCP_TREE::IDMLongiSimulator>(m, "IDMLongiSimulator")
      .def(py::init<>())
      .def("init", &DCP_TREE::IDMLongiSimulator::init)
      .def("forward_simu", &DCP_TREE::IDMLongiSimulator::forward_simu)
      .def("sort_lane_objs_with_s",
           &DCP_TREE::IDMLongiSimulator::sort_lane_objs_with_s)
      .def("update_lc_belief", &DCP_TREE::IDMLongiSimulator::update_lc_belief)
      .def("get_update_lanes_obs",
           &DCP_TREE::IDMLongiSimulator::get_update_lanes_obs)
    //   .def("get_belief", &DCP_TREE::IDMLongiSimulator::get_belief)
      .def("get_update_ego_info",
           &DCP_TREE::IDMLongiSimulator::get_update_ego_info,
           py::return_value_policy::reference);

  py::class_<DCP_TREE::DCPTreeInput>(m, "DCPTreeInput")
      .def(py::init<>())
      .def_readwrite("obstacle_list", &DCP_TREE::DCPTreeInput::obstacle_list)
      .def_readwrite("ego_info", &DCP_TREE::DCPTreeInput::ego_info)
      .def_readwrite("init_action_dir",
                     &DCP_TREE::DCPTreeInput::init_action_dir)
      .def_readwrite("dcp_params", &DCP_TREE::DCPTreeInput::dcp_params)
      .def("reset", &DCP_TREE::DCPTreeInput::reset);

  py::class_<DCP_TREE::DCPTreeOutput>(m, "DCPTreeOutput")
      .def(py::init<>())
      .def_readwrite("suggest_action", &DCP_TREE::DCPTreeOutput::suggest_action)
      .def_readwrite("suggest_action_relative_time",
                     &DCP_TREE::DCPTreeOutput::suggest_action_relative_time)
      .def_readwrite("suggest_action_confidence",
                     &DCP_TREE::DCPTreeOutput::suggest_action_confidence)
      .def("reset", &DCP_TREE::DCPTreeOutput::reset);

  py::class_<DCP_TREE::DebugNode>(m, "DebugNode")
      .def(py::init<>())
      .def_readwrite("time", &DCP_TREE::DebugNode::time)
      .def_readwrite("cost", &DCP_TREE::DebugNode::cost)
      .def_readwrite("action", &DCP_TREE::DebugNode::action)
      .def_readwrite("id", &DCP_TREE::DebugNode::id)
      .def("reset", &DCP_TREE::DebugNode::reset);

  py::class_<DCP_TREE::DCPTreeDebug>(m, "DCPTreeDebug")
      .def(py::init<>())
      .def_readwrite("node_debug_list",
                     &DCP_TREE::DCPTreeDebug::node_debug_list)
      .def_readwrite("node_path_debug_list",
                     &DCP_TREE::DCPTreeDebug::node_path_debug_list)
      .def("reset", &DCP_TREE::DCPTreeDebug::reset);

  py::class_<DCP_TREE::DCPTreeRunner>(m, "DCPTreeRunner")
      .def(py::init<>())
      .def("init", &DCP_TREE::DCPTreeRunner::init)
      .def("run", &DCP_TREE::DCPTreeRunner::run)
      .def("get_output", &DCP_TREE::DCPTreeRunner::get_output,
           py::return_value_policy::reference)
      .def("get_debug", &DCP_TREE::DCPTreeRunner::get_debug,
           py::return_value_policy::reference)
      .def("get_all_path", &DCP_TREE::DCPTreeRunner::get_all_path);

  py::enum_<MathUtils::FuncStatus>(m, "FuncStatus")
      .value("FuncSucceed", MathUtils::FuncStatus::FuncSucceed)
      .value("FuncFailed", MathUtils::FuncStatus::FuncFailed)
      .export_values();

  py::class_<DCP_TREE::DCPMotionTreePoint>(m, "DCPMotionTreePoint")
      .def(py::init<>())
      .def(py::init<const DCP_TREE::DCPMotionTreePoint&>(), "Copy constructor")
      .def_readwrite("point",&DCP_TREE::DCPMotionTreePoint::point)
      .def_readwrite("theta",&DCP_TREE::DCPMotionTreePoint::theta)
      .def_readwrite("v",&DCP_TREE::DCPMotionTreePoint::v)
      .def_readwrite("omega",&DCP_TREE::DCPMotionTreePoint::omega)
      .def_readwrite("speed_limit",&DCP_TREE::DCPMotionTreePoint::speed_limit)
      .def_readwrite("index",&DCP_TREE::DCPMotionTreePoint::index)
      .def_readwrite("length",&DCP_TREE::DCPMotionTreePoint::length)
      .def_readwrite("width",&DCP_TREE::DCPMotionTreePoint::width)
      .def_readwrite("polygon",&DCP_TREE::DCPMotionTreePoint::polygon)
      .def("update_polygon",&DCP_TREE::DCPMotionTreePoint::update_polygon);

  py::class_<DCP_TREE::DCPMotionTreeParameter>(m, "DCPMotionTreeParameter")
      .def(py::init<>());

  py::class_<DCP_TREE::DCPMotionTree>(m, "DCPMotionTree")
      .def(py::init<>())
      .def("init", &DCP_TREE::DCPMotionTree::init)
      .def("process",&DCP_TREE::DCPMotionTree::process)
      .def("get_init_points", &DCP_TREE::DCPMotionTree::get_init_points)
      .def("get_open_points", &DCP_TREE::DCPMotionTree::get_open_points)
      .def("get_result_points", &DCP_TREE::DCPMotionTree::get_result_points)
      .def_static("step_a_omega_dcp", DCP_TREE::DCPMotionTree::step_a_omega_dcp)
      .def_static("check_collision", DCP_TREE::DCPMotionTree::check_collision);

//  double theta;
//  double ellipse_a;
//  double ellipse_b;
//  std::vector<MathUtils::Point2D> polygon;
//  MathUtils::Point2D center_point;

  py::class_<DCP_TREE::DCPObjectInfo>(m, "DCPObjectInfo")
      .def(py::init<>())
      .def_readwrite("theta", &DCP_TREE::DCPObjectInfo::theta)
      .def_readwrite("ellipse_a", &DCP_TREE::DCPObjectInfo::ellipse_a)
      .def_readwrite("ellipse_b", &DCP_TREE::DCPObjectInfo::ellipse_b)
      .def_readwrite("polygon", &DCP_TREE::DCPObjectInfo::polygon)
      .def_readwrite("center_point", &DCP_TREE::DCPObjectInfo::center_point);

  m.doc() = "dcp_tree";  // optional module docstring
}