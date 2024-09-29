//
// Created by SENSETIME\fengxiaotong on 24-8-8.
//

#include "data_process_pybind/dlp_pybind.h"
#include "data_preprocess/occ_vector_generator.h"
#include "data_preprocess/map_data_preprocess.h"
#include "data_preprocess/agent_data_preprocess.h"
namespace py = pybind11;
namespace dlp = DLP;

PYBIND11_MODULE(pybind_dlp, m) {
  py::class_<DLP::TensorInfo>(m, "TensorInfo")
      .def(py::init<>())
      .def("shape", &DLP::TensorInfo::shape)
      .def("set_shape", &DLP::TensorInfo::set_shape)
      .def("reset_shape", &DLP::TensorInfo::reset_shape)
      .def("__lshift__", [](DLP::TensorInfo &info, size_t value) {
        return info << value;
      });

  py::class_<DLP::Tensor>(m, "Tensor")
      .def("shape", &DLP::Tensor::shape);

  py::class_<DLP::TensorD1<float>>(m, "TensorD1Float")
      .def(py::init<>())
      .def(py::init<size_t>())
      .def("data", &DLP::TensorD1<float>::data);

  py::class_<DLP::TensorD1<int>>(m, "TensorD1Int")
      .def(py::init<>())
      .def(py::init<size_t>())
      .def("data", &DLP::TensorD1<int>::data);

  py::class_<DLP::TensorD1<uint8_t>>(m, "TensorD1Uint8")
      .def(py::init<>())
      .def(py::init<size_t>())
      .def("data", &DLP::TensorD1<uint8_t>::data);

  py::class_<DLP::TensorD2<float>>(m, "TensorD2Float")
      .def(py::init<>())
      .def(py::init<size_t, size_t>())
      .def("data", &DLP::TensorD2<float>::data);

  py::class_<DLP::TensorD2<bool>>(m, "TensorD2Bool")
      .def(py::init<>())
      .def(py::init<size_t, size_t>())
      .def("data", &DLP::TensorD2<bool>::data);

  py::class_<DLP::TensorD2<int>>(m, "TensorD2Int")
      .def(py::init<>())
      .def(py::init<size_t, size_t>())
      .def("data", &DLP::TensorD2<int>::data);

   py::class_<DLP::TensorD2<long>>(m, "TensorD2Long")
      .def(py::init<>())
      .def(py::init<size_t, size_t>())
      .def("data", &DLP::TensorD2<long>::data);

  py::class_<DLP::TensorD3<float>>(m, "TensorD3Float")
      .def(py::init<>())
      .def(py::init<size_t, size_t, size_t>())
      .def("data", &DLP::TensorD3<float>::data);

  py::class_<dlp::AgentDataParam>(m, "AgentDataParam")
      .def(py::init<>())
      .def_readwrite("history_num", &dlp::AgentDataParam::history_num)
      .def_readwrite("predict_num", &dlp::AgentDataParam::predict_num)
      .def_readwrite("multi_predict_num", &dlp::AgentDataParam::multi_predict_num)
      .def_readwrite("agent_max_num", &dlp::AgentDataParam::agent_max_num)
      .def_readwrite("map_polygon_num", &dlp::AgentDataParam::map_polygon_num)
      .def_readwrite("time_span", &dlp::AgentDataParam::time_span)
      .def_readwrite("a2a_radius", &dlp::AgentDataParam::a2a_radius)
      .def_readwrite("num_t2m_steps", &dlp::AgentDataParam::num_t2m_steps)
      .def_readwrite("time_span", &dlp::AgentDataParam::time_span)
      .def_readwrite("mode", &dlp::AgentDataParam::mode)
      .def_readwrite("a2m_radius", &dlp::AgentDataParam::a2m_radius)
      .def_readwrite("lat_dis_consider", &dlp::AgentDataParam::lat_dis_consider)
      .def_readwrite("lon_dis_consider", &dlp::AgentDataParam::lon_dis_consider);

  py::enum_<dlp::AgentType>(m, "AgentType")
      .value("VEHICLE", dlp::AgentType::VEHICLE)
      .value("PEDESTRIAN", dlp::AgentType::PEDESTRIAN)
      .value("MOTORCYCLIST", dlp::AgentType::MOTORCYCLIST)
      .value("CYCLIST", dlp::AgentType::CYCLIST)
      .value("BUS", dlp::AgentType::BUS)
      .value("AV_STRAIGHT", dlp::AgentType::AV_STRAIGHT)
      .value("AV_LEFT", dlp::AgentType::AV_LEFT)
      .value("AV_RIGHT", dlp::AgentType::AV_RIGHT)
      .value("OTHERS", dlp::AgentType::OTHERS)
      .export_values();

  py::enum_<dlp::AgentInfoMode>(m, "AgentInfoMode")
      .value("Train", dlp::AgentInfoMode::Train)
      .value("Predict", dlp::AgentInfoMode::Predict);

  py::class_<dlp::AgentFrameInfo>(m, "AgentFrameInfo")
      .def(py::init<>())
      .def_readwrite("rel_time_step", &dlp::AgentFrameInfo::rel_time_step)
      .def_readwrite("position_x", &dlp::AgentFrameInfo::position_x)
      .def_readwrite("position_y", &dlp::AgentFrameInfo::position_y)
      .def_readwrite("position_z", &dlp::AgentFrameInfo::position_z)
      .def_readwrite("heading", &dlp::AgentFrameInfo::heading)
      .def_readwrite("velocity", &dlp::AgentFrameInfo::velocity)
      .def_readwrite("acceleration", &dlp::AgentFrameInfo::acceleration)
      .def_readwrite("id", &dlp::AgentFrameInfo::id)
      .def_readwrite("type", &dlp::AgentFrameInfo::type)
      .def_readwrite("is_static", &dlp::AgentFrameInfo::is_static);

  py::class_<dlp::AgentInfo>(m, "AgentInfo")
      .def(py::init<dlp::AgentDataParam &>())
      .def("reset", &dlp::AgentInfo::reset)
      .def("update_and_check_exist", &dlp::AgentInfo::update_and_check_exist)
      .def("get_historical_info", &dlp::AgentInfo::get_historical_info)
      .def("create", &dlp::AgentInfo::create);

  py::class_<dlp::AgentOutputTensor>(m, "AgentOutputTensor")
      .def(py::init<dlp::AgentOutputTensor &>())
      .def("get_agent_valid_mask", &dlp::AgentOutputTensor::get_agent_valid_mask)
      .def("get_agent_type", &dlp::AgentOutputTensor::get_agent_type)
      .def("get_agent_position", &dlp::AgentOutputTensor::get_agent_position)
      .def("get_agent_heading", &dlp::AgentOutputTensor::get_agent_heading)
      .def("get_agent_velocity", &dlp::AgentOutputTensor::get_agent_velocity)
      .def("get_agent_enc_x_a", &dlp::AgentOutputTensor::get_agent_enc_x_a)
      .def("get_agent_enc_num_t", &dlp::AgentOutputTensor::get_agent_enc_num_t)
      .def("get_agent_enc_edge_index_t", &dlp::AgentOutputTensor::get_agent_enc_edge_index_t)
      .def("get_agent_enc_r_t", &dlp::AgentOutputTensor::get_agent_enc_r_t)
      .def("get_agent_enc_num_a2a", &dlp::AgentOutputTensor::get_agent_enc_num_a2a)
      .def("get_agent_enc_edge_index_a2a", &dlp::AgentOutputTensor::get_agent_enc_edge_index_a2a)
      .def("get_agent_enc_r_a2a", &dlp::AgentOutputTensor::get_agent_enc_r_a2a)
      .def("get_agent_predict_mask", &dlp::AgentOutputTensor::get_agent_predict_mask)
      .def("get_dec_num_t2m", &dlp::AgentOutputTensor::get_dec_num_t2m)
      .def("get_dec_num_t2m_multi", &dlp::AgentOutputTensor::get_dec_num_t2m_multi)
      .def("get_dec_edge_index_t2m_multi", &dlp::AgentOutputTensor::get_dec_edge_index_t2m_multi)
      .def("get_dec_num_a2m", &dlp::AgentOutputTensor::get_dec_num_a2m)
      .def("get_dec_edge_index_a2m", &dlp::AgentOutputTensor::get_dec_edge_index_a2m)
      .def("get_dec_edge_index_a2m_multi", &dlp::AgentOutputTensor::get_dec_edge_index_a2m_multi)
      .def("get_dec_r_a2m", &dlp::AgentOutputTensor::get_dec_r_a2m)
      .def("get_dec_num_a2m_multi", &dlp::AgentOutputTensor::get_dec_num_a2m_multi)
      .def("get_dec_edge_index_t2m", &dlp::AgentOutputTensor::get_dec_edge_index_t2m)
      .def("get_dec_r_t2m", &dlp::AgentOutputTensor::get_dec_r_t2m)
      .def("get_dec_num_m2m", &dlp::AgentOutputTensor::get_dec_num_m2m)
      .def("get_dec_edge_index_m2m", &dlp::AgentOutputTensor::get_dec_edge_index_m2m)
      .def("get_agent_enc_num_pl2a", &dlp::AgentOutputTensor::get_agent_enc_num_pl2a)
      .def("get_agent_enc_edge_index_pl2a", &dlp::AgentOutputTensor::get_agent_enc_edge_index_pl2a)
      .def("get_agent_enc_r_pl2a", &dlp::AgentOutputTensor::get_agent_enc_r_pl2a)
      .def("get_dec_num_pl2m_multi", &dlp::AgentOutputTensor::get_dec_num_pl2m_multi)
      .def("get_dec_edge_index_pl2m_multi", &dlp::AgentOutputTensor::get_dec_edge_index_pl2m_multi)
      .def("get_dec_r_pl2m", &dlp::AgentOutputTensor::get_dec_r_pl2m)
      .def("get_valid_num", &dlp::AgentOutputTensor::get_valid_num);

  py::class_<dlp::AgentInfoManager>(m, "AgentInfoManager")
      .def(py::init<dlp::AgentDataParam &>())
      .def("push_agent", &dlp::AgentInfoManager::push_agent)
      .def("update", &dlp::AgentInfoManager::update)
      .def("get_agent_info_map", &dlp::AgentInfoManager::get_agent_info_map)
      .def("data_preprocess", &dlp::AgentInfoManager::data_preprocess)
      .def("agent_output_tensor", &dlp::AgentInfoManager::agent_output_tensor)
      .def_readwrite("mask_id", &dlp::AgentInfoManager::mask_id);

  py::class_<dlp::Point3D>(m, "Point3D")
      .def(py::init<>())
      .def_readwrite("x", &dlp::Point3D::x)
      .def_readwrite("y", &dlp::Point3D::y)
      .def_readwrite("z", &dlp::Point3D::z);

  py::enum_<dlp::MapDataMode>(m, "MapDataMode")
      .value("M_TRAIN", dlp::MapDataMode::M_TRAIN)
      .value("M_PREDICT", dlp::MapDataMode::M_PREDICT);

  py::class_<dlp::MapDataParam>(m, "MapDataParam")
      .def(py::init<>())
      .def_readwrite("mode", &dlp::MapDataParam::mode)
      .def_readwrite("max_polygon_num", &dlp::MapDataParam::max_polygon_num)
      .def_readwrite("max_point_num", &dlp::MapDataParam::max_point_num)
      .def_readwrite("pl2pl_radius", &dlp::MapDataParam::pl2pl_radius);

  py::enum_<dlp::PL2PLType>(m, "PL2PLType")
      .value("PL2PL_NONE", dlp::PL2PLType::PL2PL_NONE)
      .value("PL2PL_PRED", dlp::PL2PLType::PL2PL_PRED)
      .value("PL2PL_SUCC", dlp::PL2PLType::PL2PL_SUCC)
      .value("PL2PL_LEFT", dlp::PL2PLType::PL2PL_LEFT)
      .value("PL2PL_RIGHT", dlp::PL2PLType::PL2PL_RIGHT);

  py::enum_<dlp::PolygonType>(m, "PolygonType")
      .value("VEHICLE_LANE", dlp::PolygonType::VEHICLE_LANE)
      .value("BIKE_LANE", dlp::PolygonType::BIKE_LANE)
      .value("BUS_LANE", dlp::PolygonType::BUS_LANE)
      .value("PEDESTRIAN_LANE", dlp::PolygonType::PEDESTRIAN_LANE)
      .value("STOP_LANE", dlp::PolygonType::STOP_LANE)
      .value("OCC", dlp::PolygonType::OCC);

  py::enum_<dlp::PointType>(m, "PointType")
      .value("DASH", dlp::PointType::DASH)
      .value("SOLID", dlp::PointType::SOLID)
      .value("NONE", dlp::PointType::NONE)
      .value("UNKNOWN", dlp::PointType::UNKNOWN)
      .value("CROSSWALK", dlp::PointType::CROSSWALK)
      .value("CENTER_LINE", dlp::PointType::CENTER_LINE)
      .value("CENTER_LINE_YELLOW_RED", dlp::PointType::CENTER_LINE_YELLOW_RED)
      .value("CENTER_LINE_GREEN", dlp::PointType::CENTER_LINE_GREEN)
      .value("STOP_LINE_VALID", dlp::PointType::STOP_LINE_VALID)
      .value("STOP_LINE_INVALID", dlp::PointType::STOP_LINE_INVALID)
      .value("OCC_LINE", dlp::PointType::OCC_LINE);

  py::enum_<dlp::PointSide>(m, "PointSide")
      .value("LEFT", dlp::PointSide::LEFT)
      .value("RIGHT", dlp::PointSide::RIGHT)
      .value("CENTER", dlp::PointSide::CENTER);

  py::enum_<dlp::PolygonBaseType>(m, "PolygonBaseType")
      .value("P_MAP_LANE_SEGMENT", dlp::PolygonBaseType::P_MAP_LANE_SEGMENT)
      .value("P_CROSSWALK", dlp::PolygonBaseType::P_CROSSWALK)
      .value("P_OCC", dlp::PolygonBaseType::P_OCC)
      .value("P_NONE", dlp::PolygonBaseType::P_NONE);

  py::class_<dlp::PolygonBase>(m, "PolygonBase")
      .def(py::init<int>())
      .def("update_tensor", &dlp::PolygonBase::update_tensor);

  py::class_<dlp::MapTensorOutput>(m, "MapTensorOutput")
      .def(py::init<dlp::MapTensorOutput &>())
      .def("get_map_polygon_position", &dlp::MapTensorOutput::get_map_polygon_position)
      .def("get_map_polygon_orientation", &dlp::MapTensorOutput::get_map_polygon_orientation)
      .def("get_map_polygon_num", &dlp::MapTensorOutput::get_map_polygon_num)
      .def("get_map_polygon_type", &dlp::MapTensorOutput::get_map_polygon_type)
      .def("get_map_polygon_is_intersection", &dlp::MapTensorOutput::get_map_polygon_is_intersection)
      .def("get_map_point_num_nodes", &dlp::MapTensorOutput::get_map_point_num_nodes)
      .def("get_map_point_position", &dlp::MapTensorOutput::get_map_point_position)
      .def("get_map_point_orientation", &dlp::MapTensorOutput::get_map_point_orientation)
      .def("get_map_point_magnitude", &dlp::MapTensorOutput::get_map_point_magnitude)
      .def("get_map_point_type", &dlp::MapTensorOutput::get_map_point_type)
      .def("get_map_point_side", &dlp::MapTensorOutput::get_map_point_side)
      .def("get_map_enc_num_pt2pl", &dlp::MapTensorOutput::get_map_enc_num_pt2pl)
      .def("get_map_enc_edge_index_pt2pl", &dlp::MapTensorOutput::get_map_enc_edge_index_pt2pl)
      .def("get_map_enc_r_pt2pl", &dlp::MapTensorOutput::get_map_enc_r_pt2pl)
      .def("get_map_enc_num_pl2pl", &dlp::MapTensorOutput::get_map_enc_num_pl2pl)
      .def("get_map_enc_edge_index_pl2pl", &dlp::MapTensorOutput::get_map_enc_edge_index_pl2pl)
      .def("get_map_enc_type_pl2pl", &dlp::MapTensorOutput::get_map_enc_type_pl2pl)
      .def("get_map_enc_r_pl2pl", &dlp::MapTensorOutput::get_map_enc_r_pl2pl);

  py::class_<dlp::MapInfoManager, std::shared_ptr<dlp::MapInfoManager>>(m, "MapInfoManager")
      .def(py::init<dlp::MapDataParam &>())
      .def("reset", &dlp::MapInfoManager::reset)
      .def("push_polygon", &dlp::MapInfoManager::push_polygon)
      .def("push_pl2pl_edge", &dlp::MapInfoManager::push_pl2pl_edge)
      .def("update_tensor", &dlp::MapInfoManager::update_tensor)
      .def("get_map_tensor_out", &dlp::MapInfoManager::get_map_tensor_out)
      .def("get_polygon_list", &dlp::MapInfoManager::get_polygon_list);

  py::class_<dlp::OccPartitionInfo>(m, "OccPartitionInfo")
      .def(py::init<>())
      .def_readwrite("num_partition", &dlp::OccPartitionInfo::num_partition)
      .def_readwrite("ranges", &dlp::OccPartitionInfo::ranges)
      .def_readwrite("steps", &dlp::OccPartitionInfo::steps)
      .def_readwrite("num_voxels_per_range", &dlp::OccPartitionInfo::num_voxels_per_range);

  py::class_<dlp::OccVoxel>(m, "OccVoxel")
      .def(py::init<>())
      .def_readwrite("i", &dlp::OccVoxel::i)
      .def_readwrite("j", &dlp::OccVoxel::j)
      .def_readwrite("v", &dlp::OccVoxel::v);

  py::class_<dlp::RawOccInfo>(m, "RawOccInfo")
      .def(py::init<>())
      .def_readwrite("x_partition", &dlp::RawOccInfo::x_partition)
      .def_readwrite("y_partition", &dlp::RawOccInfo::y_partition)
      .def_readwrite("voxel_list", &dlp::RawOccInfo::voxel_list);

  py::class_<dlp::OccFreeSpaceVector>(m, "OccFreeSpaceVector")
      .def(py::init<>())
      .def_readwrite("free_space_edge", &dlp::OccFreeSpaceVector::free_space_edge);

  py::class_<dlp::OccVectorGenerator>(m, "OccVectorGenerator")
      .def(py::init<>())
      .def("build_occ_vector", &dlp::OccVectorGenerator::build_occ_vector);

  m.doc() = "dlp data transformer & data preprocess";  // optional module docstring
}
