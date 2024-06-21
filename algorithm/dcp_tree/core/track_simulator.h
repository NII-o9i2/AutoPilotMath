#pragma once
#include "vector"
#include <cmath>
#include "unordered_map"
#include "tuple"
#include "utils.h"
#include "core/ilqr_lon_condition.h"
#include "core/ilqr_tree_solver.h"
#include "core/dcp_tree_interface.h"

namespace DCP_TREE {
class IDMLongiSimulator {
 public:
  IDMLongiSimulator() = default;
  ~IDMLongiSimulator() = default;

  void init(const std::unordered_map<DCPLaneDir,
                                     std::vector<ILQR::IDMCarInfo>,
                                     DCPLaneDirHash>& _lanes_objs,
            const ILQR::IDMCarInfo& _ego_info,
            const DCPLaneDir& _ego_lane_dir,
            const DCPActionDir& _lc_type,
            const Params& _params);

  double forward_simu(const double& dur_time);

  void sort_lane_objs_with_s(std::vector<ILQR::IDMCarInfo>& cars);

  double update_lc_belief();

  std::unordered_map<DCPLaneDir, std::vector<ILQR::IDMCarInfo>, DCPLaneDirHash>
  get_update_lanes_obs() const {
    return update_lanes_objs_;
  };

//   double get_belief() const { return lc_belief_; };

  ILQR::IDMCarInfo& get_update_ego_info() { return update_ego_info_; };

 private:
  std::vector<ILQR::IDMCarInfo> forward_simu_single_lane(
      const std::vector<ILQR::IDMCarInfo>& single_lane_objs,
      const double& dur_time,
      const bool& is_ego_next_lane);

  std::vector<ILQR::IDMCarInfo> forward_simu_single_lane_step(
      const std::vector<ILQR::IDMCarInfo>& single_lane_objs,
      const double& dur_time,
      const ILQR::IDMCarInfo& init_ego_info,
      const bool& is_ego_next_lane,
      ILQR::IDMCarInfo& updated_ego_info,
      bool& ego_updated);

 private:
  const double max_acc_threshold_ = 2.0;
  const double min_acc_threshold_ = -3.5;
  const double two_cars_space_threshold_ = 5.0;
  bool two_car_too_close_ = false;
  // input
  std::unordered_map<DCPLaneDir, std::vector<ILQR::IDMCarInfo>, DCPLaneDirHash>
      raw_lanes_objs_;
  ILQR::IDMCarInfo raw_ego_info_;
  DCPActionDir next_action_;
  DCPLaneDir ego_raw_lane_dir_;
  Params params_;

  // output
  std::unordered_map<DCPLaneDir, std::vector<ILQR::IDMCarInfo>, DCPLaneDirHash>
      update_lanes_objs_;
  ILQR::IDMCarInfo update_ego_info_;
//   double lc_belief_ = -1;
};
}  // namespace DCP_TREE