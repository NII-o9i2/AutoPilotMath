// #pragma once

// #include "vector"
// #include "memory"
// #include "unordered_map"
// #include <unordered_set>
// #include <algorithm>
// #include "interaction_search_common.h"

// namespace InteractionSearch {

// class InteractionSearchHelper {
//  public:
//   static void gen_traj_by_ilqr(
//       std::shared_ptr<OSPEnv> &osp_env,
//       std::unordered_map<int, ObstacleInfo> &resoveld_obs_info,
//       Eigen::VectorXd &init_state,
//       std::vector<Eigen::VectorXd> &init_actions,
//       ILQR::ILQRParam &param,
//       std::vector<TrajectoryPoint> &res_traj);

//   static void gen_traj_by_ilqr(
//       std::shared_ptr<OSPEnv> &osp_env,
//       std::unordered_map<int, ObstacleInfo> &resoveld_obs_info,
//       Eigen::VectorXd &init_state,
//       std::vector<Eigen::VectorXd> &init_actions,
//       ILQR::ILQRParam &param,
//       std::vector<TrajectoryPoint> &res_traj,
//       std::vector<Eigen::VectorXd> &u_space,
//       double &traj_cost);
// };

// class ILQRInternalEnvInterface : public ILQR::ILQREnvInterface {
//  public:
//   ILQRInternalEnvInterface() = default;
//   ~ILQRInternalEnvInterface() override = default;

//   ILQR::FuncReturn<ILQR::InterfacePointInfo> get_nearest_point_info(
//       const MathUtils::Point2D &pos) override;
//   ILQR::FuncReturn<double> get_lane_s(const MathUtils::Point2D &pos)
//   override;

//   void init(const std::shared_ptr<OSPEnv> &refline,
//             const std::unordered_map<int, ObstacleInfo> &obs_info);

//  private:
//   std::shared_ptr<OSPEnv> refline_;
// };

// class ILQRInternalObstacleInterface : public ILQR::ILQRObstacleInterface {
//  public:
//   ~ILQRInternalObstacleInterface() override = default;

//   void init(const std::shared_ptr<OSPEnv> &refline,
//             const ObstacleInfo &obs_info);

//  private:
// };

// class ILQRInternalObstacleMgrInterface : public
// ILQR::ILQRObstacleMgrInterface {
//  public:
//   ILQRInternalObstacleMgrInterface() = default;
//   ~ILQRInternalObstacleMgrInterface() = default;

//   void init(const std::shared_ptr<OSPEnv> &refline,
//             const std::unordered_map<int, ObstacleInfo> &obs_info);

//  private:
// };
// }
