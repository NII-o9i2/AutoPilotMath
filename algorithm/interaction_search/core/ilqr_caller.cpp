// #include "vector"
// #include "memory"
// #include "unordered_map"
// #include <cmath>
// #include "ilqr_caller.h"

// namespace InteractionSearch {

// void InteractionSearchHelper::gen_traj_by_ilqr(
//     std::shared_ptr<OSPEnv>& osp_env,
//     std::unordered_map<int, ObstacleInfo>& resoveld_obs_info,
//     Eigen::VectorXd& init_state,
//     std::vector<Eigen::VectorXd>& init_actions,
//     ILQR::ILQRParam& param,
//     std::vector<TrajectoryPoint>& res_traj) {
//   std::vector<Eigen::VectorXd> u_space;
//   double traj_cost;
//   gen_traj_by_ilqr(osp_env, resoveld_obs_info, init_state, init_actions,
//   param,
//                    res_traj, u_space, traj_cost);
// }

// void InteractionSearchHelper::gen_traj_by_ilqr(
//     std::shared_ptr<OSPEnv>& osp_env,
//     std::unordered_map<int, ObstacleInfo>& resoveld_obs_info,
//     Eigen::VectorXd& init_state,
//     std::vector<Eigen::VectorXd>& init_actions,
//     ILQR::ILQRParam& param,
//     std::vector<TrajectoryPoint>& res_traj,
//     std::vector<Eigen::VectorXd>& u_space,
//     double& traj_cost) {
//   auto interface_ptr = std::make_shared<ILQRInternalEnvInterface>();
//   interface_ptr->init(osp_env, resoveld_obs_info);

//   auto ilqr_start = std::chrono::system_clock::now();
//   ILQR::TrajectoryTreeManager ilqr_solver;
//   ilqr_solver.init(interface_ptr, init_actions, init_state, param);
//   ilqr_solver.solve();
//   auto ilqr_end = std::chrono::system_clock::now();
//   std::chrono::duration<double> diff_ilqr = ilqr_end - ilqr_start;
//   *SolverInfoLog::Instance() << "ilqr_solver time " +
//                                     std::to_string(diff_ilqr.count() *
//                                     1000.0) +
//                                     " ms ";

//   res_traj.clear();
//   u_space.clear();
//   auto solver_x_output = ilqr_solver.get_tree_x_space();
//   auto solver_u_output = ilqr_solver.get_tree_u_space();
//   for (auto& state : solver_x_output[0]) {
//     TrajectoryPoint tmp;
//     tmp.position.x = state[0];
//     tmp.position.y = state[1];
//     tmp.velocity = state[2];
//     tmp.theta = state[3];
//     tmp.acceleration = state[4];
//     tmp.yaw_rate = state[5];
//     tmp.curvature =
//         std::fabs(state[2]) > 1e-3 ? state[5] / state[2] : state[5] / 1e-3;
//     // tmp.relative_time = 0.0;
//     res_traj.emplace_back(tmp);
//   }
//   for (auto& control : solver_u_output[0]) {
//     u_space.emplace_back(control);
//   }

//   traj_cost = ilqr_solver.get_tree_l_sum();
// }

// void ILQRInternalEnvInterface::init(
//     const std::shared_ptr<OSPEnv>& refline,
//     const std::unordered_map<int, ObstacleInfo>& obs_info) {
//   refline_ = refline;

//   auto internal_obs_ptr =
//   std::make_shared<ILQRInternalObstacleMgrInterface>();
//   internal_obs_ptr->init(refline, obs_info);
//   obstacle_mgr_interface_ =
//       std::dynamic_pointer_cast<ILQRInternalObstacleMgrInterface>(
//           internal_obs_ptr);
// }

// ILQR::FuncReturn<double> ILQRInternalEnvInterface::get_lane_s(
//     const MathUtils::Point2D& pos) {
//   if (refline_ == nullptr) {
//     return ILQR::FuncReturn<double>{MathUtils::FuncStatus::FuncFailed, 0.0};
//   }
//   auto sl_point = refline_->cartesian_to_frenet(pos);
//   return ILQR::FuncReturn<double>{MathUtils::FuncStatus::FuncSucceed,
//                                   sl_point.s};
// }

// ILQR::FuncReturn<ILQR::InterfacePointInfo>
// ILQRInternalEnvInterface::get_nearest_point_info(
//     const MathUtils::Point2D& pos) {
//   ILQR::InterfacePointInfo info;
//   if (refline_ == nullptr) {
//     return ILQR::FuncReturn<ILQR::InterfacePointInfo>{
//         ILQR::FuncStatus::FuncFailed, info};
//   }
//   auto info_lane_point = refline_->get_lane_point_info(pos);
//   info.point.x = info_lane_point.point.x;
//   info.point.y = info_lane_point.point.y;
//   info.theta = info_lane_point.theta;
//   info.curvature = info_lane_point.curvature;
//   info.is_on_left = info_lane_point.is_on_left;
//   return ILQR::FuncReturn<ILQR::InterfacePointInfo>{
//       ILQR::FuncStatus::FuncSucceed, info};
// };

// void ILQRInternalObstacleMgrInterface::init(
//     const std::shared_ptr<OSPEnv>& refline,
//     const std::unordered_map<int, ObstacleInfo>& obs_info) {
//   for (auto& ob : obs_info) {
//     auto tmp = std::make_shared<ILQRInternalObstacleInterface>();
//     tmp->init(refline, ob.second);
//     insert(std::pair<int, std::shared_ptr<ILQRInternalObstacleInterface>>(
//         ob.first, tmp));
//   }
// }

// void ILQRInternalObstacleInterface::init(
//     const std::shared_ptr<OSPEnv>& refline, const ObstacleInfo& obs_info)
//     {
//   trajectory_points.clear();
//   for (auto& obstacle_point : obs_info.traj) {
//     ILQR::ILQRObstacleTrajectoryPoint pt_tmp;
//     pt_tmp.relative_time = obstacle_point.relative_time;
//     pt_tmp.length = obs_info.length;
//     pt_tmp.width = obs_info.width;
//     pt_tmp.position = obstacle_point.position;
//     pt_tmp.belief = obstacle_point.follow_belief;
//     pt_tmp.v = obstacle_point.v;
//     pt_tmp.a = obstacle_point.a;
//     pt_tmp.theta = obstacle_point.theta;
//     pt_tmp.polygon.clear();
//     for (auto& pt : obstacle_point.polygon) {
//       pt_tmp.polygon.emplace_back(pt);
//     }
//     auto sl_point = refline->cartesian_to_frenet(obstacle_point.position);
//     pt_tmp.s = sl_point.s;
//     trajectory_points.emplace_back(pt_tmp);
//   }
// }
// }
