/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Shengfa Zhu <zhushengfa@senseauto.com>
 */

#include <eigen3/Eigen/Core>
#include <memory>
#include <random>
#include "gtest/gtest.h"
#include "glog/logging.h"
#include "proto/cilqr_debug_info.pb.h"
#include "bicycle_dynamic.hpp"
#include "quadratic_cost.hpp"
#include "constrain.hpp"
#include "cilqr_problem.hpp"
#include "control_bound.hpp"

#ifndef GFLAGS_GFLAGS_H_
namespace gflags = google;
#endif  // GFLAGS_GFLAGS_H_

namespace pp = senseAD::planning;
typedef Eigen::Matrix<double, 6, 1> VectorNd;
typedef Eigen::Matrix<double, 2, 1> VectorMd;
typedef Eigen::Matrix<double, 6, 6> MatrixNd;
typedef Eigen::Matrix<double, 2, 2> MatrixMd;

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    testing::InitGoogleTest(&argc, argv);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return RUN_ALL_TESTS();
}

void GenInitTraj(const int num_step,
                 const double delta_t,
                 const pp::BicycleDynamic::Ptr& dynamic,
                 const VectorNd& init_state,
                 std::vector<VectorNd>* init_traj,
                 std::vector<VectorNd>* tgt_traj,
                 std::vector<VectorMd>* ctrls) {
    std::vector<VectorMd> init_ctrls(num_step, VectorMd::Zero());
    init_traj->reserve(num_step + 1);
    init_traj->emplace_back(init_state);
    VectorNd prev_state = init_state;
    for (int i = 0; i < num_step; ++i) {
        VectorMd ctrl = init_ctrls[i];
        VectorNd next_state;
        dynamic->Forward(prev_state, ctrl, &next_state);
        init_traj->emplace_back(next_state);
        ctrls->emplace_back(ctrl);
        prev_state = next_state;
    }

    // generate random value between [0, 0.1]
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0);
    for (const auto& tjp : *init_traj) {
        auto tgt_tjp = tjp;
        double random_number = dis(gen);
        tgt_tjp[0] += random_number;
        tgt_tjp[1] += random_number;
        tgt_traj->emplace_back(tgt_tjp);
    }
}

TEST(CILQR, joint_optimizer) {
    pp::cilqr::CilqrDebugInfo debug_info;
    double delta_t = 0.2;  // unit, s
    int num_step = 40;
    pp::BicycleDynamic::Ptr dynamic =
        std::make_shared<pp::BicycleDynamic>(pp::BicycleDynamic(delta_t));

    VectorNd init_state;
    init_state << 0.0, 0.0, 0.0, 4.0, 0.0, 0.0;
    std::vector<VectorNd> init_traj;
    std::vector<VectorNd> tgt_traj;
    std::vector<VectorMd> init_ctrls;

    GenInitTraj(num_step, delta_t, dynamic, init_state, &init_traj, &tgt_traj,
                &init_ctrls);

    pp::cilqr::CilqrSolverOpt opt{.verbose = true,
                                  .max_line_search_ite = 10,
                                  .line_search_init_step = 1.0,
                                  .line_search_step_decay = 0.5,
                                  .rho = 0.0,
                                  .max_reg = 1.0e8,
                                  .min_reg = 1.0e-8,
                                  .reg_scale = 1.5,
                                  .max_inner_loop_ite = 10,
                                  .max_outer_loop_ite = 10};
    pp::cilqr::CilqrProblem<6, 2> problem(opt, num_step);
    problem.Update(init_state, init_traj, init_ctrls, dynamic);

    // Q, R of LQR cost
    MatrixNd Q = MatrixNd::Zero();
    MatrixMd R = MatrixMd::Zero();
    Q.diagonal().setConstant(0.00001);
    R.diagonal()(0) = 0.0002;
    R.diagonal()(1) = 0.001;

    VectorMd u_ref = VectorMd::Zero();
    for (int i = 0; i < tgt_traj.size(); ++i) {
        auto tgt_state = tgt_traj.at(i);
        problem.SetCostFunc(
            std::make_shared<pp::cilqr::QuadraticCost<6, 2>>(
                pp::cilqr::QuadraticCost<6, 2>::LQRCost(Q, R, tgt_state, u_ref)),
            i);
    }

    // add control bound
    double jerk_bnd = 4.0;
    double omega_rate_bnd = 0.1;
    std::vector<double> lb{-1.0 * omega_rate_bnd, -1.0 * jerk_bnd};
    std::vector<double> ub{omega_rate_bnd, jerk_bnd};
    for (int i = 0; i < num_step; ++i) {
        std::shared_ptr<pp::cilqr::ControlBound<6, 2>> control_bound(
            std::make_shared<pp::cilqr::ControlBound<6, 2>>(i, lb, ub));
        problem.SetInequalityConstrain(control_bound, i);
    }

    std::vector<VectorNd> solution;
    std::vector<VectorMd> ctrls;

    pp::cilqr::CilqrSolveStatus status = problem.Solve(&solution, &ctrls);

    for (std::size_t idx = 0; idx < ctrls.size(); ++idx) {
        std::cout << "idx = " << idx << ", x = " << solution[idx].transpose()
                  << ", ctrls = " << ctrls[idx].transpose() << std::endl;
    }
    if (status != pp::cilqr::CilqrSolveStatus::SUCCESS &&
        status != pp::cilqr::CilqrSolveStatus::LOCAL_OPTIMUM) {
        std::cout << "CiLQR solve problem failed!" << std::endl;
        EXPECT_TRUE(false);
    }
}
