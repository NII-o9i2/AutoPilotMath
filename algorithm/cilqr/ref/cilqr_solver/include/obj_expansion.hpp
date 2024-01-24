/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * ZhongYang Zang <zangzhongyang@senseauto.com>
 * Shengfa Zhu <zhushengfa@senseauto.com>
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <vector>
#include <memory>
#include <random>
#include <iostream>

namespace senseAD {
namespace planning {
namespace cilqr {

template <int n, int m>
class ObjectiveExpansion {
 public:
    using VectorNd = Eigen::Matrix<double, n, 1>;
    using MatrixNd = Eigen::Matrix<double, n, n>;
    using VectorMd = Eigen::Matrix<double, m, 1>;
    using MatrixMd = Eigen::Matrix<double, m, m>;
    using Ptr = std::shared_ptr<ObjectiveExpansion<n, m>>;

    ObjectiveExpansion() = default;

    /**
     * @brief Refresh by trajectory after every iteration
     *
     * @param traj is the current solution
     */
    virtual void Refresh(const std::vector<VectorNd>& traj) = 0;

    /**
     * @brief get objective value, this function can be used as debug
     *
     * @param traj is the trajectory
     * @param state_obj is the objective value of state
     * @param ctrl_obj is the constrol objective state
     */
    virtual void GetObj(const std::vector<VectorNd>& traj,
                        const std::vector<VectorMd>& ctrls,
                        double* state_obj,
                        double* ctrl_obj) = 0;

    /**
     * @brief get objective value, this function can be used as debug
     *
     * @param state is the state
     * @param ctrl is the control
     * @param state_obj is the objective value of state
     * @param ctrl_obj is the constrol objective state
     */
    virtual void GetObj(const int step,
                        const VectorNd& state,
                        const VectorMd& ctrl,
                        double* state_obj,
                        double* ctrl_obj) const = 0;

    /**
     * @brief get objective value of endpoint state
     *
     * @param state is the endpoint state
     * @param obj[out] is the objective value
     */
    virtual void GetEndStateObj(const VectorNd& state, double* obj) const {
        // provide the default implementation
        return;
    }

    /**
     * @brief get derivates for state and control
     *
     * @param state is the current state
     * @param ctrl is the current control
     * @param d_x is the derivate w.r.t state
     * @param d_u is the derivate w.r.t control
     * @param d_xx is the second derivate w.r.t state
     * @param d_uu is the second derivate w.r.t control
     */
    virtual void GetDerivates(const int step,
                              const VectorNd& state,
                              const VectorMd& ctrl,
                              VectorNd* d_x,
                              VectorMd* d_u,
                              MatrixNd* d_xx,
                              MatrixMd* d_uu) const = 0;

    /**
     * @brief get derivates for end state
     *
     * @param state is the current state
     * @param d_x is the derivate w.r.t state
     * @param d_xx is the second derivate w.r.t state
     */
    virtual void GetDerivatesOfEndState(const VectorNd& state,
                                        VectorNd* d_x,
                                        MatrixNd* d_xx) const {
        // provide the default implementation
        return;
    }

    bool CheckDerivatives(const int num_step, const double tol = 1e-3) const {
        constexpr int test_num = 100000;

        auto GenRandomIndex = [](const int num_step) -> int {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, num_step);
            return dis(gen);
        };

        for (int test = 0; test < test_num; ++test) {
            VectorNd anchor;
            anchor << 0.0, 10.0, M_PI / 2.0, 5.0, 0.0, 0.0;
            VectorNd state0 = Eigen::MatrixXd::Random(n, 1) + anchor;
            VectorMd ctrl0 = Eigen::MatrixXd::Random(m, 1);
            int step = GenRandomIndex(num_step);
            double state0_obj = 0.0, ctrl0_obj = 0.0;
            GetObj(step, state0, ctrl0, &state0_obj, &ctrl0_obj);
            VectorNd d_x;
            MatrixNd d_xx;
            VectorMd d_u;
            MatrixMd d_uu;
            GetDerivates(step, state0, ctrl0, &d_x, &d_u, &d_xx, &d_uu);

            // check derivates w.r.t. state
            for (int k = 0; k < n; ++k) {
                constexpr double h = 1e-3;
                VectorNd identity = VectorNd::Zero();
                identity(k) = 1.0;

                Eigen::MatrixXd pre_state = state0 - h * identity;
                Eigen::MatrixXd next_state = state0 + h * identity;
                double state_prev_obj = 0.0, state_next_obj = 0.0,
                       ctrl_obj = 0.0;
                GetObj(step, pre_state, ctrl0, &state_prev_obj, &ctrl_obj);
                GetObj(step, next_state, ctrl0, &state_next_obj, &ctrl_obj);

                double deriv = (state_next_obj - state_prev_obj) / (2 * h);

                if (std::fabs(deriv - d_x(k)) > tol) {
                    std::cout << "step = " << step
                              << ", state = " << state0.transpose()
                              << ", test variable = " << k
                              << ", finit deriv = " << deriv
                              << ", analytic deriv = " << d_x[k] << std::endl;
                    return false;
                }

                double second_deriv =
                    (state_next_obj - 2.0 * state0_obj + state_prev_obj) /
                    (h * h);

                if (std::fabs(second_deriv - d_xx(k, k)) > tol) {
                    std::cout << "check second deriv failed, step = " << step
                              << ", state = " << state0.transpose()
                              << ", test variable = " << k
                              << "finit deriv = " << second_deriv
                              << ", analytic deriv = " << d_xx(k, k)
                              << std::endl;
                    return false;
                }
            }

            // check derivates w.r.t. control
            for (int k = 0; k < m; ++k) {
                constexpr double h = 1e-3;
                VectorMd identity = VectorMd::Zero();
                identity(k) = 1.0;

                VectorMd prev_ctrl = ctrl0 - h * identity;
                VectorMd next_ctrl = ctrl0 + h * identity;
                double state_obj = 0.0, ctrl_prev_obj = 0.0,
                       ctrl_next_obj = 0.0;
                GetObj(step, state0, prev_ctrl, &state_obj, &ctrl_prev_obj);
                GetObj(step, state0, next_ctrl, &state_obj, &ctrl_next_obj);

                double deriv = (ctrl_next_obj - ctrl_prev_obj) / (2 * h);

                if (std::fabs(deriv - d_u(k)) > tol) {
                    std::cout << "step = " << step
                              << ", state = " << state0.transpose()
                              << ", test variable = " << k
                              << "finit deriv = " << deriv
                              << ", analytic deriv = " << d_u[k] << std::endl;
                    return false;
                }

                double second_deriv =
                    (ctrl_next_obj - 2.0 * ctrl0_obj + ctrl_prev_obj) / (h * h);

                if (std::fabs(second_deriv - d_uu(k, k)) > tol) {
                    std::cout << "check second deriv failed, step = " << step
                              << ", state = " << state0.transpose()
                              << ", test variable = " << k
                              << "finit deriv = " << second_deriv
                              << ", analytic deriv = " << d_uu(k, k)
                              << std::endl
                              << std::endl;
                    return false;
                }
            }
        }

        return true;
    }
};

}  // namespace cilqr
}  // namespace planning
}  // namespace senseAD
