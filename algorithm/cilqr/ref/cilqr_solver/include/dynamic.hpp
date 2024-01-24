/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Shengfa Zhu <zhushengfa@senseauto.com>
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <algorithm>
#include <memory>
#include "derivative_checker.hpp"

namespace senseAD {
namespace planning {
namespace cilqr {

template <int n, int m>
class Dynamic {
 public:
    using VectorNd = Eigen::Matrix<double, n, 1>;
    using VectorMd = Eigen::Matrix<double, m, 1>;
    using MatrixNd = Eigen::Matrix<double, n, n>;
    using MatrixNMd = Eigen::Matrix<double, n, m>;
    using Ptr = std::shared_ptr<Dynamic<n, m>>;

    /**
     * @brief default constructor
     */
    Dynamic() = default;

    /**
     * @brief get derivates for state and control
     *
     * @param state is the current state
     * @param ctrl is the current control
     * @param d_x is the derivate w.r.t state
     * @param d_u is the derivate w.r.t control
     */
    virtual void GetDerivates(const VectorNd& state,
                              const VectorMd& ctrl,
                              MatrixNd* d_x,
                              MatrixNMd* d_u) = 0;

    /**
     * @brief update state by dynamic
     *
     * @param prev_state
     * @param control
     * @param next_state
     *
     * @return false if control is violate the control bounds
     */
    virtual bool Forward(const VectorNd& prev_state,
                         const VectorMd& control,
                         VectorNd* next_state) = 0;
    bool Check(const int num = 1000,
               const double eps = 1e-6,
               const bool verbose = false) {
        Eigen::Matrix<double, n, n + m> fd_jac =
            Eigen::MatrixXd::Zero(n, n + m);
        Eigen::Matrix<double, n, n + m> jac = Eigen::MatrixXd::Zero(n, n + m);
        auto f = [&](auto z) -> VectorNd {
            VectorNd out;
            this->Forward(z.head(n), z.tail(m), &out);
            return out;
        };
        for (int i = 0; i < num; ++i) {
            VectorNd state = Eigen::VectorXd::Random(n);
            VectorMd ctrl = Eigen::VectorXd::Random(m);
            MatrixNd d_x;
            MatrixNMd d_u;
            GetDerivates(state, ctrl, &d_x, &d_u);
            jac << d_x, d_u;
            Eigen::VectorXd z(n + m);
            z << state, ctrl;
            fd_jac = FiniteDiffJacobian<Eigen::Dynamic, Eigen::Dynamic>(f, z);
            if (!MatrixComparison(fd_jac, jac, eps, verbose)) return false;
        }
        if (verbose) std::cout << "OK!" << std::endl;
        return true;
    }

    constexpr int GetStateSize() { return n; }

    constexpr int GetControlSize() { return m; }

 protected:
    double Clamp(const double value,
                 const double lower_bound,
                 const double upper_bound) const {
        return std::min(upper_bound, std::max(value, lower_bound));
    }
};

}  // namespace cilqr
}  // namespace planning
}  // namespace senseAD
