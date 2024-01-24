/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Zhongyang Zang <zangzhongyang@senseauto.com>
 */

#pragma once
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>

namespace senseAD {
namespace planning {
namespace cilqr {
using VectorXdRef = Eigen::Ref<const Eigen::VectorXd>;

template <class Func>
struct ScalarToVec {
    using Vector1d = Eigen::Matrix<double, 1, 1>;
    Vector1d operator()(const Eigen::VectorXd &x) const {
        Vector1d y;
        y << f(x);
        return y;
    }
    Func f;
};

template <class MatA, class MatB>
bool MatrixComparison(const MatA &expected,
                      const MatB &actual,
                      const double eps,
                      const bool verbose) {
    // Compare
    double err = (expected - actual).norm();

    // Print results
    if (err > eps) {
        if (verbose) {
            std::cout << "Calculated: " << actual << std::endl;
            std::cout << "Finite Diff: " << expected << std::endl;
        }
        std::cout << "Error: " << err << std::endl;
    }
    return err < eps;
}

template <int nrows, int ncols, class Func>
Eigen::Matrix<double, nrows, ncols> FiniteDiffJacobian(
    const Func &f,
    const Eigen::Ref<const Eigen::Matrix<double, ncols, 1>> &x,
    const double eps = 1e-6,
    const bool central = false) {
    // Evaluate the function and get output size
    Eigen::Matrix<double, nrows, 1> y = f(x);
    const int n = x.rows();
    const int m = y.rows();
    Eigen::Matrix<double, nrows, ncols> jac =
        Eigen::Matrix<double, nrows, ncols>::Zero(m, n);

    // Create pertubation vector
    Eigen::Matrix<double, ncols, 1> e =
        Eigen::Matrix<double, ncols, 1>::Zero(n);

    // Loop over columns
    e(0) = eps;
    for (int i = 0; i < n; ++i) {
        double step = eps;
        if (central) {
            y = f(x - e);
            step = 2 * eps;
        }
        jac.col(i) = (f(x + e) - y) / step;
        if (i < n - 1) {
            e(i + 1) = e(i);
            e(i) = 0;
        }
    }
    return jac;
}

template <class Func>
Eigen::MatrixXd FiniteDiffJacobian(const Func &f,
                                   const VectorXdRef &x,
                                   const double eps = 1e-6,
                                   const bool central = false) {
    return FiniteDiffJacobian<-1, -1, Func>(f, x, eps, central);
}

template <int ncols, class Func>
Eigen::Matrix<double, ncols, 1> FiniteDiffGradient(
    const Func &f,
    const Eigen::Matrix<double, ncols, 1> &x,
    const double eps = 1e-6,
    const bool central = false) {
    ScalarToVec<Func> f2 = {f};
    return FiniteDiffJacobian<1, ncols, ScalarToVec<Func>>(f2, x, eps, central)
        .transpose();
}

}  // namespace cilqr
}  // namespace planning
}  // namespace senseAD
