/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * ZhongYang Zang <zangzhongyang@senseauto.com>
 * Shengfa Zhu <zhushengfa@senseauto.com>
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <memory>
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>
#include "derivative_checker.hpp"

namespace senseAD {
namespace planning {
namespace cilqr {

/*
 * h(x) <= 0.0
 * */
class Inequality {
 public:
    Inequality() = delete;

    static void Project(const Eigen::Ref<const Eigen::VectorXd>& x,
                        Eigen::Ref<Eigen::VectorXd> x_proj) {
        assert(x.size() == x_proj.size());
        // TODO(Shengfa) Vectorized?
        for (int i = 0; i < x.size(); ++i) {
            x_proj(i) = std::min(0.0, x(i));
        }
    }

    static void Jacobian(const Eigen::Ref<const Eigen::VectorXd>& x,
                         Eigen::Ref<Eigen::MatrixXd> jac) {
        assert(jac.rows() == jac.cols());
        // TODO(Shengfa) Vectorized?
        for (int i = 0; i < x.size(); ++i) {
            jac(i, i) = x(i) > 0 ? 0.0 : 1;
        }
    }

    static void hessian(const Eigen::Ref<const Eigen::VectorXd>& x,
                        const Eigen::Ref<const Eigen::VectorXd>& b,
                        Eigen::Ref<Eigen::MatrixXd> hess) {
        assert(hess.rows() == hess.cols());
        assert(x.size() == b.size());
        hess.setZero();
    }
};

/*
 * h(x) = 0
 * */
class Equality {
 public:
    Equality() = delete;

    static void Project(const Eigen::Ref<const Eigen::VectorXd>& x,
                        Eigen::Ref<Eigen::VectorXd> x_proj) {
        assert(x.size() == x_proj.size());
        x_proj.setZero();
    }

    static void Jacobian(const Eigen::Ref<const Eigen::VectorXd>& x,
                         Eigen::Ref<Eigen::MatrixXd> jac) {
        assert(jac.rows() == jac.cols());
        jac.setZero();
    }

    static void hessian(const Eigen::Ref<const Eigen::VectorXd>& x,
                        const Eigen::Ref<const Eigen::VectorXd>& b,
                        Eigen::Ref<Eigen::MatrixXd> hess) {
        assert(hess.rows() == hess.cols());
        assert(x.size() == b.size());
        hess.setZero();
    }
};

/*
 * n = num of states
 * m = num of states
 * ConType = Equality or Inequality
 * */
template <int n, int m, class ConType>
class Constrain {
 public:
    using VectorNd = Eigen::Matrix<double, n, 1>;
    using VectorNdRef = Eigen::Ref<const VectorNd>;
    using VectorMd = Eigen::Matrix<double, m, 1>;
    using VectorMdRef = Eigen::Ref<const VectorMd>;
    using MatrixNd = Eigen::Matrix<double, n, n>;
    using MatrixNMd = Eigen::Matrix<double, n, m>;

    Constrain() = delete;

    /**
     * @brief constructed by config and time step
     *
     * @param config is the Constrain config
     * @param step is the timestep
     */
    explicit Constrain(const std::string& name, const std::size_t step = 0)
        : name_(name + "-" + std::to_string(step)), step_(step) {}

    virtual ~Constrain() = default;

    const std::string& Name() const { return name_; }

    const std::size_t& Step() const { return step_; }

    virtual void Refresh(const VectorNdRef& x, const VectorMdRef& u) {}

    virtual void Debuginfo() {}

    virtual bool CheckFucDerivatives(const double tol = 1e-3) {}

    virtual int OutputDimension() const = 0;

    /**
     * @brief get state size for checking
     *
     * @return state size
     */
    constexpr std::size_t GetStateSize() const { return n; }

    bool Check(const int num = 1000,
               const double eps = 1e-6,
               const bool verbose = false) {
        Eigen::Matrix<double, n, n + m> fd_jac =
            Eigen::MatrixXd::Zero(n, n + m);
        Eigen::Matrix<double, n, n + m> jac = Eigen::MatrixXd::Zero(n, n + m);
        auto f = [&](auto z) -> VectorNd {
            Eigen::VectorXd out(this->OutputDimension());
            this->Evaluate(z.head(n), z.tail(m), out);
            return out;
        };
        for (int i = 0; i < num; ++i) {
            VectorNd state = Eigen::VectorXd::Random(n);
            VectorMd ctrl = Eigen::VectorXd::Random(m);
            MatrixNd d_x;
            MatrixNMd d_u;
            d_x.setZero();
            d_u.setZero();
            jac << d_x, d_u;
            Jacobian(state, ctrl, jac);
            Eigen::VectorXd z(n + m);
            z << state, ctrl;
            fd_jac = FiniteDiffJacobian<Eigen::Dynamic, Eigen::Dynamic>(f, z);
            if (!MatrixComparison(fd_jac, jac, eps, verbose)) return false;
        }
        if (verbose) {
            std::cout << "OK!" << std::endl;
        }
        return true;
    }

    std::string GetConstrainType() const {
        if (std::is_same<ConType, Equality>::value) {
            return "Equality";
        } else if (std::is_same<ConType, Inequality>::value) {
            return "Inequality";
        } else {
            return "Undefined";
        }
    }

    virtual void Evaluate(
        const VectorNdRef& x,
        const VectorMdRef& u,
        Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> c) const = 0;

    virtual void Jacobian(
        const VectorNdRef& x,
        const VectorMdRef& u,
        Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, n + m>> jac) const = 0;

 private:
    const std::string name_;
    const std::size_t step_;
};

template <int n, int m, class ConType>
using ConstrainPtr = std::shared_ptr<Constrain<n, m, ConType>>;

}  // namespace cilqr
}  // namespace planning
}  // namespace senseAD
