/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Shengfa Zhu <zhushengfa@senseauto.com>
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <string>
#include "constrain.hpp"

namespace senseAD {
namespace planning {
namespace cilqr {

/*
 * ConType = Equality or Inequality
 * */
template <int n, int m, class ConType>
class ConstrainValue : public Constrain<n, m, ConType> {
    using VectorNd = Eigen::Matrix<double, n, 1>;
    using VectorNdRef = Eigen::Ref<const VectorNd>;
    using VectorMd = Eigen::Matrix<double, m, 1>;
    using VectorMdRef = Eigen::Ref<const VectorMd>;
    using MatrixNd = Eigen::Matrix<double, n, n>;
    using MatrixMd = Eigen::Matrix<double, m, m>;
    using MatrixNMd = Eigen::Matrix<double, n, m>;

 public:
    ConstrainValue(const ConstrainPtr<n, m, ConType>& con,
                   const double penalty,
                   const double penalty_scaling)
        : Constrain<n, m, ConType>(con->Name(), con->Step()),
          con_(con),
          penalty_(penalty),
          penalty_scaling_(penalty_scaling) {
        Reset();
    }

    void Reset() {
        int output_dim = con_->OutputDimension();
        c_.setZero(output_dim);
        lambda_.setZero(output_dim);
        jac_.setZero(output_dim, n + m);
    }

    const std::string& Name() const { return con_->Name(); }

    void UpdateConstrainValue(const VectorNdRef& x, const VectorMdRef& u) {
        con_->Evaluate(x, u, c_);
    }

    void UpdateDual() { ConType::Project(lambda_ - penalty_ * c_, lambda_); }

    void UpdatePenalty() { penalty_ *= penalty_scaling_; }

    void Evaluate(
        const VectorNdRef& x,
        const VectorMdRef& u,
        Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> c) const override {
        con_->Evaluate(x, u, c);
    }

    void Jacobian(const VectorNdRef& x,
                  const VectorMdRef& u,
                  Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, n + m>> jac)
        const override {
        con_->Jacobian(x, u, jac);
    }

    double AugLag(const VectorNdRef& x, const VectorMdRef& u) const {
        Eigen::Matrix<double, Eigen::Dynamic, 1> c;
        c.setZero(OutputDimension());
        con_->Evaluate(x, u, c);
        Eigen::Matrix<double, Eigen::Dynamic, 1>
            lambda_proj;  //  projected multiplier
        lambda_proj.setZero(OutputDimension());
        ConType::Project(lambda_ - penalty_ * c, lambda_proj);
        double J = lambda_proj.squaredNorm() - lambda_.squaredNorm();
        J = J / (2 * penalty_);
        return J;
    }

    void AugLagGradient(Eigen::Ref<VectorNd> dx,
                        Eigen::Ref<VectorMd> du) const {
        Eigen::Matrix<double, Eigen::Dynamic, 1>
            lambda_proj;  //  projected multiplier
        lambda_proj.setZero(con_->OutputDimension());
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
            proj_jac;  // jacobian of prejected
        proj_jac.setZero(con_->OutputDimension(), con_->OutputDimension());

        ConType::Project(lambda_ - penalty_ * c_, lambda_proj);
        ConType::Jacobian(lambda_ - penalty_ * c_, proj_jac);
        // AD_LERROR(DEBUG) << "lambda_proj = " << lambda_proj
        //                  << ", proj_jac = " << proj_jac << ", jac = " <<
        //                  jac_;
        const int output_dim = con_->OutputDimension();
        dx = -(proj_jac * jac_.topLeftCorner(output_dim, n)).transpose() *
             lambda_proj;
        du = -(proj_jac * jac_.topRightCorner(output_dim, m)).transpose() *
             lambda_proj;
    }

    void AugLagHessian(Eigen::Ref<MatrixNd> dxdx,
                       Eigen::Ref<MatrixNMd> dxdu,
                       Eigen::Ref<MatrixMd> dudu) const {
        const int output_dim = con_->OutputDimension();
        Eigen::Matrix<double, Eigen::Dynamic, 1> lambda_proj;
        lambda_proj.setZero(output_dim);

        Eigen::Matrix<double, Eigen::Dynamic, 1> c_proj;
        c_proj.setZero(output_dim);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
            proj_jac;  // jacobian of prejected
        proj_jac.setZero(output_dim, output_dim);
        Eigen::Matrix<double, Eigen::Dynamic, n + m> jac_proj;
        jac_proj.setZero(output_dim, n + m);

        ConType::Project(lambda_ - penalty_ * c_, lambda_proj);
        ConType::Jacobian(lambda_ - penalty_ * c_, proj_jac);
        jac_proj = proj_jac * jac_;
        dxdx = penalty_ * jac_proj.topLeftCorner(output_dim, n).transpose() *
               jac_proj.topLeftCorner(output_dim, n);
        dxdu = penalty_ * jac_proj.topLeftCorner(output_dim, n).transpose() *
               jac_proj.topRightCorner(output_dim, m);
        dudu = penalty_ * jac_proj.topRightCorner(output_dim, m).transpose() *
               jac_proj.topRightCorner(output_dim, m);
    }

    void CalcExpansion(const VectorNdRef& x, const VectorMdRef& u) {
        con_->Evaluate(x, u, c_);
        con_->Jacobian(x, u, jac_);
    }

    template <int norm = Eigen::Infinity>
    double MaxViolation() const {
        Eigen::Matrix<double, Eigen::Dynamic, 1> c_proj;
        c_proj.setZero(con_->OutputDimension());
        ConType::Project(c_, c_proj);
        c_proj = c_ - c_proj;
        return c_proj.template lpNorm<norm>();
    }

    int OutputDimension() const override { return con_->OutputDimension(); }

    void Refresh(const VectorNdRef& x, const VectorMdRef& u) {
        con_->Refresh(x, u);
    }

 private:
    ConstrainPtr<n, m, ConType> con_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> c_;        // constrain value
    Eigen::Matrix<double, Eigen::Dynamic, 1> lambda_;   // lagrange multiplier
    Eigen::Matrix<double, Eigen::Dynamic, n + m> jac_;  // jacobian

    double penalty_;
    double penalty_scaling_;
};
}  // namespace cilqr
}  // namespace planning
}  // namespace senseAD
