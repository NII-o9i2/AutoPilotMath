/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Shengfa Zhu <zhushengfa@senseauto.com>
 */

#pragma once

#include "cost_func.hpp"

namespace senseAD {
namespace planning {
namespace cilqr {

template <int n, int m>
class QuadraticCost : public CostFunc<n, m> {
 public:
    using VectorNd = Eigen::Matrix<double, n, 1>;
    using VectorNdRef = Eigen::Ref<const VectorNd>;
    using MatrixNd = Eigen::Matrix<double, n, n>;
    using VectorMd = Eigen::Matrix<double, m, 1>;
    using VectorMdRef = Eigen::Ref<const VectorMd>;
    using MatrixMd = Eigen::Matrix<double, m, m>;
    using MatrixNMd = Eigen::Matrix<double, n, m>;

    QuadraticCost(const MatrixNd& Q,
                  const MatrixMd& R,
                  const MatrixNMd& H,
                  const VectorNd& q,
                  const VectorMd& r,
                  double c = 0,
                  bool terminal = false)
        : n_(q.size()),
          m_(r.size()),
          isblockdiag_(H.norm() < 1e-8),
          Q_(Q),
          R_(R),
          H_(H),
          q_(q),
          r_(r),
          c_(c),
          terminal_(terminal) {
        Validate();
    }

    static QuadraticCost LQRCost(const MatrixNd& Q,
                                 const MatrixMd& R,
                                 const Eigen::VectorXd& xref,
                                 const Eigen::VectorXd& uref,
                                 bool terminal = false) {
        MatrixNMd H = MatrixNMd::Zero(n, m);
        VectorNd q = -(Q * xref);
        VectorMd r = -(R * uref);
        double c = 0.5 * xref.dot(Q * xref) + 0.5 * uref.dot(R * uref);
        return QuadraticCost(Q, R, H, q, r, c, terminal);
    }

    double Evaluate(const VectorNdRef& x, const VectorMdRef& u) override {
        return 0.5 * x.dot(Q_ * x) + x.dot(H_ * u) + 0.5 * u.dot(R_ * u) +
               q_.dot(x) + r_.dot(u) + c_;
    }

    void Gradient(const VectorNdRef& x,
                  const VectorMdRef& u,
                  Eigen::Ref<Eigen::Matrix<double, n + m, 1>> grad) override {
        VectorNd dx = Q_ * x + q_ + H_ * u;
        VectorMd du = R_ * u + r_ + H_.transpose() * x;
        grad.block(0, 0, n, 1) = dx;
        grad.block(n, 0, m, 1) = du;
    }

    void Hessian(
        const VectorNdRef& x,
        const VectorMdRef& u,
        Eigen::Ref<Eigen::Matrix<double, n + m, n + m>> hess) override {
        // dxdx = Q_, dudu = R_, dxdu = H_;
        hess.block(0, 0, n, n) = Q_;
        hess.block(n, n, m, m) = R_;
        hess.block(0, n, n, m) = H_;
        hess.block(n, 0, m, n) = H_.transpose();
    }

    const MatrixNd& GetQ() const { return Q_; }
    const MatrixMd& GetR() const { return R_; }
    const MatrixNMd& GetH() const { return H_; }
    const VectorNd& Getq() const { return q_; }
    const VectorMd& Getr() const { return r_; }
    double GetConstant() const { return c_; }
    const Eigen::LDLT<MatrixNd>& GetQfact() const { return Qfact_; }
    const Eigen::LLT<MatrixNMd>& GetRfact() const { return Rfact_; }
    bool IsBlockDiagonal() const { return isblockdiag_; }

 private:
    void Validate() {
        // ALTRO_ASSERT(Q_.rows() == n_, "Q has the wrong number of rows");
        // ALTRO_ASSERT(Q_.cols() == n_, "Q has the wrong number of columns");
        // ALTRO_ASSERT(R_.rows() == m_, "R has the wrong number of rows");
        // ALTRO_ASSERT(R_.cols() == m_, "R has the wrong number of columns");
        // ALTRO_ASSERT(H_.rows() == n_, "H has the wrong number of rows");
        // ALTRO_ASSERT(H_.cols() == m_, "H has the wrong number of columns");

        // Check symmetry of Q and R
        // ALTRO_ASSERT(Q_.isApprox(Q_.transpose()), "Q is not symmetric");
        // ALTRO_ASSERT(R_.isApprox(R_.transpose()), "R is not symmetric");

        // Check that R is positive definite
        if (!terminal_) {
            Rfact_.compute(R_);
            // ALTRO_ASSERT(Rfact_.info() == Eigen::Success,
            //              "R must be positive definite");
        }

        // Check if Q is positive semidefinite
        Qfact_.compute(Q_);
        // ALTRO_ASSERT(Qfact_.info() == Eigen::Success,
        //              "The LDLT decomposition could of Q could not be
        //              computed. "
        //              "Must be positive semi-definite");
        Eigen::Diagonal<const MatrixNd> D = Qfact_.vectorD();
        bool ispossemidef = true;
        (void)ispossemidef;  // surpress erroneous unused variable error
        for (int i = 0; i < n_; ++i) {
            if (D(i) < 0) {
                ispossemidef = false;
                break;
            }
        }
        // ALTRO_ASSERT(ispossemidef, "Q must be positive semi-definite");
    }

    int n_;
    int m_;
    bool isblockdiag_;
    MatrixNd Q_;
    MatrixMd R_;
    MatrixNMd H_;
    VectorNd q_;
    VectorMd r_;
    double c_;
    bool terminal_;

    // decompositions of Q and R
    Eigen::LDLT<MatrixNd> Qfact_;
    Eigen::LLT<MatrixMd> Rfact_;
};

}  // namespace cilqr
}  // namespace planning
}  // namespace senseAD
