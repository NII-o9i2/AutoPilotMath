/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Shengfa Zhu <zhushengfa@senseauto.com>
 */

#pragma once

#include <limits>
#include <vector>
#include <string>
#include "constrain.hpp"

namespace senseAD {
namespace planning {
namespace cilqr {

template <int n, int m>
class StateBound : public Constrain<n, m, Inequality> {
 public:
    using VectorNd = Eigen::Matrix<double, n, 1>;
    using VectorNdRef = Eigen::Ref<const VectorNd>;
    using VectorMd = Eigen::Matrix<double, m, 1>;
    using VectorMdRef = Eigen::Ref<const VectorMd>;
    using MatrixNd = Eigen::Matrix<double, n, n>;

    explicit StateBound(const std::string& name, const std::size_t& step = 0)
        : Constrain<n, m, Inequality>(name, step),
          lower_bound_(n, -1.0 * std::numeric_limits<double>::infinity()),
          upper_bound_(n, std::numeric_limits<double>::infinity()) {}

    ~StateBound() = default;

    StateBound(const std::string& name,
               const std::size_t& step,
               const std::vector<double>& lb,
               const std::vector<double>& ub)
        : Constrain<n, m, Inequality>(name, step),
          lower_bound_(lb),
          upper_bound_(ub) {
        GetFiniteIndices(lower_bound_, &index_lower_bound_);
        GetFiniteIndices(upper_bound_, &index_upper_bound_);
    }

    void Debuginfo() override {}

    bool CheckFucDerivatives(const double tol = 1e-3) override {}

    int OutputDimension() const override {
        return index_lower_bound_.size() + index_upper_bound_.size();
    }

    void Evaluate(
        const VectorNdRef& x,
        const VectorMdRef& u,
        Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> c) const override {
        for (size_t i = 0; i < index_lower_bound_.size(); ++i) {
            size_t j = index_lower_bound_[i];
            c(i) = lower_bound_.at(j) - x(j);
        }
        int offset = index_lower_bound_.size();
        for (size_t i = 0; i < index_upper_bound_.size(); ++i) {
            size_t j = index_upper_bound_[i];
            c(i + offset) = x(j) - upper_bound_.at(j);
        }
    }

    void Jacobian(const VectorNdRef& x,
                  const VectorMdRef& u,
                  Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, n + m>> jac)
        const override {
        jac.setZero();

        for (size_t i = 0; i < index_lower_bound_.size(); ++i) {
            size_t j = index_lower_bound_[i];
            jac(i, j) = -1;
        }
        int offset = index_lower_bound_.size();
        for (size_t i = 0; i < index_upper_bound_.size(); ++i) {
            size_t j = index_upper_bound_[i];
            jac(i + offset, j) = 1;
        }
    }

 private:
    static void GetFiniteIndices(const std::vector<double>& bound,
                                 std::vector<size_t>* index) {
        index->clear();
        for (size_t i = 0; i < bound.size(); ++i) {
            if (std::fabs(bound[i]) < std::numeric_limits<double>::max()) {
                index->emplace_back(i);
            }
        }
    }

 private:
    std::vector<double> lower_bound_;
    std::vector<double> upper_bound_;
    std::vector<size_t> index_lower_bound_;
    std::vector<size_t> index_upper_bound_;
};

}  // namespace cilqr
}  // namespace planning
}  // namespace senseAD
