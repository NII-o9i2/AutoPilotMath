/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Shengfa Zhu <zhushengfa@senseauto.com>
 */

#pragma once

#include "dynamic.hpp"

namespace senseAD {
namespace planning {
/*
 * state : x, y, theta, vel, omega, acc
 * ctrl: omega_rate, jerk
 */
class BicycleDynamic : public cilqr::Dynamic<6, 2> {
 public:
    explicit BicycleDynamic(const double delta_t);

    void GetDerivates(const VectorNd& state,
                      const VectorMd& ctrl,
                      MatrixNd* d_x,
                      MatrixNMd* d_u) override;

    bool Forward(const VectorNd& prev_state,
                 const VectorMd& control,
                 VectorNd* next_state) override;

 private:
    const double delta_t_;
};

}  // namespace planning
}  // namespace senseAD
