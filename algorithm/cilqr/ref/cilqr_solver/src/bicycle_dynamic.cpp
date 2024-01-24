/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Shengfa Zhu <zhushengfa@senseauto.com>
 */

#include <cmath>
#include "bicycle_dynamic.hpp"
#include <iostream>

namespace senseAD {
namespace planning {

BicycleDynamic::BicycleDynamic(const double delta_t)
    : cilqr::Dynamic<6, 2>(), delta_t_(delta_t) {}

void BicycleDynamic::GetDerivates(const VectorNd& state,
                                  const VectorMd& ctrl,
                                  MatrixNd* d_x,
                                  MatrixNMd* d_u) {
    if (d_x == nullptr || d_u == nullptr) {
        std::cerr << "d_x or d_u are nullptr";
        return;
    }

    d_x->setZero(), d_u->setZero();

    const double t_2 = std::pow(delta_t_, 2.0);
    const double t_3 = std::pow(delta_t_, 3.0);
    const double& theta = state(2);
    const double& v = state(3);
    const double& omega = state(4);
    const double& omega_rate = ctrl(0);
    double cos_theta = std::cos(theta), sin_theta = std::sin(theta);

    /***************** ∂f/∂x ****************/
    (*d_x)(0, 0) = 1.0;
    (*d_x)(0, 2) = -sin_theta * v * delta_t_;
    (*d_x)(0, 3) = delta_t_ * cos_theta;
    (*d_x)(0, 5) = 0.5 * t_2;

    (*d_x)(1, 1) = 1.0;
    (*d_x)(1, 2) = cos_theta * v * delta_t_;
    (*d_x)(1, 3) =
        delta_t_ * sin_theta + 0.5 * omega * t_2 + 1.0 / 6.0 * omega_rate * t_3;
    (*d_x)(1, 4) = 0.5 * v * t_2;

    (*d_x)(2, 2) = 1.0;
    (*d_x)(2, 4) = delta_t_;

    (*d_x)(3, 3) = 1.0;
    (*d_x)(3, 5) = delta_t_;

    (*d_x)(4, 4) = 1.0;
    (*d_x)(5, 5) = 1.0;

    /***************** ∂f/∂u ****************/
    (*d_u)(0, 1) = 1.0 / 6.0 * t_3;
    (*d_u)(1, 0) = 1.0 / 6.0 * v * t_3;
    (*d_u)(2, 0) = 0.5 * t_2;
    (*d_u)(3, 1) = 0.5 * t_2;
    (*d_u)(4, 0) = delta_t_;
    (*d_u)(5, 1) = delta_t_;
}

bool BicycleDynamic::Forward(const VectorNd& prev_state,
                             const VectorMd& control,
                             VectorNd* next_state) {
    if (next_state == nullptr) {
        return false;
    }

    const double& x = prev_state(0);
    const double& y = prev_state(1);
    const double& theta = prev_state(2);
    const double& v = prev_state(3);
    const double& omega = prev_state(4);
    const double& acc = prev_state(5);
    const double t_2 = std::pow(delta_t_, 2.0);
    const double t_3 = std::pow(delta_t_, 3.0);

    double omega_rate = control(0);
    double jerk = control(1);

    (*next_state)(0) = x + std::cos(theta) * v * delta_t_ + 0.5 * acc * t_2 +
                       1.0 / 6.0 * jerk * t_3;
    (*next_state)(1) = y + std::sin(theta) * v * delta_t_ +
                       0.5 * v * omega * t_2 + 1.0 / 6.0 * v * omega_rate * t_3;
    (*next_state)(2) = theta + omega * delta_t_ + 0.5 * omega_rate * t_2;
    (*next_state)(3) = v + acc * delta_t_ + 0.5 * jerk * t_2;
    (*next_state)(4) = omega + omega_rate * delta_t_;
    (*next_state)(5) = acc + jerk * delta_t_;

    return true;
}

}  // namespace planning
}  // namespace senseAD
