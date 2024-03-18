//
// Created by gaoyuhui on 2023/11/29.
//

#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include "idm.h"

// std::vector<double> poly_fit_eigen_python(const
// std::vector<math_utils::Point2D> &points);

std::vector<IDM::IDMOutput> idm_python(
    IDM::EgoInfo &ego_info,
    std::vector<IDM::SpeedPlannerProfile> &speed_planner_profile,
    IDM::IDMParam &idm_params,
    std::vector<double> v_refs);