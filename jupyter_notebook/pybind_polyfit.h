//
// Created by 冯晓彤 on 2023/5/9.
//

#ifndef BENCH_TEST_PYBIND_POLYFIT_H
#define BENCH_TEST_PYBIND_POLYFIT_H
#include "pybind/include/pybind11/pybind11.h"
#include "polyfit_osqp.h"
#include "polyfit.h"
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include "utils.h"
std::vector<double> poly_fit_eigen_python(const std::vector<math_utils::Point2D> &points);
#endif //BENCH_TEST_PYBIND_POLYFIT_H
