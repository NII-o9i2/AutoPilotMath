//
// Created by 冯晓彤 on 2023/5/9.
//
#include <iostream>

#include "pybind_polyfit.h"

namespace py = pybind11;
int add_c(int i, int j) {
  return i + j;
}

std::vector<double> poly_fit_eigen_python(const std::vector<math_utils::Point2D> &points){
  Eigen::VectorXd line;
  auto res = PolyFit::PolyLineFit(points,3,line,true);
  std::vector<double> python_output;
  for (auto &coef:line){
    python_output.emplace_back(coef);
  }
  return python_output;
}

std::vector<double> poly_fit_osqp_python(const std::vector<math_utils::Point2D> &points){
  std::vector<double> coef;
  auto res = PolyFit::PolyLineFitOSQP(points,3,coef);
  return coef;
}

std::vector<math_utils::Point2D> poly_fit_spline_python(const std::vector<math_utils::Point2D> &points){
  std::vector<math_utils::Point2D> smooth_points;
  PolyFit::PolyFitSplineSmooth(points, smooth_points);
  // for (const auto& pt : smooth_points) {
  //   std::cout << "smooth_points x " << pt.x << " y " << pt.y << std::endl;
  // }
  return smooth_points;
}

std::vector<math_utils::Point2D> poly_fit_spline_new_python(const std::vector<math_utils::Point2D> &points){
  std::vector<math_utils::Point2D> smooth_points;
    std::vector<double> curva_list;
  PolyFit::PolyFitSplineSmoothNew(points, smooth_points, curva_list);
  // for (const auto& pt : smooth_points) {
  //   std::cout << "smooth_points x " << pt.x << " y " << pt.y << std::endl;
  // }
  return smooth_points;
}

std::vector<::vector<math_utils::Point2D>> poly_fit_spline_new_python_piece(const std::vector<math_utils::Point2D> &points){
    std::vector<std::vector<math_utils::Point2D>> smooth_points;
    std::vector<double> curva_list;
    PolyFit::PolyFitSplineSmoothNewPiece(points, smooth_points, curva_list);
    // for (const auto& pt : smooth_points) {
    //   std::cout << "smooth_points x " << pt.x << " y " << pt.y << std::endl;
    // }
    return smooth_points;
}

std::vector<double> poly_fit_spline_new_python_get_curva(const std::vector<math_utils::Point2D> &points){
    std::vector<math_utils::Point2D> smooth_points;
    std::vector<double> curva_list;
    PolyFit::PolyFitSplineSmoothNew(points, smooth_points, curva_list);
    // for (const auto& pt : smooth_points) {
    //   std::cout << "smooth_points x " << pt.x << " y " << pt.y << std::endl;
    // }
    return curva_list;
}

// refs: https://pybind11.readthedocs.io/en/stable/classes.html
PYBIND11_MODULE(pybind_test, m) {
  py::class_<math_utils::Point2D>(m, "Point2D")
          .def(py::init<double, double>())
          .def_readwrite("x", &math_utils::Point2D::x)
          .def_readwrite("y", &math_utils::Point2D::y);

  m.doc() = "pybind11 example plugin"; // optional module docstring

  m.def("add_test", &add_c, "A function that adds two numbers");

  m.def("gen_random_points", &PolyFit::Gen_random_curv, "function generate random points");

  m.def("poly_fit_eigen_python", &poly_fit_eigen_python,"poly fit use eigen");

  m.def("poly_fit_osqp_python", &poly_fit_osqp_python,"poly fit use osqp");

  m.def("poly_fit_spline_python", &poly_fit_spline_python,"poly fit use spline");

  m.def("poly_fit_spline_new_python", &poly_fit_spline_new_python,"poly fit use spline new");

  m.def("poly_fit_spline_new_python_piece", &poly_fit_spline_new_python_piece,"poly fit use spline new piece");

  m.def("poly_fit_spline_new_python_get_curva", &poly_fit_spline_new_python_get_curva,"poly fit use spline new get curva");
}

