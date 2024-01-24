//
// Created by SENSETIME\fengxiaotong on 23-12-8.
//

#include "spline.h"
#include <Eigen/Dense>
#include <iostream>
#include <unsupported/Eigen/Splines>
#include <vector>

using namespace std;
void interpolate_points(const std::vector<MathUtils::Point2D> &input_points,
                        std::vector<MathUtils::Point2D> &output_points,
                        const int &output_points_num) {

  size_t n = input_points.size();

  // 构建矩阵A和向量B
  Eigen::MatrixXd A(n, 4);
  Eigen::VectorXd B(n);

  for (size_t i = 0; i < n; ++i) {
    A(i, 0) = 1.0;
    A(i, 1) = input_points[i].x;
    A(i, 2) = input_points[i].x * input_points[i].x;
    A(i, 3) = input_points[i].x * input_points[i].x * input_points[i].x;
    B(i) = input_points[i].y;
  }

  // 使用线性最小二乘法求解系数
  Eigen::VectorXd coefficients =
      A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

  // 输出插值结果

  double step = std::max((input_points.back().x - input_points.front().x) /
                             static_cast<double>(output_points_num),
                         0.1);
  output_points.clear();
  for (double xi = input_points[0].x; xi <= input_points[n - 1].x; xi += step) {
    double yi = coefficients[0] + coefficients[1] * xi +
                coefficients[2] * xi * xi + coefficients[3] * xi * xi * xi;
    //    std::cout << "(" << xi << ", " << yi << ")" << std::endl;
    output_points.emplace_back(xi, yi);
  }
}

MathUtils::Point2D
get_point_via_s(const std::vector<MathUtils::Point2D> &ref_points, double s) {
  double s_sum = 0.0;
  MathUtils::Point2D res;
  if (ref_points.size() == 1 && s < 1e-3) {
    res = ref_points[0];
  }
  for (int i = 1; i < ref_points.size(); i++) {
    double next_s_sum =
        s_sum + std::hypot(ref_points[i].x - ref_points[i - 1].x,
                           ref_points[i].y - ref_points[i - 1].y);
    if (s < next_s_sum) {
      double radio = (s - s_sum) / (next_s_sum - s_sum);
      res.x = ref_points[i].x + (ref_points[i].x - ref_points[i - 1].x) * radio;
      res.y = ref_points[i].y + (ref_points[i].y - ref_points[i - 1].y) * radio;
      return res;
    } else {
      s_sum = next_s_sum;
    }
  }
  return res;
};