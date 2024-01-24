#pragma once

#include <deque>
#include <memory>
#include <vector>

#include "spline_generator.hpp"
#include "common/utils.h"

namespace PolyFit {

class SplineReferenceSmootherNew {
public:
  static bool SplineSmooth(const std::vector<math_utils::Point2D> &raw_points,
                           std::vector<math_utils::Point2D> &smooth_points, std::vector<double> &curva_list) {
    SplineFitting::SplineFittingParams param;
    Spline2D<N_DEG> spline;
      SplineFitting spline_fit;
    auto status =
            spline_fit.gen_piecewise_quintic_spline(raw_points, param, spline);
    if (!status.first) {
      std::cout << status.second << std::endl;
      return false;
    }

    smooth_points.clear();
    curva_list.clear();

    std::vector<math_utils::Point2D> output_pts;
    std::vector<double> acc_s;
      spline_fit.gen_accumulate_s_and_points(raw_points,param,output_pts,acc_s);


    double s = acc_s.front();
    double step = 0.5;
    while (s < acc_s.back()){
      auto tmp_ret = spline.evaluate(s);
        auto xy_dot = spline.evaluate(s,1);
        auto xy_ddot = spline.evaluate(s,2);
        double curva = 0.0;
        if (xy_dot.first ==  Spline2D<N_DEG>::EvaluateStatus::OK &&  xy_ddot.first ==  Spline2D<N_DEG>::EvaluateStatus::OK) {
            curva = (xy_dot.second.x * xy_ddot.second.y - xy_ddot.second.x * xy_dot.second.y)
                    / std::pow((xy_dot.second.x *xy_dot.second.x + xy_dot.second.y * xy_dot.second.y),1.5);
        }
        curva_list.emplace_back(curva);
      if (tmp_ret.first != Spline2D<N_DEG>::EvaluateStatus::OK) {
        std::cout << "s " << s << " EvaluateStatus " << static_cast<int>(tmp_ret.first) << std::endl;
      }
      smooth_points.emplace_back(tmp_ret.second);
      s += step;
    }

    return true;
  }

    static bool SplineSmoothPiece(const std::vector<math_utils::Point2D> &raw_points,
                             std::vector<std::vector<math_utils::Point2D>> &smooth_points_list, std::vector<double> &curva_list) {
        SplineFitting::SplineFittingParams param;
        Spline2D<N_DEG> spline;
        SplineFitting spline_fit;
        auto status =
                spline_fit.gen_piecewise_quintic_spline(raw_points, param, spline);
        if (!status.first) {
            std::cout << status.second << std::endl;
            return false;
        }

        smooth_points_list.clear();
        curva_list.clear();
        std::vector<math_utils::Point2D> tmp;
        tmp.clear();

        std::vector<math_utils::Point2D> output_pts;
        std::vector<double> acc_s;
        spline_fit.gen_accumulate_s_and_points(raw_points,param,output_pts,acc_s);


        SplineFitting::SplineFittingGeneratorInput input;
        auto status_1 =
        spline_fit.gen_spline_fitting_generator_input(output_pts, acc_s, param, input);

        for (int i = 0; i+1 < input.knots.size();i++){
            double s = input.knots[i];
            double s_start = input.knots[i];
            double s_end = input.knots[i+1];
            double step = 0.5;
            tmp.clear();
            while (s < s_end){
                auto tmp_ret = spline.evaluate(s);
                auto xy_dot = spline.evaluate(s,1);
                auto xy_ddot = spline.evaluate(s,2);
                double curva = 0.0;
                if (xy_dot.first ==  Spline2D<N_DEG>::EvaluateStatus::OK &&  xy_ddot.first ==  Spline2D<N_DEG>::EvaluateStatus::OK) {
                    curva = (xy_dot.second.x * xy_ddot.second.y - xy_ddot.second.x * xy_dot.second.y)
                            / std::pow((xy_dot.second.x *xy_dot.second.x + xy_dot.second.y * xy_dot.second.y),1.5);
                }
                curva_list.emplace_back(curva);
                if (tmp_ret.first != Spline2D<N_DEG>::EvaluateStatus::OK) {
                    std::cout << "s " << s << " EvaluateStatus " << static_cast<int>(tmp_ret.first) << std::endl;
                }
                tmp.emplace_back(tmp_ret.second);
                s += step;
            }
            smooth_points_list.emplace_back(tmp);

        }

        return true;
    }

};

} // namespace PolyFit
