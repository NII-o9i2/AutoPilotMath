//
// Created by jojo.feng on 23-2-1.
//

#pragma once

#include "math.h"
#include "qp_solver/osqp_solver.h"
#include "spline.hpp"
#include "string"
#include "common/utils.h"
#include "vector"
#include <algorithm>
#include <iostream>
#include <sstream>

namespace PolyFit {

class SplineFitting {
public:
  static constexpr double kEpsilon = 1e-3;

  using Status = std::pair<bool, std::string>;
  using Point2D = math_utils::Point2D;
  using QuinticPoly = Polynomial<N_DEG>;
  using QuinticPoly2D = Polynomial2D<N_DEG>;

  struct SplineFittingParams {
      double max_accumulate_length{200.0};
      double min_interval_length{1.0};
    double min_segment_length{10.0};
    double max_segment_length{215.0};
    int max_num_segments{1};
    int min_num_segments{1};
    double curv_cal_3pt_distance{5.0};
    double start_pt_curv_limit{1e-3};
    double regulator_0{1.0e8};
    double regulator_other{1.0e5};
    bool has_constraint{false};
    bool use_points_segment{true};

    // osqp_params
    int max_iter{1000};
    double eps_abs{1e-3};
    bool verbose{false};
    bool has_coarse_projection_method{false};

    SplineFittingParams() = default;
  };

  struct SplineFittingGeneratorInput {
    std::vector<double> knots{};
    int num_segments{9};
    Point2D vec_0;
    double kappa_0{0.0};

    SplineFittingGeneratorInput() = default;

    std::string debug_string() const {
      std::stringstream ss;
      ss << "SplineFittingGeneratorInput";
      ss << " num_segments " << num_segments;
      ss << " knots front " << knots.front() << " knots back " << knots.back() << " knots size " << knots.size();
      ss << " vec_0 " << vec_0;
      ss << " kappa_0 " << kappa_0;
      return ss.str();
    }
  };

    void gen_accumulate_s_and_points(const std::vector<Point2D> &input_pts,
                                            const SplineFittingParams &params,
                                            std::vector<Point2D> &output_pts,
                                            std::vector<double> &acc_s) {
        acc_s.clear();
        double s = 0.0;
        int accumulate_index = 0;
        acc_s.emplace_back(0);
        output_pts.emplace_back(input_pts.front());
        for (int i = 0; i + 1 < input_pts.size(); ++i) {
            const double dx = input_pts[i + 1].x - input_pts[i-accumulate_index].x;
            const double dy = input_pts[i + 1].y - input_pts[i-accumulate_index].y;
            if (std::hypot(dx, dy) < params.min_interval_length){
                accumulate_index++;
                continue;
            }else{
                accumulate_index = 0;
            }
            s += std::hypot(dx, dy);
            acc_s.emplace_back(s);
            output_pts.emplace_back(input_pts[i+1]);
            if (s > params.max_accumulate_length) {
                return;
            }
        }
    }
  Status gen_piecewise_quintic_spline(const std::vector<Point2D> &pts,
                                             const SplineFittingParams &params,
                                             Spline2D<N_DEG> &spline) {
      std::vector<double> accumulate_s;
      std::vector<Point2D> point_pts;
      point_pts.clear();
      point_pts.reserve(pts.size());
      gen_accumulate_s_and_points(pts,params,point_pts,accumulate_s);

      std::cout << "point size " << point_pts.size() << " accumulate_s front " << accumulate_s.front()
                << " accumulate_s back " << accumulate_s.back()
                << " accumulate_s size " << accumulate_s.size() << std::endl;

    SplineFittingGeneratorInput input;
    auto status_1 =
        gen_spline_fitting_generator_input(point_pts, accumulate_s, params, input);
    if (!status_1.first) {
      return status_1;
    }

    std::cout << input.debug_string() << std::endl;

    OSQP::OsqpInstance instance;
    auto status_2 =
        gen_osqp_instance(point_pts, accumulate_s, params, input, instance);
    if (!status_2.first) {
      return status_2;
    }

    std::vector<QuinticPoly2D> spline_segments;
    auto cal_osqp_status = cal_piecewise_quintic_spline(
        instance, point_pts[0], params, input, spline_segments);
    if (!cal_osqp_status.first) {
      return cal_osqp_status;
    }

    spline.set_domains_and_polys(input.knots, spline_segments);
    return {true, ""};
  }

  std::vector<double> gen_accumulate_s(const std::vector<Point2D> &pts) {
    std::vector<double> ret;
    ret.reserve(pts.size());
    double s = 0.0;
    ret.emplace_back(s);
    for (int i = 0; i + 1 < pts.size(); ++i) {
      const double dx = pts[i + 1].x - pts[i].x;
      const double dy = pts[i + 1].y - pts[i].y;
      s += std::hypot(dx, dy);
      ret.emplace_back(s);
    }
    return ret;
  }

  double cross_prod(const Point2D &vec1, const Point2D &vec2) {
    return vec1.x * vec2.y - vec1.y * vec2.x;
  }

  double cal_curvature(const Point2D &pt1, const Point2D &pt2,
                              const Point2D &pt3) {
    const auto p12 = Point2D(pt2.x - pt1.x, pt2.y - pt1.y);
    const auto p23 = Point2D(pt3.x - pt2.x, pt3.y - pt2.y);
    const auto p13 = Point2D(pt3.x - pt1.x, pt3.y - pt1.y);
    const double len_p12 = std::hypot(p12.x, p12.y);
    const double len_p23 = std::hypot(p23.x, p23.y);
    const double len_p13 = std::hypot(p13.x, p13.y);
    if (len_p12 < kEpsilon || len_p23 < kEpsilon || len_p13 < kEpsilon) {
      return 0.0;
    }
    return 2 * cross_prod(p12, p23) / (len_p12 * len_p23 * len_p13);
  }

  Status gen_spline_fitting_generator_input(
      const std::vector<Point2D> &pts, const std::vector<double> &accumulate_s,
      const SplineFittingParams &params, SplineFittingGeneratorInput &ret) {
    if (pts.size() < 2) {
      return {false, "pts size < 2"};
    }
    if (accumulate_s.size() != pts.size()) {
      return {false, "accumulate_s size != pts size"};
    }
    const bool is_accumulate_s_invalid =
        (fabs(accumulate_s.front()) > kEpsilon) ||
        (accumulate_s.back() >
         params.max_segment_length * params.max_num_segments + kEpsilon);
    if (is_accumulate_s_invalid) {
      // accumulate_s must be 0~end_s, end_s must less than 900
      return {false, "accumulate_s is out of range"};
    }
    if (accumulate_s.back() <
        params.min_segment_length * params.min_num_segments - kEpsilon) {
      ret.num_segments = 1;
      ret.knots = std::vector<double>{0.0, accumulate_s.back()};
      return {true, ""};
    }

    // cal step and num_segments by params and accumulate_s
    // try to keep the number of segments as most as possible
    if (params.use_points_segment){
        auto check_seg_length = [&](int step_num, int seg){
            for (int i = 0; i< seg ;i++){
                if (accumulate_s[(i + 1) * step_num] - accumulate_s[i * step_num] < params.min_segment_length){
                   return false;
                }
            }
            if (accumulate_s.back() - accumulate_s[(seg - 1) * step_num] < params.min_segment_length){
                return false;
            }
            return true;
        };
        size_t points_num = pts.size();
        size_t step_num = points_num /  params.max_num_segments;
        int num_segments = params.max_num_segments;
        if (step_num * num_segments > pts.size()){
            step_num--;
        }
        while (step_num < N_DEG + 1  || !check_seg_length(step_num,num_segments)){
            if (num_segments <= 1){
                break;
            }
            num_segments --;
            step_num = points_num /  num_segments;
            if (step_num * num_segments > pts.size()){
                step_num--;
            }
        }
        if (num_segments < params.min_num_segments ||
            num_segments > params.max_num_segments) {
            return {false, "num_segments is not valid"};
        }


        ret.num_segments = num_segments;
        ret.knots.clear();
        ret.knots.emplace_back(accumulate_s.front());
        for (int i = 1; i < num_segments ;i++ ){
            ret.knots.emplace_back(accumulate_s[i * step_num] + kEpsilon);
        }
        ret.knots.emplace_back(accumulate_s.back());
        std::cout << "num seg :" << ret.num_segments << std::endl;
        for (auto s : ret.knots){
            std::cout << "knot s" << s << std::endl;
        }
    }else{
        double step = accumulate_s.back() / params.max_num_segments;
        int num_segments = static_cast<int>(accumulate_s.back() / step);
        while ((step < params.min_segment_length || !check_segments_points_num_enough(accumulate_s, step, num_segments)) &&
               num_segments >= params.min_num_segments
               ) {
          num_segments--;
          step = accumulate_s.back() / num_segments;
        }
        if (num_segments < params.min_num_segments ||
            num_segments > params.max_num_segments) {
          return {false, "num_segments is not valid"};
        }
        if (step < params.min_segment_length) {
          return {false, "step is too small"};
        }
        if (!check_segments_points_num_enough(accumulate_s, step, num_segments)) {
          return {false, "segment points num not enough"};
        }

        ret.num_segments = num_segments;
        ret.knots.clear();
        for (double s = accumulate_s.front(); s < accumulate_s.back() + kEpsilon; s += step) {
          ret.knots.emplace_back(s);
        }
        std::cout << "num seg :" << ret.num_segments << std::endl;
        for (auto s : ret.knots){
            std::cout << "knot s" << s << std::endl;
        }
    }



//

    // cal vec_0 and kappa_0
    // todo: why need these?

    ret.vec_0 = Point2D((pts[1].x - pts[0].x) / accumulate_s[1],
                         (pts[1].y - pts[0].y) / accumulate_s[1]);

    const auto idx = std::upper_bound(accumulate_s.begin(), accumulate_s.end(),
                                      params.curv_cal_3pt_distance);
    const auto idx_2 =
        std::upper_bound(accumulate_s.begin(), accumulate_s.end(),
                         2 * params.curv_cal_3pt_distance);
    if (idx == accumulate_s.end() && idx_2 == accumulate_s.end()) {
      ret.kappa_0 = 0.0;
      return {true, "the first point is too close to the end"};
    }

    const auto &pt_0 = pts.front();
    const auto &pt_1 = pts[idx - accumulate_s.begin()];
    const auto &pt_2 = pts[idx_2 - accumulate_s.begin()];
    const auto kappa = cal_curvature(pt_0, pt_1, pt_2);
    ret.kappa_0 = std::fabs(kappa) < params.start_pt_curv_limit ? 0.0 : kappa;

    return {true, ""};
  }

  Status
  gen_osqp_instance(const std::vector<Point2D> &pts,
                    const std::vector<double> &accumulate_s,
                    const SplineFittingParams &params,
                    const SplineFittingGeneratorInput &generator_input,
                    OSQP::OsqpInstance &instance) {
    assert(accumulate_s.size() == pts.size());

    std::vector<std::pair<int, int>> seg_idx_pairs;
    auto tmp_status = get_idx_pairs_base_on_breaks(
        accumulate_s, generator_input.knots, seg_idx_pairs);
    if (!tmp_status.first) {
      return tmp_status;
    }
    for (int i= 0; i< 32;i++){
        instance.objective_vector.emplace_back(1.0);
    }

    const int N = gen_objective_constraint_and_vector(
        pts, accumulate_s, params, generator_input, seg_idx_pairs,
        instance.objective_matrix, instance.objective_vector);

    if (generator_input.num_segments == 1) {
      instance.constraint_matrix.SetFromTriplets({}, 0, N);
      return {true, ""};
    }

    std::vector<Triplet> triplets;
    const int M =
            gen_constraint_matrix_triplets(generator_input, params, triplets, accumulate_s, seg_idx_pairs);
    instance.constraint_matrix.SetFromTriplets(triplets, M, N);
    instance.lower_bounds.resize(M);
    instance.upper_bounds.resize(M);

    if (params.has_constraint) {
      instance.lower_bounds[M - 3] = generator_input.kappa_0;
      instance.lower_bounds[M - 2] = generator_input.vec_0.x;
      instance.lower_bounds[M - 1] = generator_input.vec_0.y;
      instance.upper_bounds[M - 3] = generator_input.kappa_0;
      instance.upper_bounds[M - 2] = generator_input.vec_0.x;
      instance.upper_bounds[M - 1] = generator_input.vec_0.y;
    }

    return {true, ""};
  }

  int gen_objective_constraint_and_vector(
      const std::vector<Point2D> &pts, const std::vector<double> &accumulate_s,
      const SplineFittingParams &params,
      const SplineFittingGeneratorInput &generator_input,
      const std::vector<std::pair<int, int>> &seg_idx_pairs,
      CSC &P_upper_triangle, std::vector<double> &q) {
    const int num_segs = static_cast<int>(generator_input.knots.size()) - 1;
    const int num_order = N_DEG + 1;
    const int half_N = num_segs * num_order;
    const int N = N_DIM * half_N;
    // todo: one_piece_P_size?
    const int one_piece_P_size = (num_order + 1) * num_order / 2;
    const int P_nzmax_dim = one_piece_P_size * num_segs;
    const int P_nzmax = P_nzmax_dim * N_DIM;

    P_upper_triangle.Reserve(P_nzmax, N, N);
    auto &P_p = P_upper_triangle.p;
    auto &P_i = P_upper_triangle.i;
    auto &P_x = P_upper_triangle.x;
    P_p[N] = P_nzmax;
    q.resize(N);

    for (int seg = 0; seg < num_segs; seg++) {
      const int start_idx = seg_idx_pairs[seg].first;
      const int end_idx = seg_idx_pairs[seg].second;
      const int X_num = end_idx - start_idx + 1;
      double X[num_order][X_num];

      for (int i = start_idx; i <= end_idx; i++) {
        const double dt = accumulate_s[i] - accumulate_s[start_idx];
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt3 * dt;
        const double dt5 = dt4 * dt;
        X[0][i - start_idx] = dt5 / FACTORIALS[5];
        X[1][i - start_idx] = dt4 / FACTORIALS[4];
        X[2][i - start_idx] = dt3 / FACTORIALS[3];
        X[3][i - start_idx] = dt2 / FACTORIALS[2];
        X[4][i - start_idx] = dt / FACTORIALS[1];
        X[5][i - start_idx] = 1.0;
      }

      const auto inner_prod_with_x = [&](const double *X_i) -> double {
        double ret = 0.0;
        for (int i = start_idx; i <= end_idx; i++) {
          ret += (pts[i].x - pts[0].x) * X_i[i - start_idx];
        }
        return ret;
      };
      const auto inner_prod_with_y = [&](const double *X_i) -> double {
        double ret = 0.0;
        for (int i = start_idx; i <= end_idx; i++) {
          ret += (pts[i].y - pts[0].y) * X_i[i - start_idx];
        }
        return ret;
      };
      const auto inner_prod = [&](const double *xi,
                                  const double *xj) -> double {
        double ret = 0.0;
        for (int i = 0; i < X_num; i++) {
          ret += (xi[i] * xj[i]);
        }
        return ret;
      };

      for (int i = 0; i < num_order; i++) {
        const int p_idx = seg * num_order + i;
        const int p_val = one_piece_P_size * seg + (i + 1) * i / 2;
        P_p[p_idx] = p_val;
        P_p[half_N + p_idx] = P_nzmax_dim + p_val;
        const int idx = seg * num_order + i;
        q[idx] = -inner_prod_with_x(X[i]);
        q[idx + half_N] = -inner_prod_with_y(X[i]);
        for (int j = i; j < num_order; j++) {
          const double val = inner_prod(X[i], X[j]);
          const int i_idx = one_piece_P_size * seg + (j + 1) * j / 2 + i;
          P_i[i_idx] = idx;
          P_x[i_idx] = val;
          P_i[i_idx + P_nzmax_dim] = idx + half_N;
          P_x[i_idx + P_nzmax_dim] = val;
        }
      }
    }

//    const std::array<double, 3> regulator_vals_0{params.regulator_0,
//                                                 params.regulator_0 / 10.0,
//                                                 params.regulator_0 / 100.0};
//    const std::array<double, 3> regulator_vals{params.regulator_other,
//                                               params.regulator_other / 10.0,
//                                               params.regulator_other / 100.0};
//    const std::array<int, 3> regulator_idxes{0, 2, 5};
//
//    for (int seg = 0; seg < num_segs; seg++) {
//      for (int j = 0; j < REGULATOR_TOP_DEGREE; j++) {
//        const int i_idx = one_piece_P_size * seg + regulator_idxes[j];
//        if (seg == 0) {
//          P_x[i_idx] += regulator_vals_0[j];
//          P_x[i_idx + P_nzmax_dim] += regulator_vals_0[j];
//        } else {
//          P_x[i_idx] += regulator_vals[j];
//          P_x[i_idx + P_nzmax_dim] += regulator_vals[j];
//        }
//      }
//    }
    return N;
  }

    int gen_constraint_matrix_triplets(
            const SplineFittingGeneratorInput &generator_input,
            const SplineFittingParams &params, std::vector<Triplet> &triplets, const std::vector<double> &accumulate_s,
            const std::vector<std::pair<int, int>> &seg_idx_pairs){
    const int num_segs = generator_input.num_segments;
    const int num_order = N_DEG + 1;
    const int num_connections = num_segs - 1;
    const int M_for_continuous = N_DIM * num_connections * N_CONTINUOUS;
    const int one_piece_A_size =
        N_CONTINUOUS * (num_order + num_order - N_CONTINUOUS + 1) / 2;
    const int A_nzmax =
        num_connections * N_DIM * (N_CONTINUOUS + one_piece_A_size) + 4;
    const int num_x_connections_total = num_connections * N_CONTINUOUS;
    const int num_y_connections_total = num_segs * num_order;
    triplets.reserve(A_nzmax);

    for (int n = 0; n < num_connections; n++) {
        const double T = generator_input.knots[n+1] - generator_input.knots[n];
      for (int c = 0; c < N_CONTINUOUS; c++) {
        for (int j = 0; j < N_DEG - c; j++) {
          // maybe unsafe
          const double val_l =
              std::pow(T, N_DEG - j - c) / FACTORIALS[N_DEG - j - c];
          const int idx = n * N_CONTINUOUS + c;
          const int idy_l = n * num_order + j;
          triplets.emplace_back(idx, idy_l, val_l);
          triplets.emplace_back(idx + num_x_connections_total,
                                idy_l + num_y_connections_total, val_l);
        }
        const double val_l = 1.0;
        const double val_r = -1.0;
        const int idx = n * N_CONTINUOUS + c;
        const int idy_l = n * num_order + N_DEG - c;
        const int idy_r = (n + 1) * num_order + N_DEG - c;
        triplets.emplace_back(idx, idy_l, val_l);
        triplets.emplace_back(idx + num_x_connections_total,
                              idy_l + num_y_connections_total, val_l);
        triplets.emplace_back(idx, idy_r, val_r);
        triplets.emplace_back(idx + num_x_connections_total,
                              idy_r + num_y_connections_total, val_r);
      }
    }

    if (params.has_constraint) {
      const auto &vec_0 = generator_input.vec_0;
      const auto curv_factor =
          1.0 / std::pow(vec_0.x * vec_0.x + vec_0.y * vec_0.y, 1.5);
      triplets.emplace_back(M_for_continuous, 3, -vec_0.y * curv_factor);
      triplets.emplace_back(M_for_continuous, 3 + num_y_connections_total,
                            vec_0.x * curv_factor);
      // init heading
      triplets.emplace_back(M_for_continuous + 1, 4, 1.0);
      triplets.emplace_back(M_for_continuous + 2, 4 + num_y_connections_total,
                            1.0);
    }

    auto triplet_cmp = [](const Triplet &a, const Triplet &b) -> bool {
      return a.y == b.y ? a.x < b.x : a.y < b.y;
    };
    std::sort(triplets.begin(), triplets.end(), triplet_cmp);

    return params.has_constraint ? M_for_continuous + 3 : M_for_continuous;
  }

  Status cal_piecewise_quintic_spline(
      OSQP::OsqpInstance &instance, const Point2D &pt_0,
      const SplineFittingParams &params,
      const SplineFittingGeneratorInput &generator_input,
      std::vector<QuinticPoly2D> &spline_segments) {
    const int num_segs = generator_input.num_segments;
    const int num_order = N_DEG + 1;

    OSQP::OsqpSettings settings;
    settings.max_iter = params.max_iter;
    settings.eps_abs = params.eps_abs;
    settings.verbose = params.verbose;

    OSQP::OsqpSolver solver;
    auto init_status = solver.Init(instance, settings);
    if (!init_status.first) {
      return init_status;
    }

    const auto solve_status = solver.Solve();
    if (solve_status != OSQP::OsqpExitCode::kOptimal && solve_status!= OSQP::OsqpExitCode::kMaxIterations
            && solve_status!= OSQP::OsqpExitCode::kTimeLimitReached) {
      return {false, OSQP::ToString(solve_status)};
    }

    spline_segments.clear();
    spline_segments.reserve(num_segs);
    for (int i = 0; i < num_segs; i++) {
      QuinticPoly::Coeffs x_coeffs;
      QuinticPoly::Coeffs y_coeffs;
      for (int j = 0; j < num_order; j++) {
        double val = 0.0;
        (void)solver.GetPrimalSolutionAtIndex(i * num_order + j, &val);
        x_coeffs[j] = val + (j == N_DEG ? pt_0.x : 0.0);

        (void)solver.GetPrimalSolutionAtIndex(
            i * num_order + j + num_segs * num_order, &val);
        y_coeffs[j] = val + (j == N_DEG ? pt_0.y : 0.0);
      }
      spline_segments.emplace_back(x_coeffs, y_coeffs);
      for (int k = 0;k < x_coeffs.size();k++){
          std::cout << " x_coeffs " << k << " : " << x_coeffs[k]<< std::endl;
      }
        for (int k = 0;k < y_coeffs.size();k++){
            std::cout << " y_coeffs " << k << " : " << y_coeffs[k]<< std::endl;
        }
    }

    return {true, ""};
  }

  Status
  get_idx_pairs_base_on_breaks(const std::vector<double> &accumulate_s,
                               const std::vector<double> &knots,
                               std::vector<std::pair<int, int>> &ret) {
    const auto num_segs = static_cast<int>(knots.size()) - 1;
    const auto num_pts = static_cast<int>(accumulate_s.size());
    ret.reserve(num_segs);
    int idx_begin = 0;
    for (int i = 0; i < num_segs; i++) {
      const auto up_bound = knots[i + 1];
      const auto up_it = std::upper_bound(begin(accumulate_s) + idx_begin,
                                          end(accumulate_s), up_bound);
      const int idx_end =
          up_it == end(accumulate_s)
              ? num_pts - 1
              : static_cast<int>(up_it - begin(accumulate_s)) - 1;
      // if (idx_end - idx_begin < N_DEG + 1) {
      //   std::stringstream ss;
      //   ss <<  "Too few points in segment";
      //   ss << " idx_end " << idx_end;
      //   ss << " idx_begin " << idx_begin;
      //   return {false, ss.str()};
      // }
      if (idx_end - idx_begin <= 0) {
        std::stringstream ss;
        ss <<  "Too few points in segment";
        ss << " idx_end " << idx_end;
        ss << " idx_begin " << idx_begin;
        return {false, ss.str()};
      }

      ret.emplace_back(idx_begin, idx_end);
      idx_begin = idx_end;
    }
    return {true, ""};
  }

  bool check_segments_points_num_enough(const std::vector<double> &accumulate_s,
                                                const double& step, const int& num_segments) {
    std::vector<double> knots;
    for (double s = accumulate_s.front(); s < accumulate_s.back() + kEpsilon; s += step) {
      knots.emplace_back(s);
    }

    const auto num_pts = static_cast<int>(accumulate_s.size());
    int idx_begin = 0;
    for (int i = 0; i < num_segments; i++) {
      const auto up_bound = knots[i + 1];
      const auto up_it = std::upper_bound(std::begin(accumulate_s) + idx_begin,
                                          std::end(accumulate_s), up_bound);
      const int idx_end =
          up_it == std::end(accumulate_s)
              ? num_pts - 1
              : static_cast<int>(up_it - std::begin(accumulate_s)) - 1;
      if (idx_end - idx_begin < N_DEG + 1) {
        std::cout << "Too few points in segment" 
                  << " num_segments " << num_segments
                  << " step " << step
                  << " i " << i << " idx_end " << idx_end 
                  << " idx_begin " << idx_begin << std::endl;
        return false;
      }

      idx_begin = idx_end;
    }

    return true;
  }
};

} // namespace PolyFit