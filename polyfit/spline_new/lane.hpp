// //
// // Created by ted.tai on 22-11-22.
// //
// #pragma once

// #include "geometry.h"
// #include "line_segment2d.h"
// #include "spline/polynomial.hpp"
// #include "spline/spline.hpp"
// #include <limits>
// #include <utility>

// namespace PolyFit {

// class Lane {
// public:
//   static constexpr int MAX_SEGS_TO_CHECK_SIZE = 2;
//   static constexpr double kEpsilon = 1e-3;
//   // kMaxIterNewton 为6的时候更精准
//   // kMaxIterProjection 为3的时候更精准
//   static constexpr int kMaxIterNewton = 6;
//   static constexpr int kMaxIterProjection = 2;

//   using QuinticSpline2D = Spline2D<N_DEG>;

//   enum class SegmentType { NORMAL = 0, BEGIN = 1, END = 2 };

//   struct ProjectionPt {
//     double t; /// projection t in spline
//     double d; /// projection distance
//     ProjectionPt(double t, double d) : t(t), d(d) {}
//   };

//   Lane() = default;

//   explicit Lane(const QuinticSpline2D &spline) : spline_(spline) {}

//   QuinticSpline2D *mutable_spline2d() { return &spline_; }

//   void init_segments(const std::vector<double> &domains,
//                      const std::vector<QuinticPoly2D> &polys) {
//     spline_.set_domains_and_polys(domains, polys);

//     accumulate_s_.clear();
//     for (double s = spline_.get_start_domain();
//          s < spline_.get_end_domain() + segment_step_; s += segment_step_) {
//       accumulate_s_.emplace_back(s);
//     }
//     num_segments_ = static_cast<int>(accumulate_s_.size()) - 1;
//     num_points_ = num_segments_ + 1;
//     length_ = accumulate_s_.back();

//     pts_.clear();
//     for (const auto &s : accumulate_s_) {
//       pts_.emplace_back(get_position_unsafe(s));
//     }

//     if (num_points_ < 2) {
//       return;
//     }

//     // TODO: segment_2d
//     segments_.reserve(num_segments_);
//     for (int i = 0; i < num_segments_; ++i) {
//       segments_.emplace_back(
//           planning_math::Vec2d(pts_[i].x, pts_[i].y),
//           planning_math::Vec2d(pts_[i + 1].x, pts_[i + 1].y));
//     }
//   }

//   const QuinticSpline2D &spline2D() const { return spline_; }

//   std::array<int, MAX_SEGS_TO_CHECK_SIZE>
//   GenCoarseSearchSegmentIdxes(const Point2D &pt) const {
//     double min_square_dis_coarse_search = std::numeric_limits<double>::max();
//     int min_idx = -1;
//     const int num_seg = spline_.num_segments();
//     std::array<int, MAX_SEGS_TO_CHECK_SIZE> ret{};
//     ret.fill(-1);
//     int idx = 0;
//     for (int n = 0; n < num_seg; n++) {
//       const auto &cir = spline_.CircleAt(n);
//       const auto radius = cir.radius;
//       const double dx = pt.x - cir.center.first;
//       const double dy = pt.y - cir.center.second;
//       const double square_distance = dx * dx + dy * dy;
//       if (square_distance < radius * radius) {
//         ret[(idx++) % MAX_SEGS_TO_CHECK_SIZE] = n;
//       }
//       if (min_square_dis_coarse_search > square_distance) {
//         min_square_dis_coarse_search = square_distance;
//         min_idx = n;
//       }
//     }
//     if (ret[0] == -1) {
//       ret[0] = min_idx;
//     }
//     return ret;
//   }

//   inline SegmentType GetSegmentType(int n) const {
//     if (n == 0) {
//       return SegmentType::BEGIN;
//     } else if (n + 1 == spline_.num_segments()) {
//       return SegmentType::END;
//     } else {
//       return SegmentType::NORMAL;
//     }
//   }

//   static std::pair<bool, double>
//   GetArcLengthByNewtonMethod(const Point2D &pt, double initial_guess,
//                              double len,
//                              const QuinticSpline2D::PolynomialType &seg) {
//     const double val_lb = 0.0;
//     const double val_ub = len;
//     double x = fplus::clamp(val_lb, val_ub, initial_guess);
//     for (int i = 0; i < kMaxIterNewton; ++i) {
//       const auto p = seg.Evaluate(x);
//       const auto dp = seg.Evaluate(x, 1);
//       const auto ddp = seg.Evaluate(x, 2);
//       const auto d1 =
//           (p.first - pt.x) * dp.first + (p.second - pt.y) * dp.second;
//       const auto d2 = (p.first - pt.x) * ddp.first +
//                       (p.second - pt.y) * ddp.second + dp.first * dp.first +
//                       dp.second * dp.second;
//       const auto dx = -d1 / d2;
//       if (fabs(dx) < kEpsilon) {
//         return {true, x};
//       }
//       if (x + dx > val_ub) {
//         x = val_ub;
//         return {false, initial_guess};
//       } else if (x + dx < val_lb) {
//         x = val_lb;
//         return {false, initial_guess};
//       }
//       x += dx;
//     }
//     return {true, initial_guess};
//   }
//   static double
//   GetClosetArcLengthAtSeg(const Point2D &pt,
//                           const QuinticSpline2D::PolynomialType &seg,
//                           const double max_length) {
//     auto getDtAtSeg = [](const Point2D &pt, const double t,
//                          const QuinticSpline2D::PolynomialType &seg) ->
//                          double {
//       const auto p = seg.Evaluate(t);
//       const double dx = p.first - pt.x;
//       const double dy = p.second - pt.y;
//       return dx * dx + dy * dy;
//     };
//     auto getPt = [](const double t, const double t1, const double t2,
//                     const double t3, const double Dt1, const double Dt2,
//                     const double Dt3) -> double {
//       const auto tmp1 = Dt1 / (t1 - t2) / (t1 - t3);
//       const auto tmp2 = Dt2 / (t2 - t1) / (t2 - t3);
//       const auto tmp3 = Dt3 / (t3 - t1) / (t3 - t2);
//       return (t - t2) * (t - t3) * tmp1 + (t - t1) * (t - t3) * tmp2 +
//              (t - t1) * (t - t2) * tmp3;
//     };
//     auto cmp = [](const std::pair<double, double> &a,
//                   const std::pair<double, double> &b) -> bool {
//       return a.second < b.second;
//     };
//     std::array<std::pair<double, double>, 4> t_Dt;
//     double t1 = 0.0;
//     double t2 = 0.5 * max_length;
//     double t3 = max_length;
//     auto t_star_last = t2;
//     for (int i = 0; i < kMaxIterProjection; ++i) {
//       const auto Dt1 = getDtAtSeg(pt, t1, seg);
//       const auto Dt2 = getDtAtSeg(pt, t2, seg);
//       const auto Dt3 = getDtAtSeg(pt, t3, seg);
//       const auto t12 = t1 - t2;
//       const auto t31 = t3 - t1;
//       const auto t23 = t2 - t3;
//       const auto y23 = t2 * t2 - t3 * t3;
//       const auto y31 = t3 * t3 - t1 * t1;
//       const auto y12 = t1 * t1 - t2 * t2;
//       const double val = (Dt1 * t23 + Dt2 * t31 + Dt3 * t12);
//       if (fabs(val) < kEpsilon) {
//         break;
//       }
//       auto t_star = 0.5 * (Dt1 * y23 + Dt2 * y31 + Dt3 * y12) / val;
//       auto Pt_star = getPt(t_star, t1, t2, t3, Dt1, Dt2, Dt3);
//       t_Dt[0] = {t1, Dt1};
//       t_Dt[1] = {t2, Dt2};
//       t_Dt[2] = {t3, Dt3};
//       t_Dt[3] = {t_star, Pt_star};
//       std::sort(t_Dt.begin(), t_Dt.end(), cmp);
//       t_star = t_Dt[0].first;
//       t1 = t_Dt[0].first;
//       t2 = t_Dt[1].first;
//       t3 = t_Dt[2].first;
//       if (fabs(t_star - t_star_last) < kEpsilon) {
//         break;
//       }
//       t_star_last = t_star;
//     }
//     return GetArcLengthByNewtonMethod(pt, t_star_last, max_length,
//     seg).second;
//   }

//   static ProjectionPt
//   GetProjectionAtSeg(const Point2D &pt,
//                      const QuinticSpline2D::PolynomialType &seg,
//                      const double max_length, const SegmentType seg_type) {
//     auto t_final = GetClosetArcLengthAtSeg(pt, seg, max_length);
//     if (seg_type == SegmentType::NORMAL) {
//       t_final = fplus::clamp(0.0, max_length, t_final);
//     } else if (seg_type == SegmentType::BEGIN) {
//       t_final = fmin(max_length, t_final);
//     } else {
//       t_final = fmax(0.0, t_final);
//     }
//     auto getLateralDistanceAtT =
//         [](const Point2D &pt, const double t,
//            const QuinticSpline2D::PolynomialType &seg) -> double {
//       const auto p = seg.Evaluate(t);
//       const double dx = pt.x - p.first;
//       const double dy = pt.y - p.second;
//       const auto vec_dir = seg.Evaluate(t, 1);
//       const auto cross_product = dy * vec_dir.first - dx * vec_dir.second;
//       const double l = sqrt(dx * dx + dy * dy);
//       return cross_product > 0.0 ? l : -l;
//     };
//     return {t_final, getLateralDistanceAtT(pt, t_final, seg)};
//   }

//   ProjectionPt GetProjection(const Point2D &pt) const {
//     const auto segs_to_check = GenCoarseSearchSegmentIdxes(pt);
//     const auto &domains = spline_.accumulate_domains();
//     if (segs_to_check[1] == -1) {
//       int n = segs_to_check[0];
//       const auto proj_pt =
//           GetProjectionAtSeg(pt, spline_.PolyAt(n), domains[n + 1] -
//           domains[n],
//                              GetSegmentType(n));
//       return {proj_pt.t + domains[n], proj_pt.d};
//     }
//     int n0 = segs_to_check[0], n1 = segs_to_check[1];
//     const auto proj_pt0 =
//         GetProjectionAtSeg(pt, spline_.PolyAt(n0),
//                            domains[n0 + 1] - domains[n0],
//                            GetSegmentType(n0));
//     const auto proj_pt1 =
//         GetProjectionAtSeg(pt, spline_.PolyAt(n1),
//                            domains[n1 + 1] - domains[n1],
//                            GetSegmentType(n1));
//     if (fabs(proj_pt0.d) < fabs(proj_pt1.d)) {
//       return {proj_pt0.t + domains[n0], proj_pt0.d};
//     }
//     return {proj_pt1.t + domains[n1], proj_pt1.d};
//   }
//   ProjectionPt GetProjectionAtDomain(const Point2D &pt, const double start,
//                                      const double end) const {
//     const auto point = msquare::planning_math::Vec2d(pt.x, pt.y);
//     auto get_dist_with_set_pt =
//         [&](const msquare::planning_math::LineSegment2d &seg) {
//           return seg.DistanceSquareTo(point);
//         };
//     int start_idx = std::max(0, static_cast<int>(std::floor(start)));
//     int end_idx = std::min(num_segments_ - 1,
//     static_cast<int>(std::ceil(end)));
//     double min_dis = std::numeric_limits<double>::max();
//     int min_index = -1;
//     for (int i = start_idx; i <= end_idx; i++) {
//       const auto dis = get_dist_with_set_pt(segments_[i]);
//       if (dis < min_dis) {
//         min_dis = dis;
//         min_index = i;
//       }
//     }
//     const auto &nearest_seg = segments_[min_index];
//     double min_distance = std::sqrt(min_dis);
//     const auto prod = nearest_seg.ProductOntoUnit(point);
//     const auto proj = nearest_seg.ProjectOntoUnit(point);
//     double accumulate_s = 0.0;
//     double lateral = 0.0;
//     if (min_index == 0) {
//       accumulate_s = std::min(proj, nearest_seg.length());
//       if (proj < 0) {
//         lateral = prod;
//       } else {
//         lateral = (prod > 0.0 ? 1 : -1) * min_distance;
//       }
//     } else if (min_index == segments_.size() - 1) {
//       accumulate_s = accumulate_s_[min_index] + std::max(0.0, proj);
//       if (proj > 0) {
//         lateral = prod;
//       } else {
//         lateral = (prod > 0.0 ? 1 : -1) * min_distance;
//       }
//     } else {
//       accumulate_s = accumulate_s_[min_index] +
//                      std::max(0.0, std::min(proj, nearest_seg.length()));
//       lateral = (prod > 0.0 ? 1 : -1) * min_distance;
//     }
//     return {accumulate_s, lateral};
//   }

//   // 用直线线段segment代替曲线, 是否会不准?
//   ProjectionPt GetProjectionCoarse(const Point2D &pt) const {
//     const auto segs_to_check = GenCoarseSearchSegmentIdxes(pt);
//     const auto &domains = spline_.accumulate_domains();
//     if (segs_to_check[1] == -1) {
//       int n = segs_to_check[0];
//       const auto proj_pt =
//           GetProjectionAtDomain(pt, domains[n], domains[n + 1]);
//       return {proj_pt.t, proj_pt.d};
//     }
//     int n0 = segs_to_check[0], n1 = segs_to_check[1];
//     const auto proj_pt0 =
//         GetProjectionAtDomain(pt, domains[n0], domains[n0 + 1]);
//     const auto proj_pt1 =
//         GetProjectionAtDomain(pt, domains[n1], domains[n1 + 1]);
//     if (fabs(proj_pt0.d) < fabs(proj_pt1.d)) {
//       return {proj_pt0.t, proj_pt0.d};
//     }
//     return {proj_pt1.t, proj_pt1.d};
//   }

//   double GetCurvatureUnsafe(const double t) const {
//     const auto &vel = spline_.Evaluate(t, 1).second;
//     const auto &acc = spline_.Evaluate(t, 2).second;
//     const auto c0 = vel.first * acc.second - vel.second * acc.first;
//     return c0 / std::pow(vel.first * vel.first + vel.second * vel.second,
//     1.5);
//   }

//   double GetDCurvatureUnsafe(const double t) const {
//     const auto vel = spline_.Evaluate(t, 1).second;
//     const auto acc = spline_.Evaluate(t, 2).second;
//     const auto jerk = spline_.Evaluate(t, 3).second;
//     const double dx = vel.first, dy = vel.second;
//     const double ddx = acc.first, ddy = acc.second;
//     const double dddx = jerk.first, dddy = jerk.second;
//     const double a1 = dx * dddy - dy * dddx;
//     const double b1 = dx * ddx + dy * ddy;
//     const double c1 = dx * ddy - dy * ddx;
//     const double d1 = dx * dx + dy * dy;
//     return (b1 * d1 - 3.0 * a1 * c1) / d1 / d1 / d1;
//   }

//   std::array<double, 3>
//   GetHeadingCurvatureDCurvatureUnsafe(const double t) const {
//     const auto vel = spline_.Evaluate(t, 1).second;
//     const auto acc = spline_.Evaluate(t, 2).second;
//     const auto jerk = spline_.Evaluate(t, 3).second;
//     const double dx = vel.first, dy = vel.second;
//     const double ddx = acc.first, ddy = acc.second;
//     const double dddx = jerk.first, dddy = jerk.second;
//     const double a1 = dx * dddy - dy * dddx;
//     const double b1 = dx * ddx + dy * ddy;
//     const double c1 = dx * ddy - dy * ddx;
//     const double d1 = dx * dx + dy * dy;
//     const double heading = std::atan2(dy, dx);
//     const double curvature = c1 / std::pow(d1, 1.5);
//     const double d_curvature = (b1 * d1 - 3.0 * a1 * c1) / d1 / d1 / d1;
//     return {heading, curvature, d_curvature};
//   }

//   double GetHeadingUnsafe(const double t) const {
//     const auto &vel = spline_.Evaluate(t, 1).second;
//     return std::atan2(vel.second, vel.first);
//   }

//   Point2D get_position_unsafe(const double t) const {
//     auto tmp = spline_.evaluate(t);
//     return {tmp.second.first, tmp.second.second};
//   }

//   inline double length() const { return length_; }

// private:
//   QuinticSpline2D spline_;
//   const double segment_step_ = 1.0;
//   int num_points_ = 0;
//   int num_segments_ = 0;
//   double length_ = 0.0;
//   std::vector<Point2D> pts_;
//   std::vector<double> accumulate_s_;
//   std::vector<planning_math::LineSegment2d> segments_;
// };

// } // namespace PolyFit
