//
// Created by 冯晓彤 on 2023/4/28.
//
#include "Eigen/Dense"
#include "iostream"
#include "polyfit/polyfit.h"
#include "polyfit_osqp.h"
#include "lateral_longitudinal_ilqr.h"
#include <benchmark/benchmark.h>
// #include "njson/njson_test.h"

static std::vector<math_utils::Point2D> prepare_poly_fit_data() {
  double noise_std = 0.1;
  double A0 = 0.0;
  double A1 = 1.0;
  double A2 = 0.0;
  double A3 = 0.0;
  double B0 = 1.0;
  double B1 = 2.0;
  double B2 = 0.05;
  double B3 = 0.001;
  int point_size = 60;
  double point_resolution = 1.0;

  return PolyFit::Gen_random_curv(noise_std, A0, A1, A2, A3, B0, B1, B2, B3,
                                  point_size, point_resolution);
}

static std::vector<math_utils::Point2D> get_wm_path_points() {
   std::vector<double> point_x = {
           0,
           -2.43316,
           -7.74198,
           -26.9626,
           -30.7752,
           -35.987,
           -42.0991,
           -49.5649,
           -57.0095,
           -62.1044,
           -67.2854,
           -67.2854,
           -71.7438,
           -75.388,
           -82.6277,
           -90.6996,
           -99.1671,
           -106.264,
           -111.995,
           -118.592,
           -123.857,
           -128.659,
           -130.598,
           -130.598,
           -135.11,
           -139.39,
           -159.405,
           -159.405,
           -162.071,
           -162.071,
           -162.551,
           -162.551,
           -164.191,
           -164.191,
           -179.133
   };
    std::vector<double> point_y ={
            0,
            4.37759,
            14.0168,
            49.1319,
            56.1682,
            65.867,
            77.3446,
            91.5019,
            105.674,
            115.428,
            125.436,
            125.436,
            134.096,
            141.23,
            155.504,
            171.607,
            188.63,
            202.981,
            214.653,
            228.139,
            238.93,
            248.833,
            252.912,
            252.912,
            262.383,
            271.435,
            313.984,
            313.984,
            319.635,
            319.635,
            320.661,
            320.661,
            324.157,
            324.157,
            356.064,
    };

    assert(point_x.size() ==point_y.size());


    std::vector<double> point2_x = {
            0,
            -8.17967,
            -20.1002,
            -28.0118,
            -34.4169,
            -39.7974,
            -43.1984,
            -43.1984,
            -45.6311,
            -50.9389,
            -70.1559,
            -73.9677,
            -79.1784,
            -85.2893,
            -92.7537,
            -100.197,
            -105.291,
            -110.471,
            -110.471,
            -114.928,
            -118.572,
            -125.81,
            -133.88,
            -142.346,
            -149.441,
            -155.171,
            -161.767,
            -167.03,
            -171.831,
            -173.77,
            -173.77,
            -178.28,
            -182.56,
            -202.571
    };
    std::vector<double> point2_y = {
            0,
            14.1217,
            34.9335,
            48.8161,
            60.1272,
            69.7099,
            75.8162,
            75.8162,
            80.194,
            89.8336,
            124.95,
            131.987,
            141.686,
            153.164,
            167.322,
            181.494,
            191.249,
            201.257,
            201.257,
            209.917,
            217.052,
            231.326,
            247.43,
            264.453,
            278.805,
            290.478,
            303.963,
            314.755,
            324.658,
            328.737,
            328.737,
            338.209,
            347.261,
            389.811
    };
    assert(point2_x.size() ==point2_y.size());

    std::vector<math_utils::Point2D> res;
    res.clear();
    for (size_t i = 0; i < point_x.size(); i++) {
        res.emplace_back(point_x[i],point_y[i]);
    }
    return res;
};

static void BM_Poly_fit_osqp(benchmark::State &state) {
  auto line_point = prepare_poly_fit_data();
  std::vector<double> coef;
  for (auto _ : state) {
    PolyFit::PolyLineFitOSQP(line_point, 3, coef);
  }
}

static void BM_Poly_fit_spline(benchmark::State &state) {
  auto line_point = prepare_poly_fit_data();
  std::vector<math_utils::Point2D> smooth_points;
  for (auto _ : state) {
    PolyFit::PolyFitSplineSmooth(line_point, smooth_points);
  }
}

static void BM_Poly_fit_spline_new(benchmark::State &state) {
  auto line_point = prepare_poly_fit_data();
  std::vector<math_utils::Point2D> smooth_points;
  std::vector<double> curva;
  for (auto _ : state) {
    PolyFit::PolyFitSplineSmoothNew(line_point, smooth_points,curva);
  }
}

static void BM_Poly_fit_eigen(benchmark::State &state) {
  auto line_point = prepare_poly_fit_data();
  Eigen::VectorXd line;
  for (auto _ : state) {
    PolyFit::PolyLineFit(line_point, 3, line, true);
  }
}

static void BM_CILQR(benchmark::State &state) {
  ILQR::VehicleModelBicycle vehicle_model_bicycle;
  ILQR::ILQRParam param;
  param.delta_t = 0.2;

  vehicle_model_bicycle.update_parameter(param);

  LateralLongitudinalMotion motion;
  PlanningPoint point_init;
  point_init.position.x = 10.0;
  point_init.position.y = 4.0;
  point_init.theta = 0.2;
  point_init.velocity = 25.0;
  point_init.acceleration  = 0.1;

  motion.init("/home/SENSETIME/fengxiaotong/ws/xviz/common_math/algorithm/env_simulator/data/straight_lanes_case_1.json",point_init);
  for (auto _ : state) {
    motion.init("/home/SENSETIME/fengxiaotong/ws/xviz/common_math/algorithm/env_simulator/data/straight_lanes_case_1.json",point_init);
    motion.execute();
  }

}
// Register the function as a benchmark
BENCHMARK(BM_Poly_fit_eigen);
BENCHMARK(BM_Poly_fit_osqp);
BENCHMARK(BM_Poly_fit_spline);
BENCHMARK(BM_Poly_fit_spline_new);
BENCHMARK(BM_CILQR);

// Run the benchmark
#ifdef BENCHMARK_MODE
BENCHMARK_MAIN();
#else

int main(void) {

    for (double s = 0; s < 5.2; s += 2.0)
        std::cout << s << std::endl;


    // polyfit with eigen
  std::cout << "======" << " polyfit with eigen " << "======" << std::endl;

//  auto line_point = prepare_poly_fit_data();

  auto line_point = get_wm_path_points();
  Eigen::VectorXd line;
  auto res = PolyFit::PolyLineFit(line_point, 3, line, true);

  for (auto &coef : line) {
    std::cout << "PolyLineFit coef " << coef << std::endl;
  }

  // polyfit with osqp
  std::cout << "======" << " polyfit with osqp " << "======" << std::endl;

  std::vector<double> coef;
  auto res_coef = PolyFit::PolyLineFitOSQP(line_point, 3, coef);

  // polyfit with spline
  std::cout << "======" << " polyfit with spline " << "======" << std::endl;

  // std::cout << "before PolyFitSplineSmooth" << std::endl;

  std::vector<math_utils::Point2D> smooth_points;
  PolyFit::PolyFitSplineSmooth(line_point, smooth_points);

  // polyfit with spline new
  std::cout << "======" << " polyfit with spline new " << "======" << std::endl;

  std::vector<math_utils::Point2D> smooth_points_new;
  std::vector<double> curva;
  PolyFit::PolyFitSplineSmoothNew(line_point, smooth_points_new,curva);

    // polyfit with spline new piece
    std::cout << "======" << " polyfit with spline new " << "======" << std::endl;

   std::vector<std::vector<math_utils::Point2D>> smooth_points_new_piece;
    std::vector<double> curva_2;
    PolyFit::PolyFitSplineSmoothNewPiece(line_point, smooth_points_new_piece,curva);
  //  for (auto &point: smooth_points_new) {
  //    std::cout << point.x << " " << point.y << std::endl;
  //  }

  // test CSC compress

  // std::vector<std::vector<double>> p = {{4, 1}, {1, 2}};
  // std::vector<double> p_x;
  // std::vector<int> p_i;
  // std::vector<int> p_p;

  // auto res_eigen = PolyFit::compress_to_csc(p, p_x, p_i, p_p);
  // for (int i = 0; i < res; i++) {
  //   std::cout << " ----- px is " << p_x[i] << std::endl;
  //   std::cout << "pi is " << p_i[i] << std::endl;
  // }
  // for (int i = 0; i < 3; i++) {
  //   std::cout << "p_p is " << p_p[i] << std::endl;
  // }

  // std::vector<math_utils::Point2D> point_order_2;
  // point_order_2.emplace_back(1, 3);
  // point_order_2.emplace_back(2, 7);
  // point_order_2.emplace_back(3, 13);
  // point_order_2.emplace_back(4, 21);
  // std::vector<double> x = {0.11726708207420372, 1.005599688095178,
  //                          1.9640316178944133, 3.0565121750392534,
  //                          4.138311722285505};
  // std::vector<double> y = {0.9533210492624835, 1.144448638106922,
  //                          1.4360407508948076, 2.2752559847950127,
  //                          2.7041426887485303};

  // std::vector<math_utils::Point2D> point_order_3;
  // for (int i = 0; i < 5; i++) {
  //   point_order_3.emplace_back(x[i], y[i]);
  // }

  // test njson
  // gen_njson_test();

  return 0;
}

#endif
