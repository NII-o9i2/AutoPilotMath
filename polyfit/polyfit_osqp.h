//
// Created by 冯晓彤 on 2023/5/8.
//

#ifndef BENCH_TEST_POLYFIT_OSQP_H
#define BENCH_TEST_POLYFIT_OSQP_H
#include "common/utils.h"
#include "vector"

using namespace std;
extern "C"{
  int OSQP_Test(void);
}
namespace PolyFit {
  int compress_to_csc(vector<vector<double>> &p, vector<double> &p_x, vector<int> &p_i, vector<int> &p_p);
  bool PolyLineFitOSQP(const std::vector <math_utils::Point2D> &points,
                   const size_t order, std::vector<double> &coef);

}

#endif //BENCH_TEST_POLYFIT_OSQP_H
