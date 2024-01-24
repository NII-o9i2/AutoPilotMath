//
// Created by 冯晓彤 on 2023/5/8.
//
#include "polyfit_osqp.h"
#include "osqp.h"
#include "iostream"
#include "Eigen/SparseCore"

extern "C" {
#include "float.h"

//#define c_float float
//#define c_int int

int OSQP_Test(void) {
  // Load problem data
  c_float P_x[3] = {4.0, 1.0, 2.0,};
  c_int P_nnz = 3;
  c_int P_i[3] = {0, 0, 1,};
  c_int P_p[3] = {0, 1, 3,};
  c_float q[2] = {1.0, 1.0,};
  c_float A_x[4] = {1.0, 1.0, 1.0, 1.0,};
  c_int A_nnz = 4;
  c_int A_i[4] = {0, 1, 0, 2,};
  c_int A_p[3] = {0, 2, 4,};
  c_float l[3] = {1.0, 0.0, 0.0,};
  c_float u[3] = {1.0, 0.7, 0.7,};
  c_int n = 2;
  c_int m = 3;

  // Exitflag
  c_int exitflag = 0;

  // Workspace structures
  OSQPWorkspace *work;
  OSQPSettings *settings = (OSQPSettings *) c_malloc(sizeof(OSQPSettings));
  OSQPData *data = (OSQPData *) c_malloc(sizeof(OSQPData));

  // Populate data
  if (data) {
    data->n = n;
    data->m = m;
    data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
    data->q = q;
    data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
    data->l = l;
    data->u = u;
  }

  // Define solver settings as default
  if (settings) {
    osqp_set_default_settings(settings);
    settings->alpha = 1.0; // Change alpha parameter
  }

  // Setup workspace
  exitflag = osqp_setup(&work, data, settings);

  // Solve Problem
  osqp_solve(work);
//  for (int i= 0; i<n;i++){
//    printf(" solution %f",work->solution->x[i]);
//  }
  // Cleanup
  osqp_cleanup(work);
  if (data) {
    if (data->A) c_free(data->A);
    if (data->P) c_free(data->P);
    c_free(data);
  }
  if (settings) c_free(settings);

  return 0;
}

}
using namespace std;
#define EPSILON_DOUBLE 0.000001
namespace PolyFit {

  int compress_to_csc(vector <vector<double>> &p, vector<double> &p_x, vector<int> &p_i, vector<int> &p_p) {
    int nzmax = 0;
    int i, j, k;

    if (p.empty()) { return 0; }
    int n_rows = p.size();
    int n_cols = p[0].size();

    // 计算稀疏矩阵中非零元素的数量 nzmax
    for (j = 0; j < n_cols; j++) {
      for (i = 0; i <= j; i++) {
        if ((abs(p[i][j])) > EPSILON_DOUBLE) {
          nzmax++;
        }
      }
    }

    // 分配存储压缩后稀疏矩阵的数组空间
    p_x.reserve(nzmax);
    p_i.reserve(nzmax);
    p_p.reserve(n_cols + 1);

    // 将二维数组压缩为 CSC 格式
    k = 0;
    for (j = 0; j < n_cols; j++) {
      p_p[j] = k;
      for (i = 0; i <= j; i++) {
        if (p[i][j] != 0) {
          p_x[k] = p[i][j];
          p_i[k] = i;
          k++;
        }
      }
    }
    p_p[n_cols] = k;
    return nzmax;
  }

  bool PolyLineFitOSQP(const std::vector<math_utils::Point2D> &points,
                       const size_t order, std::vector<double> &coef) {
    if (order < 1) {
      return false;
    }

    const size_t point_size = points.size();
    vector<vector<double>> x_i_pow(point_size, vector<double>(order+1,0.0));

// calculate x_i^k list
    int counter = 0;
    for (auto &point: points) {
      double tmp = 1.0;
      for (int order_tmp = 0;
           order_tmp < order + 1; order_tmp++) {
        if (order_tmp < 1) {
          x_i_pow[counter][order_tmp] = 1.0;
        } else {
          x_i_pow[counter][order_tmp] = x_i_pow[counter][order_tmp - 1] * point.x;
        }
      }
      counter++;
    }

    // calculate P matrix
    vector<vector<double>> P(order+1, vector<double>(order+1,0));
    // p[i,j] = sum(x_k^(i) * x_k^(j))  0<k<n
    for (int i = 0; i < order + 1; i++) {
      for (int j = 0; j < order + 1; j++) {
        double sum_tmp = 0.0;
        for (int k = 0; k < point_size; k++) {
          sum_tmp += x_i_pow[k][i] * x_i_pow[k][j];
        }
        P[i][j] = sum_tmp * 2.0;
      }
    }

    // calculate Q matrix
    c_float q[order +1];
    // q[i] = sum(y_k * x_k^i)  0<k<n
    for (int i = 0; i < order + 1; i++) {
      double sum_tmp = 0.0;
      for (int k = 0; k < point_size; k++) {
        sum_tmp += -2 * points[k].y * x_i_pow[k][i];
      }
      q[i] = sum_tmp;
    }

    int exitflag = 0;

//    auto res = PolyFit::compress_to_csc(P, p_x, p_i, p_p);
    int n_rows = P.size();
    if (P.empty()){
      return false;
    }
    int n_cols = P[0].size();
    int nzmax = 0;

    for (int j = 0; j < n_cols; j++) {
      for (int i = 0; i <= j; i++) {
        if ((abs(P[i][j])) > EPSILON_DOUBLE) {
          nzmax++;
        }
      }
    }

    c_float p_x[nzmax];
    c_int p_i[nzmax];
    c_int p_p[n_cols+1];

    int k = 0;
//    cout << "n_cols is " << n_cols <<endl;
    for (int j = 0; j < n_cols; j++) {
      p_p[j] = k;
      for (int i = 0; i <= j; i++) {
        if (P[i][j] != 0) {
          p_x[k] = P[i][j];
          p_i[k] = i;
          k++;
        }
      }
    }
    p_p[n_cols] = k;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings *settings = (OSQPSettings *) c_malloc(sizeof(OSQPSettings));
    OSQPData *data = (OSQPData *) c_malloc(sizeof(OSQPData));

    c_int A_nnz = 1;
    c_float A_x[] = {1};
    c_int A_i[] = {0};
    c_int A_p[] = {0,0,0,0,1};
    c_float l[] = {-100000};
    c_float u[] = {100000};
    // Populate data
    if (data) {
      data->n = order + 1;
      data->m = 1;
      data->P = csc_matrix(data->n, data->n, nzmax, p_x, p_i, p_p);
      data->q = q;
      data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
      data->l = l;
      data->u = u;
    }
    // Define solver settings as default
    if (settings) {
      osqp_set_default_settings(settings);
      settings->verbose = false;
      settings->max_iter = 25;
      settings->time_limit = 0.0001;
      settings->alpha = 1.0; // Change alpha parameter
    }

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);

    coef.reserve(order+1);
    for (int i= 0; i<order + 1;i++){
      coef.emplace_back(work->solution->x[i]);
    }
    // Cleanup
    osqp_cleanup(work);
    if (data) {
      if (data->A) c_free(data->A);
      if (data->P) c_free(data->P);
      c_free(data);
    }
    if (settings) c_free(settings);

    return true;
  }
}

