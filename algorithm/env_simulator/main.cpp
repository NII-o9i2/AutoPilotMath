//
// Created by SENSETIME\fengxiaotong on 23-12-8.
//

#include "env_simulator.h"
#include <Eigen/Dense>
#include <iostream>
#include <unsupported/Eigen/Splines>
#include <vector>

using namespace Eigen;
using namespace std;
int main() {
  EnvSim::EnvSimulator env_sim;
  env_sim.update_case_data(
      "/home/SENSETIME/fengxiaotong/ws/xviz/common_math/"
      "algorithm/env_simulator/data/"
      "test_case_1_corner_case.json");
  return 0;
}