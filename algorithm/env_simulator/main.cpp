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
  env_sim.update_case_data("/home/sensetime/data/ws/common_math_v2/common_math/"
                           "algorithm/env_simulator/data/"
                           "freespace_case_1.json");
  return 0;
}