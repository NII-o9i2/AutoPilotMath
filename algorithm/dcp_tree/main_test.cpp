//
// Created by SENSETIME\fengxiaotong on 23-11-27.
//
#include "iostream"
#include "track_simulator.h"
int main(){
  Interval first = {1.0,3.0};
  Interval second = {3.0,5.0};

  auto add = first + second;
  auto sub = first - second;

  std::cout<< "add front: "<< add.get_front() << " back: " << add.get_back();
  std::cout<< "sub front: "<< sub.get_front() << " back: " << sub.get_back() << std::endl;

  add = first + 1.0;
  sub = first - 1.0;

  std::cout<< "add front: "<< add.get_front() << " back: " << add.get_back();
  std::cout<< "sub front: "<< sub.get_front() << " back: " << sub.get_back() << std::endl;
}