//
// Created by SENSETIME\fengxiaotong on 24-8-8.
//
#include "iostream"
#include "data_preprocess/train_data_type.h"
#include "data_preprocess/agent_data_preprocess.h"

int main(){
  auto ts_a = DLP::zeros<int>(2);
  auto ts_b = DLP::zeros<int>(2,3);
  auto ts_c = DLP::zeros<int>(2,3,4);

  DLP::AgentDataParam param;
  param.history_num = 5;
  DLP::AgentInfo agent_info(param);

  std::vector<DLP::AgentFrameInfo> tmp;
  for (int i = 0; i <10; i++){
    DLP::AgentFrameInfo info;
    info.rel_time_step = 0;
    info.position_x = i;
    info.position_y = i;
    tmp.clear();
    tmp.emplace_back(info);
    auto exist = agent_info.update_and_check_exist(tmp);
    std::cout << " frame "<< i << " "<<exist << std::endl;
    for (const auto& info : agent_info.get_historical_info()){
      std::cout <<  "      time " << info.rel_time_step <<" x " << info.position_x << " y " <<info.position_y<< std::endl;
    }
  }

  agent_info.reset();
  for (int i = 0; i <10; i++){
    DLP::AgentFrameInfo info;
    info.rel_time_step = 0;
    info.position_x = i;
    info.position_y = i;
    tmp.clear();
    if (i % 2 == 0){
      tmp.emplace_back(info);
    }else{

    }
    auto exist = agent_info.update_and_check_exist(tmp);
    std::cout << " frame "<< i << " "<<exist << std::endl;
    for (const auto& info : agent_info.get_historical_info()){
      std::cout <<  "      time " << info.rel_time_step <<" x " << info.position_x << " y " <<info.position_y<< std::endl;
    }
  }

  auto ts_cat = DLP::zeros<int>(4,3);
  auto ts_cat_1 = DLP::zeros<int>(2,3);

  ts_cat.data()[0][0] = 1;
  ts_cat.data()[0][1] = 2;
  ts_cat.data()[0][2] = 3;
  ts_cat.data()[1][0] = 4;
  ts_cat.data()[1][1] = 5;
  ts_cat.data()[1][2] = 6;
  ts_cat.data()[2][0] = 1;
  ts_cat.data()[2][1] = 2;
  ts_cat.data()[2][2] = 3;
  ts_cat.data()[3][0] = 4;
  ts_cat.data()[3][1] = 5;
  ts_cat.data()[3][2] = 6;

  ts_cat_1.data()[0][0] = 7;
  ts_cat_1.data()[0][1] = 8;
  ts_cat_1.data()[0][2] = 9;
  ts_cat_1.data()[1][0] = 10;
  ts_cat_1.data()[1][1] = 11;
  ts_cat_1.data()[1][2] = 12;

  auto ts_cat_d0 = ts_cat.cat(ts_cat_1,0);
//  auto ts_cat_d1 = ts_cat.cat(ts_cat_1,1);
  ts_cat.cover(2,0,ts_cat_1);
  return 0;
};
