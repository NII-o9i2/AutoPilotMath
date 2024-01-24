//
// Created by 冯晓彤 on 2023/5/25.
//

#include "njson_test.h"
#include <iostream>
void gen_njson_test(void){
  float left_x = 1;
  float right_x = 2;
  std::vector<math_utils::Point2D> left_points;
  std::vector<math_utils::Point2D> right_points;
  for (int i = 0; i< 100;i++){
    if (i %2 ==0){
      left_points.emplace_back(left_x,i);
    }else{
      right_points.emplace_back(right_x,i);
    }
  }

  math_utils::Line left_line(1,left_points);
  math_utils::Line right_line(2,right_points);

  math_utils::LineManager manager(left_line,right_line);

  math_utils::LineDebug line_debug;
  for (int i = 0; i< 10;i++) {
    if (i % 2 == 0) {
      line_debug.left_line_.line_.emplace_back(left_x, i);
    } else {
      line_debug.right_line_.line_.emplace_back(right_x, i);
    }
  }
  line_debug.left_line_.line_id_ = 2;
  line_debug.right_line_.line_id_ = 3;
  nlohmann::json json_debug = line_debug;

  std::cout << json_debug.dump() << std::endl;

  return;
}