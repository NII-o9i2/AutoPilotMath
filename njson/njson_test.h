//
// Created by 冯晓彤 on 2023/5/25.
//
#ifndef BENCH_TEST_NJSON_TEST_H
#define BENCH_TEST_NJSON_TEST_H

#include "vector"
#include "utils.h"
#include "nlohmann/json.hpp"

namespace math_utils {
  class address {
  private:
    std::string street;
    int housenumber;
    int postcode;

  public:
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(address, street, housenumber, postcode)
  };

  class Line {
  public:
    Line() = default;

    Line(int id, std::vector<math_utils::Point2D> line) : line_id_(id), line_(line) {};

    ~Line() = default;

//private:
    std::vector<Point2D> line_;
    int line_id_;
  public:
//  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Line, line_, line_id_)
  };

  class LineManager {
  public:
    LineManager() = default;

    LineManager(Line left, Line right) : left_line_(left), right_line_(right) {};

    ~LineManager() = default;

  private:
    Line right_line_;
    Line left_line_;
  public:
//  NLOHMANN_DEFINE_TYPE_INTRUSIVE(LineManager, right_line_, left_line_)
  };


  struct LineD {
    std::vector<Point2D> line_;
    int line_id_;
  };

  struct LineDebug {
    LineD right_line_;
    LineD left_line_;
  };

  NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point2D, x, y)

  NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LineD, line_, line_id_)

  NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LineDebug, right_line_, left_line_)
}
void gen_njson_test(void);
#endif //BENCH_TEST_NJSON_TEST_H
