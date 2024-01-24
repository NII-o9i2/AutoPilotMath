/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * dingjie <dingjie1@senseauto.com>
 */

#include "spline_smoothing/include/math/angle.hpp"
#include "spline_smoothing/include/math/sin_table.hpp"

namespace PolyFit {
namespace math {

float sin(Angle16 a) {
  int16_t idx = a.raw();

  if (idx < -Angle16::RAW_PI_2) {
    idx = static_cast<int16_t>(idx + Angle16::RAW_PI);
    return -SIN_TABLE[idx % SIN_TABLE_SIZE];
  }
  if (idx < 0) {
    return -SIN_TABLE[(-idx) % SIN_TABLE_SIZE];
  }
  if (idx < Angle16::RAW_PI_2) {
    return SIN_TABLE[idx % SIN_TABLE_SIZE];
  }
  idx = static_cast<int16_t>(Angle16::RAW_PI - idx);
  return SIN_TABLE[idx % SIN_TABLE_SIZE];
}

float cos(Angle16 a) {
  Angle16 b(static_cast<int16_t>(Angle16::RAW_PI_2 - a.raw()));
  return sin(b);
}

float tan(Angle16 a) { return sin(a) / cos(a); }

float sin(Angle8 a) {
  Angle16 b(static_cast<int16_t>(a.raw() << 8));
  return sin(b);
}

float cos(Angle8 a) {
  Angle16 b(static_cast<int16_t>(a.raw() << 8));
  return cos(b);
}

float tan(Angle8 a) {
  Angle16 b(static_cast<int16_t>(a.raw() << 8));
  return tan(b);
}

} // namespace math
} // namespace PolyFit
