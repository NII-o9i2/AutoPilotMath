#pragma once

#include <iostream>
#include <vector>
#include <algorithm>

namespace MathUtils {

template <typename T>
class Interval {
 public:
  Interval() = default;

  Interval(const T& low, const T& high) : low_(low), high_(high) {}

  Interval(const Interval& rhs) {
    low_ = rhs.low();
    high_ = rhs.high();
  }

  Interval& operator=(const Interval& rhs) {
    if (this != &rhs) {
      this->low_ = rhs.low();
      this->high_ = rhs.high();
    }
    return *this;
  }

  virtual ~Interval() = default;

  void SetLow(const T& low) { low_ = low; }

  void SetHigh(const T& high) { high_ = high; }

  const T low() const { return low_; }

  const T high() const { return high_; }

 protected:
  T low_;
  T high_;
};

class Range : public Interval<double> {
 public:
  Range() : Interval<double>(0.0, 0.0) {}

  Range(const double low, const double high) : Interval<double>(low, high) {}

  Range(const Range& rhs) : Interval<double>(rhs) {}

  Range& operator=(const Range& input_range) {
    low_ = input_range.low();
    high_ = input_range.high();
    return *this;
  }

  bool operator==(const Range& interval) {
    if ((std::abs(low_ - interval.low()) < min_gap_) &&
        (std::abs(high_ - interval.high()) < min_gap_)) {
      return true;
    }
    return false;
  }

  bool Valid() const { return high_ >= low_; }

  bool InRange(const double value) const {
    return Valid() && value >= low_ && value <= high_;
  }

  bool IsOverlap(const Range& rhs) const {
    double ego_max = std::max(low_, high_);
    double ego_min = std::min(low_, high_);
    double rhs_max = std::max(rhs.low(), rhs.high());
    double rhs_min = std::min(rhs.low(), rhs.high());
    if (ego_max < rhs_min || ego_min > rhs_max) {
      return false;
    } else {
      return true;
    }
  }

  double GetOverlapLen(const Range& rhs) const {
    if (!IsOverlap(rhs)) {
      return 0.0;
    }
    double overlap_len =
        std::min(high_, rhs.high()) - std::max(low_, rhs.low());
    overlap_len = std::max(overlap_len, 0.0);
    return overlap_len;
  }

  bool MergeWith(const Range& input_range, const double min_gap = 0.001) {
    if (low_ > high_) {
      return false;
    }
    if (input_range.high() < input_range.low() || min_gap < 0) {
      return false;
    }

    if ((input_range.low() - high_ > min_gap) ||
        (low_ - input_range.high() > min_gap)) {
      return false;
    }

    low_ = std::min(input_range.low(), low_);
    high_ = std::max(input_range.high(), high_);
    return true;
  }

  std::vector<Range> ExcludeWith(const Range& input_range) {
    std::vector<Range> res;
    if (!Valid()) {
      return res;
    }
    if (!input_range.Valid()) {
      res.emplace_back(*this);
      return res;
    }

    if ((input_range.low() - high_ >= 0) || (low_ - input_range.high() >= 0)) {
      res.emplace_back(*this);
    } else if (high_ > input_range.high() && low_ < input_range.low()) {
      res.emplace_back(Range(input_range.high(), high_));
      res.emplace_back(Range(low_, input_range.low()));
    } else if (high_ > input_range.high() && low_ < input_range.high()) {
      res.emplace_back(Range(input_range.high(), high_));
    } else if (low_ < input_range.low() && input_range.low() < high_) {
      res.emplace_back(Range(low_, input_range.low()));
    }

    return res;
  }

  static std::vector<Range> MergeAll(std::vector<Range> range_list) {
    sort(range_list.begin(), range_list.end(),
         [](const Range& a, const Range& b) -> bool {
           return a.low() > b.low();
         });

    std::vector<Range> merged_list;
    for (const auto& range : range_list) {
      if (!range.Valid()) {
        continue;
      }
      if (!merged_list.empty() && range.low() < merged_list.back().high()) {
        merged_list.back().SetLow(
            std::min(merged_list.back().low(), range.low()));
        merged_list.back().SetHigh(
            std::max(merged_list.back().high(), range.high()));
      } else {
        merged_list.emplace_back(range);
      }
    }
    return merged_list;
  }

 private:
  const double min_gap_ = 0.001;
};

template <class charT, class traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, const Range& range) {
  os << "(" << range.low() << ", " << range.high() << ")";
  return os;
}

}  // namespace MathUtils
