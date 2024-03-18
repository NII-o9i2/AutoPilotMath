
#include "vector"
#include "unordered_map"
#include "utils.h"

class Interval {
 public:
  Interval(double _front, double _back) : front_(_front), back_(_back){};

  Interval operator+(const Interval& other) const {
    return {this->front_ + other.front_, this->back_ + other.back_};
  }

  Interval operator-(const Interval& other) const {
    return {this->front_ - other.front_, this->back_ - other.back_};
  }
  Interval operator+(const double& other) const {
    return {this->front_ + other, this->back_ + other};
  }

  Interval operator-(const double& other) const {
    return {this->front_ - other, this->back_ - other};
  }
  double get_front() const { return front_; }
  double get_back() const { return back_; }

 private:
  double front_ = 0.0;
  double back_ = 0.0;
  int id = -1;
};

class Racer {
 public:
  ~Racer() = default;

 private:
  Interval agent_;
  double relative_time_ = 0;
  std::unordered_map<int, MathUtils::Point2D> trajectory_map_;
  bool is_me_ = false;
};

class Track {
 public:
  Track() = default;
  ~Track() = default;

  enum TrackRelativeID {
    LEFT_LEFT = 2,
    LEFT = 1,
    MIDDLE = 0,
    RIGHT = -1,
    RIGHT_RIGHT = -2
  };

 private:
  std::vector<Racer> racer_list_;
  TrackRelativeID relative_id_ = MIDDLE;
};

class TrackSimulator {
 public:
  TrackSimulator() = default;
  ~TrackSimulator() = default;

 private:
  std::vector<Track> track_list_;
};