#include "track_simulator.h"
#include "utils.h"

enum DCPAction{
  LC_Normal = 0,
  LC_Aggressive = 1,
  LK = 2,
};

class DCPTreeNode{
 public:
  DCPTreeNode() = default;
  ~DCPTreeNode() = default;

 private:
  TrackSimulator simulator_;
  DCPAction ongoing_action_ = DCPAction::LK;
  int current_time_ = 0;
  MathUtils::Point2D ego_position;
};

struct ObstacleTrajectory{
  double relative_time = 0.0;
  MathUtils::Point2D position;
};

struct DCPTreeInput{
  std::unordered_map<int,ObstacleTrajectory> obstacle_list;
  MathUtils::Point2D ego_position;
  DCPAction current_action = DCPAction::LK;
};

class DCPTree
{
public:
  DCPTree() = default;
  ~DCPTree() = default;


  void search();
  void add_ongoing_action();
private:
  int tree_height_ = 5;

};

