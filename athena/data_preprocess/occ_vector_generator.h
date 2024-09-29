#pragma once

#include <string>
#include <vector>

namespace DLP {

constexpr int kOccVoxelTypeUnknown = 0;
constexpr int kOccVoxelTypeGround = 1;
constexpr int kOccVoxelTypeObs = 2;

struct OccPartitionInfo {
  int num_partition;
  std::vector<double> ranges;
  std::vector<double> steps;
  std::vector<int> num_voxels_per_range;
};

struct OccVoxel {
  int i;
  int j;
  int v;
};

struct RawOccInfo {
  OccPartitionInfo x_partition;
  OccPartitionInfo y_partition;
  std::vector<OccVoxel> voxel_list;
};

struct OccFreeSpaceVector {
  std::vector<std::pair<int, int>> free_space_edge;
};

class OccVectorGenerator {
public:
  OccVectorGenerator() = default;
  ~OccVectorGenerator() = default;

  std::vector<OccFreeSpaceVector> build_occ_vector(RawOccInfo &raw_occ_info);

private:
  const double voxel_cluster_dist = 2.0;
};
} // namespace DLP