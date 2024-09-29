#include "occ_vector_generator.h"
#include <cmath>
#include <deque>
#include <iostream>
#include <tuple>

namespace DLP {

bool is_point_in_edge(std::vector<std::vector<int>> &image,
                      std::pair<int, int> &point) {
  int length = image.size();
  int width = image[0].size();
  int i = point.first, j = point.second;

  if (i < 0 || i >= length || j < 0 || j >= width) {
    return false;
  }
  if (image[i][j] != kOccVoxelTypeObs) {
    return false;
  }
  if (i == 0 || i == length - 1 || j == 0 || j == width - 1) {
    return true;
  }
  if ((i - 1 >= 0 && (image[i - 1][j] == kOccVoxelTypeUnknown ||
                      image[i - 1][j] == kOccVoxelTypeGround)) ||
      (i + 1 < length && (image[i + 1][j] == kOccVoxelTypeUnknown ||
                          image[i + 1][j] == kOccVoxelTypeGround)) ||
      (j - 1 >= 0 && (image[i][j - 1] == kOccVoxelTypeUnknown ||
                      image[i][j - 1] == kOccVoxelTypeGround)) ||
      (j + 1 < width && (image[i][j + 1] == kOccVoxelTypeUnknown ||
                         image[i][j + 1] == kOccVoxelTypeGround))) {
    return true;
  }

  return false;
}

std::vector<std::pair<int, int>>
detect_edges(std::vector<std::vector<int>> &image, std::pair<int, int> &center,
             int num_bins) {
  int length = image.size();
  int width = image[0].size();
  int center_x = center.first, center_y = center.second;

  // tuple_element = <valid, i, j, dist>
  std::vector<std::tuple<int, int, int, double>> bins(
      num_bins, std::make_tuple(0, 0, 0, 0.0));
  double angle_step = M_PI / num_bins;

  auto update_bins = [&](int bin_index, std::pair<int, int> &point,
                         double distance) {
    if (bin_index < 0 || bin_index >= num_bins)
      return;
    if (std::get<0>(bins[bin_index]) == 0 ||
        std::get<3>(bins[bin_index]) > distance) {
      bins[bin_index] = std::make_tuple(1, point.first, point.second, distance);
    }
  };

  std::vector<std::pair<int, int>> boundary_points;
  for (int i = 0; i < length; ++i) {
    for (int j = 0; j < width; ++j) {
      std::pair<int, int> point = {i, j};
      if (is_point_in_edge(image, point)) {
        boundary_points.emplace_back(point);

        int dx = i - center_x;
        int dy = j - center_y;
        double angle = std::atan2(dy, dx);
        double distance = dx * dx + dy * dy;
        int bin_index = static_cast<int>((angle + M_PI / 2) / angle_step);

        if ((0 <= bin_index && bin_index < num_bins * 0.3) ||
            (num_bins * 0.7 <= bin_index && bin_index < num_bins)) {
          for (int offset = -3; offset <= 3; ++offset) {
            update_bins(bin_index + offset, point, distance);
          }
          continue;
        }

        if ((num_bins * 0.3 <= bin_index && bin_index < num_bins * 0.4) ||
            (num_bins * 0.6 <= bin_index && bin_index < num_bins * 0.7)) {
          for (int offset = -2; offset <= 2; ++offset) {
            update_bins(bin_index + offset, point, distance);
          }
          continue;
        }

        update_bins(bin_index, point, distance);
        update_bins(bin_index - 1, point, distance);
        update_bins(bin_index + 1, point, distance);
      }
    }
  }

  std::vector<std::pair<int, int>> edge;
  for (int i = 0; i < num_bins; ++i) {
    auto &bin = bins[i];
    if (std::get<0>(bin) != 0) {
      auto edge_point = std::make_pair(std::get<1>(bin), std::get<2>(bin));
      if (edge.empty() || edge.back().first != edge_point.first ||
          edge.back().second != edge_point.second) {
        edge.emplace_back(edge_point);
      }
    }
  }

  return edge;
}

std::vector<std::pair<int, int>>
detect_edges_v2(std::vector<std::vector<int>> &image, std::pair<int, int> &center,
                int num_rays) {
  int length = image.size();
  int width = image[0].size();
  int center_x = center.first;
  int center_y = center.second;

  std::vector<double> angles;
  double angle_step = M_PI / num_rays;

  for (int i = -num_rays / 2; i < 0; ++i) {
    angles.emplace_back(i * angle_step);
  }
  for (int i = 0; i < num_rays / 2; ++i) {
    angles.emplace_back(i * angle_step);
  }

  std::vector<std::pair<int, int>> edges;

  for (double angle : angles) {
    double x = center_x, y = center_y;
    double dx = std::cos(angle), dy = std::sin(angle);

    while (0 <= x && x < length && 0 <= y && y < width) {
      if (image[static_cast<int>(x)][static_cast<int>(y)] == kOccVoxelTypeObs) {
        edges.emplace_back(static_cast<int>(x), static_cast<int>(y));
        break;
      }
      x += dx;
      y += dy;
    }
  }

  return edges;
}

std::vector<std::vector<int>>
find_connected_components_bfs(std::vector<std::vector<int>> &image,
                              int neighbour_check_num) {
  int length = image.size();
  int width = image[0].size();
  std::vector<std::vector<uint8_t>> visited(length,
                                            std::vector<uint8_t>(width, 0));
  std::vector<std::vector<std::pair<int, int>>> components;

  auto is_valid = [&](int x, int y, int nx, int ny) {
    return (0 <= nx && nx < length && 0 <= ny && ny < width &&
            !visited[nx][ny] && image[nx][ny] == kOccVoxelTypeObs &&
            abs(nx - x) + abs(ny - y) <= neighbour_check_num);
  };

  auto bfs = [&](int x, int y) {
    std::deque<std::pair<int, int>> queue;
    queue.emplace_back(std::make_pair(x, y));
    std::vector<std::pair<int, int>> component;
    while (!queue.empty()) {
      auto pt = queue.front();
      auto &cx = pt.first;
      auto &cy = pt.second;
      queue.pop_front();
      if (visited[cx][cy]) {
        continue;
      }
      visited[cx][cy] = 1;
      component.emplace_back(pt);
      for (int dx = -neighbour_check_num; dx <= neighbour_check_num; ++dx) {
        for (int dy = -neighbour_check_num; dy <= neighbour_check_num; ++dy) {
          int nx = cx + dx, ny = cy + dy;
          if (is_valid(cx, cy, nx, ny)) {
            queue.emplace_back(std::make_pair(nx, ny));
          }
        }
      }
    }
    return component;
  };

  for (int i = 0; i < length; ++i) {
    for (int j = 0; j < width; ++j) {
      if (image[i][j] == kOccVoxelTypeObs && !visited[i][j]) {
        std::vector<std::pair<int, int>> component = bfs(i, j);
        components.emplace_back(component);
      }
    }
  }

  int components_num = components.size();
  std::vector<std::vector<int>> image_component(
      length, std::vector<int>(width, components_num));
  for (int i = 0; i < components_num; ++i) {
    auto &component = components[i];
    for (auto &pt : component) {
      image_component[pt.first][pt.second] = i;
    }
  }

  return image_component;
}

std::vector<OccFreeSpaceVector>
OccVectorGenerator::build_occ_vector(RawOccInfo &raw_occ_info) {
  std::vector<OccFreeSpaceVector> res;

  int length = 0, width = 0;
  for (auto &num : raw_occ_info.x_partition.num_voxels_per_range) {
    length += num;
  }
  for (auto &num : raw_occ_info.y_partition.num_voxels_per_range) {
    width += num;
  }

  std::vector<std::vector<int>> image(length, std::vector<int>(width, 0));
  for (const auto &v : raw_occ_info.voxel_list) {
    int x = v.i;
    int y = v.j;
    image[x][y] = v.v;
  }
  std::pair<int, int> ego_center = {0, width / 2};

  int bins_num = 100;
  auto all_edge = detect_edges(image, ego_center, bins_num);
  // for (auto &pt : all_edge) {
  //   std::cout << "i " << pt.first << " j " << pt.second << std::endl;
  // }

  if (raw_occ_info.x_partition.steps.empty()) {
    std::cout << "invalid x_partition steps" << std::endl;
    return res;
  }
  auto &voxel_step = raw_occ_info.x_partition.steps[0];
  int neighbour_check_num = std::ceil(voxel_cluster_dist / voxel_step);
  auto image_component =
      find_connected_components_bfs(image, neighbour_check_num);

  OccFreeSpaceVector temp_edge;
  for (auto &pt : all_edge) {
    if (!temp_edge.free_space_edge.empty()) {
      auto &last_pt = temp_edge.free_space_edge.back();
      auto &last_c = image_component[last_pt.first][last_pt.second];
      auto &cur_c = image_component[pt.first][pt.second];
      if (last_c != cur_c) {
        res.emplace_back(temp_edge);
        temp_edge.free_space_edge.clear();
      }
    }
    temp_edge.free_space_edge.emplace_back(pt);
  }
  if (!temp_edge.free_space_edge.empty()) {
    res.emplace_back(temp_edge);
  }

  // for (auto &edge : res) {
  //   for (auto &pt : edge.free_space_edge) {
  //     std::cout << "i " << pt.first << " j " << pt.second << " class "
  //               << image_component[pt.first][pt.second] << std::endl;
  //   }
  // }
  return res;
}
} // namespace DLP