#include "a_star/a_star_search.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

using std::cout;
using std::endl;

// 定义2D平面上的8个邻居（以及中间的(0,0)，共9个，但在代码中未使用(0,0)）
static std::vector<Eigen::Vector2i> kNeighbors = std::vector<Eigen::Vector2i>{
    Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(-1, 1),
    Eigen::Vector2i(0, -1),  Eigen::Vector2i(0, 1),  Eigen::Vector2i(1, -1),
    Eigen::Vector2i(1, 0),   Eigen::Vector2i(1, 1),
};

// 初始化A*算法所需的数据结构和参数
void Astar::Init(const double cost_threshold, const int num_layers,
                 const double resolution,  const double step_cost_weight, const Eigen::MatrixXd& cost_map,
                 const Eigen::MatrixXd& height_map,
                 const Eigen::MatrixXd& ele_map) {
  auto t0 = std::chrono::high_resolution_clock::now(); // 记录开始时间
  cost_threshold_ = cost_threshold;
  step_cost_weight_  = step_cost_weight;

  // 设置地图尺寸
  max_x_ = cost_map.cols();
  max_y_ = cost_map.rows() / num_layers;
  max_layers_ = num_layers;
  xy_size_ = max_x_ * max_y_;

  // 根据输入的地图数据，构建内部的`grid_map_`
  int row_offset = 0;
  grid_map_.resize(max_layers_);
  for (size_t i = 0; i < max_layers_; ++i) {
    row_offset = i * max_y_;
    grid_map_[i].resize(max_y_);
    for (size_t j = 0; j < max_y_; ++j) {
      grid_map_[i][j].resize(max_x_);
      for (size_t k = 0; k < max_x_; ++k) {
        // 从输入的Eigen矩阵中提取数据并填充到Node对象
        double height = height_map(j + row_offset, k);
        double z = static_cast<int>(height / resolution); // z是高度的离散化表示
        grid_map_[i][j][k] = Node(Eigen::Vector3i(z, j, k), nullptr);
        grid_map_[i][j][k].cost = cost_map(j + row_offset, k);
        grid_map_[i][j][k].height = height;
        grid_map_[i][j][k].ele = ele_map(j + row_offset, k); // ele可能代表gateway信息
        grid_map_[i][j][k].layer = i;
      }
    }
  }
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0); // 计算耗时

  // 初始化搜索层的偏移量，用于多层搜索
  search_layers_offset_.clear();
  search_layers_offset_.emplace_back(0); // 搜索当前层
  for (int i = 0; i < search_layer_depth_; ++i) {
    search_layers_offset_.emplace_back(-(i + 1)); // 搜索下层
    search_layers_offset_.emplace_back(i + 1);   // 搜索上层
  }

  printf(
      "Astar initialized, max_x: %d, max_y: %d, max_layers: %d, time elapsed: "
      "%f ms\n",
      max_x_, max_y_, max_layers_, duration.count() / 1000.0);
}

// 重置grid_map中所有节点的状态
void Astar::Reset() {
  for (size_t i = 0; i < grid_map_.size(); ++i) {
    for (size_t j = 0; j < grid_map_[i].size(); ++j) {
      for (size_t k = 0; k < grid_map_[i][j].size(); ++k) {
        grid_map_[i][j][k].Reset();
      }
    }
  }
}

// 将3D索引转换为唯一的整数哈希值
int Astar::GetHash(const Eigen::Vector3i& idx) const {
  return idx[0] * 10000000 + idx[1] * max_x_ + idx[2];
}

// A*搜索主函数
bool Astar::Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal) {
  auto t0 = std::chrono::high_resolution_clock::now();

  // 如果已有搜索结果，先重置状态
  if (!search_result_.empty()) {
    Reset();
    search_result_.clear();
  }

  // 获取起点和终点节点指针
  auto start_node = &grid_map_[start[0]][start[2]][start[1]]; // 注意索引顺序：layer, col, row
  auto goal_node = &grid_map_[goal[0]][goal[2]][goal[1]];
  start_node->g = 0.0; // 起点的g值为0

  // 检查终点是否可达
  if (goal_node->cost > cost_threshold_) {
    printf("goal node is not reachable, cost: %f", goal_node->cost);
    return false;
  }

  // open_set使用优先队列实现，f值小的节点优先
  std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_set;
  // closed_set使用哈希表实现，用于快速查找
  std::unordered_map<int, Node*> closed_set;

  open_set.push(start_node);

  printf("start searching\n");

  while (!open_set.empty()) {
    // 取出f值最小的节点
    Node* current_node = open_set.top();
    open_set.pop();

    // 如果是终点，则回溯路径并返回成功
    if (current_node->idx == goal_node->idx) {
      while (current_node->parent != nullptr) {
        search_result_.emplace_back(current_node);
        current_node = current_node->parent;
      }
      std::reverse(search_result_.begin(), search_result_.end()); // 翻转路径
      if (debug_) ConvertClosedSetToMatrix(closed_set); // 调试模式下保存访问过的节点
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now() - t0);
      printf("path found, time elapsed: %f ms\n",
             duration.count() / 1000.0);
      return true;
    }

    // 将当前节点加入closed_set
    closed_set[GetHash(current_node->idx)] = current_node;

    // 决定在哪个层上扩展邻居节点
    int layer = DecideLayer(current_node);

    int i, j = 0;
    double tentative_g = 0.0;
    // 遍历所有邻居
    for (const auto& neighbor : kNeighbors) {
      i = current_node->idx[1] + neighbor[0]; // row
      j = current_node->idx[2] + neighbor[1]; // col

      // 检查邻居是否越界
      if (i < 0 || i >= max_y_ || j < 0 || j >= max_x_) {
        continue;
      }

      auto neighbor_node = &grid_map_[layer][i][j];

      // 检查邻居是否是障碍物
      if (neighbor_node->cost > cost_threshold_) {
        // 如果是gateway，并且高度差不大，则可能允许通过
        if (abs(neighbor_node->ele) < 0.5) {
          continue;
        } else {
          if (std::abs(neighbor_node->height - current_node->height) > 0.3) {
            continue;
          }
        }
      }

      // 计算到邻居节点的g值
      auto diff = neighbor_node->idx - current_node->idx;
      double step_cost = step_cost_weight_ * neighbor_node->cost;
      if (step_cost < 5) step_cost = 0.0; // 忽略较小的成本
      tentative_g =
          current_node->g +
          std::sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]) +
          step_cost;

      // 如果邻居已在closed_set中且新的g值不更优，则跳过
      auto p_neighbor = closed_set.find(GetHash(neighbor_node->idx));
      if (p_neighbor != closed_set.end()) {
        if (tentative_g >= p_neighbor->second->g) {
          continue;
        }
      }

      // 如果通过当前节点到达邻居的路径更优
      if (tentative_g < neighbor_node->g) {
        neighbor_node->g = tentative_g;
        neighbor_node->f = tentative_g + GetHeuristic(neighbor_node, goal_node);
        neighbor_node->parent = current_node;
        open_set.push(neighbor_node);
      }
    }
  }

  // open_set为空，未找到路径
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0);
  printf("path not found\n, time elapsed: %f ms\n",
         duration.count() / 1000.0);
  if (debug_) {
    ConvertClosedSetToMatrix(closed_set);
  }
  return false;
}

// 根据当前节点及其周围环境决定应该在哪一层进行搜索
int Astar::DecideLayer(const Node* cur_node) const {
  int layer = cur_node->layer;
  int i = cur_node->idx[1];
  int j = cur_node->idx[2];
  double cur_height = cur_node->height;

  int true_layer = layer;

  // 遍历预设的层偏移量（如0, -1, 1）
  for (const auto offset : search_layers_offset_) {
    int cur_layer = layer + offset;

    if (cur_layer < 0 || cur_layer >= max_layers_) {
      continue;
    }

    const Node& search_node = grid_map_[cur_layer][i][j];

    // 如果高度差太大，则不考虑该层
    if (abs(search_node.height - cur_height) > 0.2) {
      continue;
    }

    // 根据ele(gateway)信息判断是否需要切换层
    if (search_node.ele > 0.5) { // 向上
      true_layer = std::min(cur_layer + 1, max_layers_ - 1);
      break;
    } else if (search_node.ele < -0.5) { // 向下
      true_layer = std::max(cur_layer - 1, 0);
      break;
    }
  }

  return true_layer;
}

// 计算两个节点间的移动成本（未实现）
double Astar::CalculateStepCost(const Node* node1, const Node* node2) const {}

// 计算启发式函数h(n)的值
double Astar::GetHeuristic(const Node* node1, const Node* node2) const {
  double cost = 0.0;

  if (h_type_ == kEuclidean) {
    // 欧几里得距离
    cost = (node1->idx - node2->idx).norm();
  } else if (h_type_ == kDiagonal) {
    // 对角距离（Octile distance）
    Eigen::Vector3i d = node1->idx - node2->idx;
    int dx = abs(d(0)), dy = abs(d(1)), dz = abs(d(2));
    int dmin = std::min(dx, std::min(dy, dz));
    int dmax = std::max(dx, std::max(dy, dz));
    int dmid = dx + dy + dz - dmin - dmax;
    double h =
        std::sqrt(3) * dmin + std::sqrt(2) * (dmid - dmin) + (dmax - dmid);
    cost = h;
  } else if (h_type_ == kManhattan) {
    // 曼哈顿距离
    cost = (node1->idx - node2->idx).lpNorm<1>();
  } else {
    assert(false && "not implemented");
  }
  return cost;
}

// 将搜索结果路径转换为PathPoint向量
std::vector<PathPoint> Astar::GetPathPoints() const {
  std::vector<PathPoint> path_points;

  auto size = search_result_.size();
  path_points.resize(size);

  if (size == 0) {
    printf("path is empty\n, convert to path points failed\n");
    return path_points;
  }

  for (size_t i = 0; i < size; ++i) {
    path_points[i].layer = search_result_[i]->layer;
    path_points[i].x = search_result_[i]->idx(2);
    path_points[i].y = search_result_[i]->idx(1);
    path_points[i].height = search_result_[i]->height;
    // 计算航向角
    if (i > 0) {
      path_points[i].heading =
          std::atan2(search_result_[i]->idx(1) - search_result_[i - 1]->idx(1),
                     search_result_[i]->idx(2) - search_result_[i - 1]->idx(2));
    }
  }

  // 设置起点的航向角
  if (size > 1) {
    path_points[0].heading = path_points[1].heading;
  }

  return path_points;
}

// 将搜索结果路径转换为Eigen矩阵
Eigen::MatrixXd Astar::GetResultMatrix() const {
  if (search_result_.empty()) {
    printf("path is empty\n, convert to matrix failed\n");
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd path_matrix(search_result_.size(), 3);
  for (size_t i = 0; i < search_result_.size(); ++i) {
    path_matrix(i, 0) = search_result_[i]->layer;
    path_matrix(i, 1) = search_result_[i]->idx[1];
    path_matrix(i, 2) = search_result_[i]->idx[2];
  }
  return path_matrix;
}

// 将closed_set转换为矩阵，用于调试
void Astar::ConvertClosedSetToMatrix(
    const std::unordered_map<int, Node*>& closed_set) {
  visited_set_ = Eigen::MatrixXi(closed_set.size(), 3);
  int count = 0;
  for (auto i = closed_set.begin(); i != closed_set.end(); ++i) {
    visited_set_(count, 0) = i->second->layer;
    visited_set_(count, 1) = i->second->idx[1];
    visited_set_(count, 2) = i->second->idx[2];
    count += 1;
  }
}

// 获取邻居（未实现）
std::vector<Eigen::Vector3i> Astar::GetNeighbors(Node* node) const {}

// 获取指定层的成本图（用于调试）
Eigen::MatrixXd Astar::GetCostLayer(int layer) const {
  Eigen::MatrixXd cost_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      cost_layer(i, j) = grid_map_[layer][i][j].cost;
    }
  }
  return cost_layer;
}

// 获取指定层的高程（gateway）图（用于调试）
Eigen::MatrixXd Astar::GetEleLayer(int layer) const {
  Eigen::MatrixXd ele_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      ele_layer(i, j) = grid_map_[layer][i][j].ele;
    }
  }
  return ele_layer;
}