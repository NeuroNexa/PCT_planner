#pragma once // 防止头文件被重复包含

#include <Eigen/Core>
#include <unordered_map>
#include <vector>

#include "common/data_types.h" // 包含自定义数据类型，如PathPoint

// 定义启发式函数的类型
enum HeuristicType : int { kEuclidean = 0, kManhattan = 1, kDiagonal = 2 };

/**
 * @class Node
 * @brief A*算法中用于表示搜索节点的类。
 */
class Node {
 public:
  Node() = default;
  Node(Eigen::Vector3i idx, Node* parent) : idx(idx), parent(parent) {}
  ~Node() = default;

  // 重载==运算符，用于比较两个节点是否相同（基于索引）
  bool operator==(const Node& other) const { return idx == other.idx; }

  // 重置节点的f和g值以及父节点
  void Reset() {
    f = 0.0;
    g = 1e9;
    parent = nullptr;
  }

  double f = 1e9; // f = g + h, 总代价
  double g = 1e9; // 从起点到当前节点的实际代价
  double height = 0.0; // 节点的高度
  double ele = 0; // 节点的高程
  double cost = 0.0; // 节点的通行成本
  int layer = 0; // 节点所在的层
  Eigen::Vector3i idx = Eigen::Vector3i(0, 0, 0);  // 节点的3D索引 (layer, row, col)
  Node* parent = nullptr; // 父节点指针
};

// 用于优先队列的节点比较结构体
struct NodeCompare {
  // 重载()运算符，使得f值小的节点优先级更高
  bool operator()(const Node* a, const Node* b) const { return a->f > b->f; }
};

// 定义多层栅格地图的类型别名
using MultiLayerGridMap = std::vector<std::vector<std::vector<Node>>>;

/**
 * @class Astar
 * @brief 实现了在多层栅格地图上进行搜索的A*算法。
 */
class Astar {
 public:
  // 构造函数，可以指定启发式函数类型
  Astar(const HeuristicType h_type = kDiagonal) : h_type_(h_type) {
    switch (h_type) {
      case kEuclidean:
        printf("Using Euclidean heuristic\n");
        break;
      case kManhattan:
        printf("Using Manhattan heuristic\n");
        break;
      case kDiagonal:
        printf("Using Diagonal heuristic\n");
        break;
    };
  }
  ~Astar() = default;

  /**
   * @brief 初始化A*搜索器。
   * @param cost_map 成本图。
   * @param height_map 高度图。
   * @param ele_map 包含网关信息的高程图。
   * ...其他参数
   */
  void Init(const double cost_threshold, const int num_layers,
            const double resolution, const double step_cost_weight,  const Eigen::MatrixXd& cost_map,
            const Eigen::MatrixXd& height_map, const Eigen::MatrixXd& ele_map);

  // 重置搜索状态，清空open/closed list
  void Reset();

  // 开启调试模式
  void Debug() { debug_ = true; }

  /**
   * @brief 执行A*搜索。
   * @param start 起点索引。
   * @param goal 终点索引。
   * @return 如果找到路径则返回true，否则返回false。
   */
  bool Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal);

  // 获取路径点向量
  std::vector<PathPoint> GetPathPoints() const;

  // 以矩阵形式获取搜索结果路径
  Eigen::MatrixXd GetResultMatrix() const;
  // 获取访问过的节点集合（用于调试）
  Eigen::MatrixXi GetVisitedSet() const { return visited_set_; }

  // 获取指定层的成本图（用于调试）
  Eigen::MatrixXd GetCostLayer(int layer) const;
  // 获取指定层的高程图（用于调试）
  Eigen::MatrixXd GetEleLayer(int layer) const;

 private:
  // 计算两个节点之间的移动成本
  double CalculateStepCost(const Node* node1, const Node* node2) const;

  // 决定当前节点应该在哪一层进行搜索
  int DecideLayer(const Node* cur_node) const;

  // 计算节点索引的哈希值，用于unordered_map
  int GetHash(const Eigen::Vector3i& idx) const;

  // 获取一个节点的邻居节点
  std::vector<Eigen::Vector3i> GetNeighbors(Node* node) const;

  // 计算启发式成本h(n)
  double GetHeuristic(const Node* node1, const Node* node2) const;

  // 将路径向量转换为矩阵
  Eigen::MatrixXd PathToMatrix(const std::vector<Eigen::Vector3i>& path);

  // 将路径索引向量转换为PathPoint向量
  void ToPathPoints(const std::vector<Eigen::Vector3i>& path,
                    std::vector<PathPoint>& path_points);

  // 将closed set转换为矩阵（用于调试）
  void ConvertClosedSetToMatrix(
      const std::unordered_map<int, Node*>& closed_set);

 private:
  HeuristicType h_type_ = kDiagonal; // 启发式函数类型

  // 地图参数
  int max_x_ = 0;
  int max_y_ = 0;
  int max_layers_ = 0;
  int xy_size_ = 0;
  double resolution_ = 0;
  MultiLayerGridMap grid_map_; // 存储所有节点的栅格地图
  double cost_threshold_ = 35; // 成本阈值，高于此值的节点被视为障碍物
  double step_cost_weight_ = 1.0; // 移动成本的权重

  int search_layer_depth_ = 1; // 搜索的层深度（例如，同时搜索当前层、上一层、下一层）
  std::vector<int> search_layers_offset_; // 搜索层的偏移量

  bool debug_ = false; // 调试标志
  Eigen::MatrixXi visited_set_; // 访问过的节点集合

  std::vector<Node*> search_result_; // 存储搜索结果路径的节点指针
};
