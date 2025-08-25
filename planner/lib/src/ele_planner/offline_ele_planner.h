#pragma once

#include <memory>

#include "a_star/a_star_search.h"
#include "common/data_types.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer_wnoa.h"

/**
 * @class OfflineElePlanner
 * @brief 离线高程规划器，整合了地图、路径搜索和轨迹优化的顶层类。
 *
 * 这个类是整个C++规划器模块的入口点。它负责：
 * 1. 初始化地图数据。
 * 2. 调用A*算法进行前端路径搜索。
 * 3. 调用GPMP优化器进行后端轨迹优化。
 * 4. 根据配置选择使用哪种优化器。
 */
class OfflineElePlanner {
 public:
  /**
   * @brief 构造函数。
   * @param max_heading_rate 最大航向变化率。
   * @param use_quintic 是否使用不带航向角优化的版本（wnoa）。
   */
  OfflineElePlanner(const double max_heading_rate, bool use_quintic)
      : use_quintic_(use_quintic), max_heading_rate_(max_heading_rate) {}
  ~OfflineElePlanner() = default;

  /**
   * @brief 初始化地图和所有子模块（A*, 优化器）。
   * @param a_start_cost_threshold A*搜索的成本阈值。
   * @param safe_cost_margin 优化器的安全成本边界。
   * ... 其他地图参数
   */
  void InitMap(const double a_start_cost_threshold,
               const double safe_cost_margin, const double resolution,
               const int num_layers, const double step_cost_weight, const Eigen::MatrixXd& cost_map,
               const Eigen::MatrixXd& height_map,
               const Eigen::MatrixXd& ceiling, const Eigen::MatrixXd& ele_map,
               const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y);

  /**
   * @brief 执行完整的规划流程。
   * @param start 起点索引 (layer, row, col)。
   * @param goal 终点索引 (layer, row, col)。
   * @param optimize 是否执行轨迹优化。
   * @return 如果规划成功则返回true。
   */
  bool Plan(const Eigen::Vector3i& start, const Eigen::Vector3i& goal,
            const bool optimize = true);

  // 设置参考高度（仅用于wnoj优化器，但似乎写错了，应该是wnoa）
  void SetReferenceHeight(const double height) {
    // FIXME: 这里的wnoj似乎是笔误，应为wnoa
    // trajectory_optimizer_wnoj_.SetReferenceHeight(height);
  }

  // 开启所有子模块的调试模式
  void Debug() {
    path_finder_.Debug();
    trajectory_optimizer_.SetDebug(true);
    trajectory_optimizer_wnoj_.SetDebug(true);
  }

  // 获取A*搜索的原始路径（用于调试）
  Eigen::MatrixXd GetDebugPath() const {
    return path_finder_.GetResultMatrix();
  }

  // 设置优化器的最大迭代次数
  void set_max_iterations(int max_iterations) {
    trajectory_optimizer_.set_max_iterations(max_iterations);
    trajectory_optimizer_wnoj_.set_max_iterations(max_iterations);
  }

  // --- Getter函数，用于从Python端获取内部对象 ---
  const Astar& get_path_finder() const { return path_finder_; }
  const DenseElevationMap& get_map() const { return *map_; }
  // 获取带航向角优化的优化器
  const GPMPOptimizer& get_trajectory_optimizer() const {
    return trajectory_optimizer_wnoj_;
  }
  // 获取不带航向角优化的优化器
  const GPMPOptimizerWnoa& get_trajectory_optimizer_wnoj() const {
    return trajectory_optimizer_;
  }

 private:
  double max_heading_rate_ = 0.5; // 最大航向变化率
  bool use_quintic_ = false; // 是否使用不带航向角优化的版本

  // --- 核心模块实例 ---
  std::shared_ptr<DenseElevationMap> map_; // 地图管理器
  Astar path_finder_; // A*路径搜索器
  GPMPOptimizerWnoa trajectory_optimizer_; // 不带航向角优化的版本
  GPMPOptimizer trajectory_optimizer_wnoj_; // 带航向角优化的版本 (变量名似乎有误，wnoj应为with-heading-obj)

  // --- 存储结果 ---
  std::vector<PathPoint> path_; // A*搜索结果
  std::vector<Eigen::Vector3d> trajectory_; // 最终优化轨迹
};