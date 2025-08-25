#include "ele_planner/offline_ele_planner.h"

// 初始化函数，设置所有子模块
void OfflineElePlanner::InitMap(
    const double a_start_cost_threshold, const double safe_cost_margin,
    const double resolution, const int num_layers, const double step_cost_weight,
    const Eigen::MatrixXd& cost_map, const Eigen::MatrixXd& height_map,
    const Eigen::MatrixXd& ceiling, const Eigen::MatrixXd& ele_map,
    const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y) {
  // 初始化A*搜索器
  path_finder_.Init(a_start_cost_threshold, num_layers, resolution, step_cost_weight, cost_map,
                    height_map, ele_map);
  // 创建并初始化地图管理器
  map_ = std::make_shared<DenseElevationMap>();
  map_->Init(resolution, num_layers, cost_map, ele_map, height_map, ceiling,
             grad_x, grad_y);
  // 初始化不带航向角优化的优化器 (GPMPOptimizerWnoa)
  trajectory_optimizer_ = GPMPOptimizerWnoa(safe_cost_margin, map_);
  // 初始化带航向角优化的优化器 (GPMPOptimizer)
  trajectory_optimizer_wnoj_ =
      GPMPOptimizer(safe_cost_margin, max_heading_rate_, map_);
}

// 规划函数，执行A*搜索和可选的轨迹优化
bool OfflineElePlanner::Plan(const Eigen::Vector3i& start,
                             const Eigen::Vector3i& goal, const bool optimize) {
  // 步骤1: 使用A*进行前端路径搜索
  if (!path_finder_.Search(start, goal)) {
    printf("A star Failed!\n");
    return false;
  }

  // 步骤2: 如果需要，执行后端轨迹优化
  if (optimize) {
    // 从A*获取路径点
    path_ = path_finder_.GetPathPoints();
    // 设置起点和终点的参考速度
    path_.front().ref_v = 1;
    path_.back().ref_v = 1;

    bool success = false;
    // 根据配置选择使用哪个优化器
    if (use_quintic_) {
      // 使用带航向角优化的版本
      success = trajectory_optimizer_wnoj_.GenerateTrajectory(path_, 200);
    } else {
      // 使用不带航向角优化的版本
      success = trajectory_optimizer_.GenerateTrajectory(path_, 200);
    }

    return success;
  }

  // 如果不进行优化，A*搜索成功即视为规划成功
  return true;
}
