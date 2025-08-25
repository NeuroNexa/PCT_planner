#pragma once // 防止头文件被重复包含

#include <Eigen/Dense>
#include <memory>

#include "common/data_types.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/height_smoother/height_smoother.h"

/**
 * @class GPMPOptimizerWnoa
 * @brief GPMPOptimizer的变体，"Wnoa" 推测意为 "With No Objective on Angle"。
 *
 * 这个版本使用4维状态向量(x, vx, y, vy)，不直接对航向角进行优化。
 * 航向角可以在后处理中根据速度方向计算得出。
 * 适用于对轨迹的最终航向没有严格要求的场景。
 */
class GPMPOptimizerWnoa {
 public:
  GPMPOptimizerWnoa() = default;
  ~GPMPOptimizerWnoa() = default;

  /**
   * @brief 构造函数。
   * @param safe_cost_margin 安全成本边界。
   * @param map 指向密集高程图的共享指针。
   */
  GPMPOptimizerWnoa(const double safe_cost_margin,
                    std::shared_ptr<DenseElevationMap> map)
      : map_(map), safe_cost_margin_(safe_cost_margin) {}

  // 一个用于测试GP先验因子的函数
  Eigen::MatrixXd GPPriorTest(Vector4 x0, Vector4 xN, const double T,
                              const int N);

  /**
   * @brief 生成平滑轨迹的核心函数。
   * @param path A*搜索得到的粗糙路径点序列。
   * @param T 总轨迹时间。
   * @return 如果成功生成轨迹则返回true。
   */
  bool GenerateTrajectory(const std::vector<PathPoint>& path, const double T);

  // 设置优化器的最大迭代次数
  void set_max_iterations(int max_iterations) {
    max_iterations_ = max_iterations;
  }

  // --- 结果获取函数 ---
  Eigen::MatrixXd GetOptInitValue() const { return opt_init_value_; }
  Eigen::MatrixXd GetOptInitLayer() const { return opt_init_layer_; }
  Eigen::MatrixXd GetResultMatrix() const { return trajectory_; }
  Eigen::VectorXd GetLayers() const { return opt_layers_; }
  Eigen::VectorXd GetHeights() const { return opt_height_; }

  // 设置路径采样间隔
  void set_sample_interval(const int sample_interval) {
    sample_interval_ = sample_interval;
  }

  // 设置调试模式
  void SetDebug(const bool flag) { debug_ = flag; }

 protected:
  /**
   * @brief 将PathPoint转换为4维状态向量。
   * @param path_point 输入的路径点。
   * @param x 输出的4维状态向量 (x, vx, y, vy)。
   */
  void PathPointToNode(const PathPoint& path_point, Vector4& x);

  /**
   * @brief 对路径进行降采样。
   * @param path 原始路径。
   * @param sub_sampled_path 输出的降采样后路径。
   */
  void SubSamplePath(const std::vector<PathPoint>& path,
                     std::vector<PathPoint>& sub_sampled_path);

 private:
  bool debug_ = false; // 调试标志

  std::shared_ptr<DenseElevationMap> map_ = nullptr; // 地图指针
  // 存储优化过程中的各种数据
  Eigen::MatrixXd opt_init_value_; // 优化变量的初始值
  Eigen::VectorXd opt_init_layer_; // 初始层
  Eigen::MatrixXd opt_results_;    // 优化结果
  Eigen::VectorXd opt_layers_;     // 优化后的层
  Eigen::VectorXd opt_height_;     // 优化后的高度
  Eigen::MatrixXd trajectory_;     // 最终生成的轨迹

  // 参数
  int sample_interval_ = 10;   // 路径采样间隔
  int interpolate_num_ = 8;   // 插值点数量
  double safe_cost_margin_ = 10; // 安全成本边界
  int max_iterations_ = 100;    // 最大迭代次数

  HeightSmoother height_smoother_; // 高度平滑器实例
};
