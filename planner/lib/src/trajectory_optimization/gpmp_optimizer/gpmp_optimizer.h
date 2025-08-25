#pragma once // 防止头文件被重复包含

#include <Eigen/Dense>
#include <memory>

#include "common/data_types.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/height_smoother/height_smoother.h"

/**
 * @class GPMPOptimizer
 * @brief 使用GPMP(Gaussian Process Motion Planner)思想进行轨迹优化的基类。
 *
 * 这个类负责将A*搜索得到的粗糙路径平滑成一条动力学可行且安全的轨迹。
 * 它构建一个因子图，并使用GTSAM进行优化。
 * 这个版本的目标函数中包含了航向角。
 */
class GPMPOptimizer {
 public:
  GPMPOptimizer() = default;
  ~GPMPOptimizer() = default;

  /**
   * @brief 构造函数。
   * @param safe_cost_margin 安全成本边界。
   * @param max_heading_rate 最大航向变化率。
   * @param map 指向密集高程图的共享指针。
   */
  GPMPOptimizer(const double safe_cost_margin, const double max_heading_rate,
                std::shared_ptr<DenseElevationMap> map)
      : map_(map),
        safe_cost_margin_(safe_cost_margin),
        max_heading_rate_(max_heading_rate) {}

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

  // 设置参考高度
  void SetReferenceHeight(const double height) { reference_height_ = height; }

  // --- 结果获取函数 ---
  Eigen::MatrixXd GetOptInitValue() const { return opt_init_value_; }
  Eigen::MatrixXd GetOptInitLayer() const { return opt_init_layer_; }
  Eigen::MatrixXd GetResultMatrix() const { return trajectory_; }
  Eigen::VectorXd GetLayers() const { return opt_layers_; }
  Eigen::VectorXd GetHeights() const { return opt_height_; }
  Eigen::VectorXd GetResultCeiling() const { return opt_ceiling_; }
  Eigen::VectorXd GetHeadingRate() const;

  // 设置路径采样间隔
  void set_sample_interval(const int sample_interval) {
    sample_interval_ = sample_interval;
  }

  // 设置调试模式
  void SetDebug(const bool flag) { debug_ = flag; }

 protected:
  /**
   * @brief 将PathPoint转换为6维状态向量。
   * @param path_point 输入的路径点。
   * @param x 输出的6维状态向量 (x, y, h, vx, vy, vh)。
   */
  void PathPointToNode(const PathPoint& path_point, Vector6& x);

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
  Eigen::VectorXd opt_ceiling_;    // 优化后的天花板高度
  Eigen::MatrixXd trajectory_;     // 最终生成的轨迹

  // 参数
  int sample_interval_ = 10;   // 路径采样间隔
  int interpolate_num_ = 8;   // 插值点数量
  double safe_cost_margin_ = 10; // 安全成本边界
  int max_iterations_ = 100;    // 最大迭代次数
  double max_heading_rate_ = 0.5; // 最大航向变化率
  double reference_height_ = 0.1; // 参考高度

  HeightSmoother height_smoother_; // 高度平滑器实例
};
