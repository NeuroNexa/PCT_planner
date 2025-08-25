#pragma once // 防止头文件被重复包含
#include <Eigen/Dense> // 引入Eigen库，用于向量和矩阵操作

/**
 * @class HeightSmoother
 * @brief 使用二次规划(QP)对轨迹的高度进行平滑处理。
 *
 * 这个类的主要功能是接收一个粗糙的高度序列，并生成一个更平滑、
 * 同时满足物理约束（如最大速度和加速度）的高度轨迹。
 */
class HeightSmoother {
 public:
  HeightSmoother() = default; // 默认构造函数
  ~HeightSmoother() = default; // 默认析构函数

  /**
   * @brief 对高度序列进行平滑。
   * @param coarse_height 粗糙的高度序列，来自路径规划的前端（如A*）。
   * @param upper_bound 高度的上界，通常由地形或天花板决定。
   * @param dt 时间间隔。
   * @param N 轨迹点的数量。
   * @param knot_interval B样条曲线或类似曲线的节点间距。
   * @return Eigen::VectorXd 平滑后的高度序列。
   */
  Eigen::VectorXd Smooth(const Eigen::VectorXd& coarse_height,
                         const Eigen::VectorXd& upper_bound, const double dt,
                         const int N, const double knot_interval);
};