#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer_wnoa.h"

#include <chrono>
#include <cmath>

#include "gtsam/nonlinear/GaussNewtonOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtParams.h"
#include "gtsam/nonlinear/Symbol.h"
// 引入这个优化器专用的因子
#include "trajectory_optimization/gpmp_optimizer/factors_origin/gp_prior_factor.h"
#include "trajectory_optimization/gpmp_optimizer/factors_wnoa/gp_interpolate_obstacle_factor.h"
#include "trajectory_optimization/gpmp_optimizer/factors_wnoa/gp_obstacle_factor.h"
#include "trajectory_optimization/gpmp_optimizer/factors_wnoa/gp_prior_factor.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoa_interpolator.hpp"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoa_trajectory_interpolator.h"

using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Isotropic;
using PriorFactor4 = gtsam::PriorFactor<Vector4>; // 状态向量是4维

constexpr double kQc = 0.1; // GP过程噪声

bool GPMPOptimizerWnoa::GenerateTrajectory(
    const std::vector<PathPoint>& input_path, const double T) {
  auto t0 = std::chrono::high_resolution_clock::now();
  // 4维状态向量的噪声模型
  static auto sigma_initial = Diagonal::Sigmas(Vector4(0.001, 0.1, 0.001, 0.1));
  static auto sigma_goal = Diagonal::Sigmas(Vector4(0.001, 1, 0.001, 1));

  // 路径降采样
  std::vector<PathPoint> path;
  SubSamplePath(input_path, path);

  // 计算时间步长
  double T_tmp = static_cast<double>(input_path.size() - 1) / 3;
  int N = path.size();
  double dt = T_tmp / (N - 1);
  double tau = dt / (interpolate_num_ + 1);
  std::cout << "tau: " << tau << std::endl;

  // 获取起点和终点的4D状态
  Vector4 x0, xN;
  PathPointToNode(path.front(), x0);
  PathPointToNode(path.back(), xN);

  if (debug_) {
    printf("N: %d, dt: %f, tau: %f\n", N, dt, tau);
    std::cout << "x0: " << x0.transpose() << std::endl;
    std::cout << "xN: " << xN.transpose() << std::endl;
  }

  // 构建因子图
  auto graph = gtsam::NonlinearFactorGraph();
  int factor_idx = 0;
  std::vector<int> obstacle_factor_idx;

  // 设置优化变量的初始值
  gtsam::Values init_values;
  opt_init_value_ = Eigen::MatrixXd(4, N + (N - 1) * interpolate_num_);
  opt_init_layer_ = Eigen::VectorXd(N + (N - 1) * interpolate_num_);

  // 通过插值生成初始轨迹
  int col_index = 0;
  for (int i = 0; i < N; ++i) {
    Vector4 x1, x2;
    PathPointToNode(path[i], x1);
    init_values.insert<Vector4>(gtsam::Symbol('x', i), x1);
    opt_init_value_.col(col_index) = x1;
    opt_init_layer_(col_index) = path[i].layer;
    col_index++;
    if (i < N - 1) {
      PathPointToNode(path[i + 1], x2);
      for (int j = 0; j < interpolate_num_; ++j) {
        auto inter_x =
            GPInterpolatorWnoa::Interpolate(x1, x2, kQc, dt, tau * (j + 1));
        double height_hint =
            path[i].height + (path[i + 1].height - path[i].height) * (j + 1) /
                                 (interpolate_num_ + 1);
        opt_init_value_.col(col_index) = inter_x;
        opt_init_layer_(col_index) = map_->UpdateLayerSafe(
            path[i].layer, inter_x(0, 0), inter_x(2, 0), height_hint);
        col_index++;
      }
    }
  }

  if (debug_) {
    std::cout << "opt_init_layer OK" << std::endl;
  }

  // 添加起点先验因子
  graph.add(PriorFactor4(gtsam::Symbol('x', 0), x0, sigma_initial));
  factor_idx++;
  // 添加GP因子和障碍物因子
  for (int i = 1; i < N; ++i) {
    gtsam::Key last_x = gtsam::Symbol('x', i - 1);
    gtsam::Key this_x = gtsam::Symbol('x', i);
    // GP因子
    graph.add(GPPriorFactorWnoa(last_x, this_x, dt, kQc));
    factor_idx++;
    // 插值点的障碍物因子
    for (int j = 0; j < interpolate_num_; ++j) {
      double height_hint =
          path[i - 1].height + (path[i].height - path[i - 1].height) * (j + 1) /
                                   (interpolate_num_ + 1);
      graph.add(GPInterpolateObstacleFactorWnoa(
          last_x, this_x, map_, path[i].layer, height_hint, safe_cost_margin_,
          0.1, kQc, dt, (i - 1) * dt, tau * (j + 1)));
      obstacle_factor_idx.emplace_back(factor_idx + 1e7);
      factor_idx++;
    }
    // 路径节点的障碍物因子
    if (i < N - 1) {
      graph.add(GPObstacleFactorWnoa(this_x, map_, path[i].layer,
                                     path[i].height, 0.1, safe_cost_margin_,
                                     true));
      obstacle_factor_idx.emplace_back(factor_idx);
      factor_idx++;
    }
  }
  // 添加终点先验因子
  graph.add(PriorFactor4(gtsam::Symbol('x', N - 1), xN, sigma_goal));
  factor_idx++;

  if (debug_) {
    std::cout << "graph OK" << std::endl;
  }

  // 配置优化器参数
  gtsam::LevenbergMarquardtParams param;
  param.setMaxIterations(max_iterations_);

  // 运行优化
  gtsam::LevenbergMarquardtOptimizer opt(graph, init_values, param);
  opt_results_ = Eigen::MatrixXd(N, 4);
  auto solution = opt.optimize();

  if (debug_) {
    printf("optimization OK\n");
  }

  // 提取优化后的层信息
  opt_layers_ = Eigen::VectorXd::Zero(N + (N - 1) * interpolate_num_);
  opt_layers_(0) = path.front().layer;
  opt_layers_(opt_layers_.size() - 1) = path.back().layer;
  for (int i = 0; i < obstacle_factor_idx.size(); ++i) {
    if (obstacle_factor_idx[i] < 1e7) {
      opt_layers_(i + 1) = dynamic_cast<GPObstacleFactorWnoa*>(
                               graph.at(obstacle_factor_idx[i]).get())
                               ->GetNodeLayer();
    } else {
      opt_layers_(i + 1) = dynamic_cast<GPInterpolateObstacleFactorWnoa*>(
                               graph.at(obstacle_factor_idx[i] - 1e7).get())
                               ->GetNodeLayer();
    }
  }

  // 提取优化结果
  for (int i = 0; i < N; ++i) {
    opt_results_.row(i) =
        solution.at<Vector4>(gtsam::Symbol('x', i)).transpose();
  }

  // 生成密集的轨迹
  WnoaTrajectoryInterpolator traj_interpolator =
      WnoaTrajectoryInterpolator(opt_results_, dt, kQc);
  trajectory_ = traj_interpolator.GenerateTrajectory(interpolate_num_);

  // 提取高度信息
  opt_height_ = Eigen::VectorXd::Zero(N + (N - 1) * interpolate_num_);
  for (int i = 0; i < opt_layers_.size(); ++i) {
    opt_height_(i) =
        map_->GetHeight(opt_layers_(i), trajectory_(i, 0), trajectory_(i, 2));
  }

  // 对高度进行平滑 (当前被注释掉了)
  printf("smooth height\n");
  //! todo: upper bound is not set here
  // opt_height_ =
  //     height_smoother_.Smooth(opt_height_, Eigen::VectorXd(), tau, N, dt);

  auto t1 = std::chrono::high_resolution_clock::now();
  printf(
      "Optimization finished for N = %d at iteration %d, time elapsed: %f ms, "
      "cost: %f\n",
      N, opt.iterations(),
      std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() /
          1000.0,
      opt.error());

  return true;
}

// 将PathPoint转换为4D状态向量 [x, vx, y, vy]
void GPMPOptimizerWnoa::PathPointToNode(const PathPoint& path_point,
                                        Vector4& x) {
  double v = std::max(path_point.ref_v, 1.0);
  x(0, 0) = path_point.x;
  x(1, 0) = std::cos(path_point.heading) * v;
  x(2, 0) = path_point.y;
  x(3, 0) = std::sin(path_point.heading) * v;
}

// 路径降采样
void GPMPOptimizerWnoa::SubSamplePath(
    const std::vector<PathPoint>& path,
    std::vector<PathPoint>& sub_sampled_path) {
  assert(path.size() > 1);

  if (!sub_sampled_path.empty()) {
    sub_sampled_path.clear();
  }

  int size = path.size();
  int num_segs = std::ceil((size - 1) / sample_interval_);

  if (num_segs <= 1) {
    sub_sampled_path.emplace_back(path.front());
    sub_sampled_path.emplace_back(path.back());
    return;
  }

  double new_interval = static_cast<double>(size - 1) / num_segs;

  int idx;
  double float_idx = 0.0;

  sub_sampled_path.emplace_back(path.front());

  for (int i = 1; i < num_segs + 1; ++i) {
    float_idx += new_interval;
    idx = static_cast<int>(float_idx);

    if (idx == size - 1) {
      sub_sampled_path.emplace_back(path[idx]);
    } else {
      sub_sampled_path.emplace_back(
          PathPoint::Interpolate(path[idx], path[idx + 1], float_idx - idx));
    }
  }

  printf("num segs: %d, new interval: %f\n", num_segs, new_interval);
}

// GP先验因子测试函数
Eigen::MatrixXd GPMPOptimizerWnoa::GPPriorTest(Vector4 x0, Vector4 xN,
                                               const double T, const int N) {
  auto t0 = std::chrono::high_resolution_clock::now();
  static auto sigma_initial = Isotropic::Sigma(4, 0.01);
  static auto sigma_goal = Isotropic::Sigma(4, 0.01);

  double dt = T / (N - 1);

  auto graph = gtsam::NonlinearFactorGraph();

  graph.add(PriorFactor4(gtsam::Symbol('x', 0), x0, sigma_initial));
  for (int i = 1; i < N; ++i) {
    gtsam::Key last_x = gtsam::Symbol('x', i - 1);
    gtsam::Key this_x = gtsam::Symbol('x', i);
    graph.add(GPPriorFactorOrigin(last_x, this_x, dt, 0.1));
  }
  graph.add(PriorFactor4(gtsam::Symbol('x', N - 1), xN, sigma_goal));

  gtsam::Values init_values;
  for (int i = 0; i < N; ++i) {
    init_values.insert<Vector4>(gtsam::Symbol('x', i),
                                x0 + i * (xN - x0) / (N - 1));
  }

  gtsam::LevenbergMarquardtParams param;
  param.setVerbosity("ERROR");

  gtsam::LevenbergMarquardtOptimizer opt(graph, init_values, param);
  Eigen::MatrixXd result(N, 4);
  auto solution = opt.optimize();
  for (int i = 0; i < N; ++i) {
    result.row(i) = solution.at<Vector4>(gtsam::Symbol('x', i)).transpose();
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  printf(
      "GP prior test for N = %d, time elapsed: %f ms\n", N,
      std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() /
          1000.0);

  printf("GP prior optimization finished with iteration %d, error: %f\n",
         opt.iterations(), opt.error());
  return result;
}
