#include "trajectory_optimization/height_smoother/height_smoother.h"

#include <iostream>

#include "common/smoothing/osqp_spline1d_solver.h" // 引入OSQP样条曲线求解器

Eigen::VectorXd HeightSmoother::Smooth(const Eigen::VectorXd& coarse_height,
                                       const Eigen::VectorXd& upper_bound,
                                       const double dt, const int N,
                                       const double knot_interval) {
  // --- 准备求解器所需的数据 ---
  std::vector<double> lbs;          // 高度的下界
  std::vector<double> ubs;          // 高度的上界
  std::vector<double> refs;         // 参考高度（即输入的粗糙高度）
  std::vector<double> ts;           // 时间戳
  std::vector<double> knots;        // 样条曲线的节点

  // 生成样条曲线的节点
  for (int i = 0; i < N; ++i) {
    knots.emplace_back(i * knot_interval);
  }

  // 准备每个时间点的约束和参考值
  for (int i = 0; i < coarse_height.size(); ++i) {
    lbs.emplace_back(-1.0); // 设置一个通用的下界
    ubs.emplace_back(std::max(-1.0, upper_bound(i) - 0.3)); // 上界为地形上界减去一个安全裕度
    refs.emplace_back(coarse_height(i)); // 参考值为原始高度
    ts.emplace_back(i * dt); // 时间戳
  }

  // --- 配置并运行OSQP求解器 ---

  // 创建一个5阶的1D样条曲线求解器
  common::OsqpSpline1dSolver solver(knots, 5);

  // 获取并配置代价函数（kernel）
  auto kernel = solver.mutable_kernel();
  kernel->AddRegularization(1e-5); // 添加正则化项，防止过拟合
  // kernel->AddSecondOrderDerivativeMatrix(5); // 可以选择最小化二阶导数（加速度）
  kernel->AddThirdOrderDerivativeMatrix(30); // 最小化三阶导数（jerk），使轨迹更平滑
  kernel->AddReferenceLineKernelMatrix(ts, refs, 1); // 添加参考线项，使结果接近原始高度

  // 获取并配置约束
  auto constraint = solver.mutable_constraint();
  constraint->AddThirdDerivativeSmoothConstraint(); // 添加三阶导数连续性约束
  constraint->AddPointConstraint(ts.front(), coarse_height(0)); // 固定起点的高度
  // 以下是被注释掉的约束，可以用于固定起点的一阶和二阶导数
  // constraint->AddPointDerivativeConstraint(t_knots_.front(), init_s_[1]);
  // constraint->AddPointSecondDerivativeConstraint(t_knots_.front(),
  // init_s_[2]);
  constraint->AddBoundary(ts, lbs, ubs); // 添加每个点的高度边界约束
  // 以下是被注释掉的约束，可以用于限制速度和加速度
  // constraint->AddDerivativeBoundary(t_samples, v_min, v_max);
  // constraint->AddSecondDerivativeBoundary(t_samples, a_min, a_max);

  // 求解QP问题
  if (!solver.Solve()) {
    // 如果求解失败，打印信息并返回原始的粗糙高度
    // std::cout << "Fail to solve the spline" << std::endl;
    return coarse_height;
  }

  // --- 提取并返回结果 ---
  auto spline = solver.spline(); // 获取求解得到的样条曲线
  Eigen::VectorXd smooth_height(coarse_height.size());
  // 在每个时间点上对样条曲线进行采样，得到平滑后的高度值
  for (int i = 0; i < coarse_height.size(); ++i) {
    smooth_height(i) = spline(ts[i]);
  }
  return smooth_height;
}
