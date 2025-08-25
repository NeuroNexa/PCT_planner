# 创建一个名为 gpmp_optimizer 的共享库，
# 这个库包含了所有轨迹优化相关的逻辑，包括两种优化器、所有自定义因子和高度平滑器。
add_library(gpmp_optimizer SHARED
  # 带航向角优化的因子
  gpmp_optimizer/factors/gp_prior_factor.cc
  gpmp_optimizer/factors/gp_obstacle_factor.cc
  gpmp_optimizer/factors/gp_interpolate_obstacle_factor.cc
  gpmp_optimizer/factors/gp_heading_rate_factor.cc
  gpmp_optimizer/factors/gp_interpolate_heading_rate_factor.cc
  # 不带航向角优化的因子
  gpmp_optimizer/factors_wnoa/gp_prior_factor.cc
  gpmp_optimizer/factors_wnoa/gp_obstacle_factor.cc
  gpmp_optimizer/factors_wnoa/gp_interpolate_obstacle_factor.cc
  # 原始的先验因子
  gpmp_optimizer/factors_origin/gp_prior_factor.cc
  # 插值器
  gpmp_optimizer/interpolator/wnoj_trajectory_interpolator.cc
  gpmp_optimizer/interpolator/wnoa_trajectory_interpolator.cc
  # 优化器主类
  gpmp_optimizer/gpmp_optimizer.cc
  gpmp_optimizer/gpmp_optimizer_wnoa.cc
  # 高度平滑器
  height_smoother/height_smoother.cc
)

# 链接 gpmp_optimizer 库所需的依赖。
target_link_libraries(gpmp_optimizer PUBLIC
  gtsam             # GTSAM优化库
  map_manager       # 地图管理库
  common_smoothing  # 通用的平滑工具库
)

# 使用pybind11创建名为 traj_opt 的Python模块。
pybind11_add_module(traj_opt
  python_interface.cc
)

# 将 gpmp_optimizer 库链接到 traj_opt 模块。
target_link_libraries(traj_opt PRIVATE gpmp_optimizer)

# --- (可选) 用于测试高度平滑器的可执行文件 ---
# 如果需要单独测试HeightSmoother，可以取消以下注释。
# add_executable(test_height_smoother
#   height_smoother/test_height_smoother.cc
#   height_smoother/height_smoother.cc
# )
# target_link_libraries(test_height_smoother PUBLIC
#   common_smoothing)