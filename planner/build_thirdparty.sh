#!/bin/bash

# 获取脚本所在的根目录
ROOT_DIR=$(cd $(dirname "$0"); pwd)

# --- 构建 gtsam ---
# gtsam (Georgia Tech Smoothing and Mapping) 是一个用于机器人和计算机视觉中平滑和映射问题的C++库。
# 它常用于解决SLAM（同时定位与建图）和SFM（运动恢复结构）等问题。
echo "Building gtsam..."
cd ${ROOT_DIR}/lib/3rdparty/gtsam-4.1.1
# 清理旧的构建文件和安装目录
rm -rf build install
# 创建新的构建和安装目录
mkdir build install
cd build
# 运行CMake进行配置
# -DCMAKE_INSTALL_PREFIX="../install"：指定安装目录
# -DCMAKE_BUILD_TYPE=Release：设置为发布模式以进行优化
# -DGTSAM_USE_SYSTEM_EIGEN=ON：使用系统中已安装的Eigen库
cmake .. -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON
# 使用6个核心并行编译并安装
make -j6 && make install
echo "gtsam built successfully."

# --- 构建 osqp ---
# osqp (Operator Splitting Quadratic Program) 是一个用于求解二次规划(QP)问题的数值优化包。
# 它在这个项目中可能用于轨迹优化。
echo "Building osqp..."
cd ${ROOT_DIR}/lib/3rdparty/osqp
# 清理旧的构建文件和安装目录
rm -rf build install
# 创建新的构建目录
mkdir build && cd build
# 运行CMake进行配置
cmake .. -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release
# 使用4个核心并行编译并安装
make -j4 && make install
echo "osqp built successfully."
