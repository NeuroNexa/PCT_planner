# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.13.0)
# 定义项目名称和语言
project(planning_core CXX)

# --- 编译器和编译选项 ---
# 设置C++标准为C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)

# 如果没有指定构建类型，默认为Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 设置通用的编译警告选项
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# 为Debug模式添加调试信息
set(CMAKE_CXX_FLAGS_DEBUG "-g")
# 为Release模式添加优化选项
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
# 添加-march=native以利用CPU的特定指令集进行优化，这对gtsam等库有性能提升
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native")

# --- 依赖项和头文件路径 ---
# 指定GTSAM库的CMake配置文件路径
set(GTSAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/gtsam-4.1.1/install/lib/cmake/GTSAM)

# 查找所需的GTSAM和Eigen3库
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

# 添加全局的头文件包含目录
include_directories(
  src  # 项目自身的src目录
  ${EIGEN3_INCLUDE_DIR} # Eigen库的头文件目录
  ${GTSAM_INCLUDE_DIR}  # GTSAM库的头文件目录
)

# --- 添加子目录 ---
# 依次添加各个子模块的CMakeLists.txt文件，以便CMake可以处理它们
add_subdirectory(src/common)
add_subdirectory(3rdparty/pybind11)
add_subdirectory(src/a_star)
add_subdirectory(src/trajectory_optimization)
add_subdirectory(src/map_manager)
add_subdirectory(src/ele_planner)