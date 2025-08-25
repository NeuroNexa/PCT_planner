#!/bin/bash

# 获取脚本所在的根目录
ROOT_DIR=$(cd $(dirname "$0"); pwd)
# echo "ROOT_DIR: ${ROOT_DIR}"

# --- 构建规划器核心库 ---
echo "Building planner libraries..."
cd lib

# 如果需要全新构建，可以取消下面这行注释来删除旧的构建目录
# rm -rf build
mkdir -p build # -p 选项确保如果目录已存在，则不会报错

cd build
# 运行CMake配置项目，使用Release模式进行优化
cmake ../ -DCMAKE_BUILD_TYPE=Release
# 使用6个核心并行编译
make -j6

# --- 复制编译好的库文件 ---
# 将编译生成的Python模块（.so文件）复制到 'planner/lib' 目录下，
# 以便Python脚本可以直接导入它们。
echo "Copying shared libraries..."
cp ./src/a_star/a_star*.so ../
cp ./src/trajectory_optimization/traj_opt*.so ../
cp ./src/ele_planner/ele_planner*.so ../
cp ./src/map_manager/py_map_manager*.so ../
cp ./src/common/smoothing/libcommon_smoothing.so ../
cd ..
echo "Build complete."


# --- 环境变量和可选步骤 ---

# 设置LD_LIBRARY_PATH，让系统能找到gtsam和自定义的平滑库
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${ROOT_DIR}/lib/3rdparty/gtsam-4.1.1/install/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${ROOT_DIR}/lib/build/src/common/smoothing

# 将 'planner/lib' 目录添加到PYTHONPATH，以便Python解释器能找到我们编译的模块
export PYTHONPATH=$PYTHONPATH:${ROOT_DIR}/lib

# --- （可选）生成Python存根文件 (.pyi) ---
# pybind11-stubgen可以为pybind11生成的二进制模块创建.pyi存根文件。
# 这些存根文件提供了类型提示，可以极大地改善开发体验（例如，在VSCode中实现自动补全和类型检查）。
# 如果需要重新生成存根文件，请取消以下行的注释并确保已安装pybind11-stubgen (`pip install pybind11-stubgen`)。
# echo "Generating pyi stubs (optional)..."
# pybind11-stubgen -o ./ a_star
# pybind11-stubgen -o ./ traj_opt
# pybind11-stubgen -o ./ ele_planner
# pybind11-stubgen -o ./ py_map_manager
# # 将生成的存根文件移动到正确的位置并重命名
# cp ./a_star-stubs/__init__.pyi ./a_star.pyi
# cp ./traj_opt-stubs/__init__.pyi ./traj_opt.pyi
# cp ./ele_planner-stubs/__init__.pyi ./ele_planner.pyi
# cp ./py_map_manager-stubs/__init__.pyi ./py_map_manager.pyi
# echo "Stubs generated."
