# 创建一个名为 map_manager 的共享库，
# 包含 DenseElevationMap 类的实现。
add_library(map_manager SHARED
    dense_elevation_map.cc
)

# 使用pybind11创建名为 py_map_manager 的Python模块。
# "py_" 前缀通常用于区分纯C++库和Python绑定模块。
pybind11_add_module(py_map_manager
  python_interface.cc
)

# 将 map_manager 库链接到 py_map_manager 模块。
target_link_libraries(py_map_manager PRIVATE map_manager)