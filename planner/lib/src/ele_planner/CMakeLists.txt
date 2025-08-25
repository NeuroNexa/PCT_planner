# 创建一个名为 ele_planner_lib 的共享库，
# 这个库包含了顶层规划器 OfflineElePlanner 的实现。
add_library(ele_planner_lib SHARED
    offline_ele_planner.cc
)

# 链接 ele_planner_lib 所需的依赖库。
# PUBLIC 关键字意味着链接到 ele_planner_lib 的其他目标也会自动链接到这些库。
target_link_libraries(ele_planner_lib PUBLIC
    a_star_search  # A* 搜索库
    gpmp_optimizer # 轨迹优化库
    map_manager    # 地图管理库
)

# 使用pybind11创建名为 ele_planner 的Python模块。
pybind11_add_module(ele_planner
    python_interface.cc
)

# 将 ele_planner_lib 链接到 ele_planner 模块，
# 这样Python绑定代码才能调用C++规划器的功能。
target_link_libraries(ele_planner PRIVATE
    ele_planner_lib
)