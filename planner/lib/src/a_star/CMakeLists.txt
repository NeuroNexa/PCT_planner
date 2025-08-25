# 创建一个名为 a_star_search 的共享库（.so文件），
# 这个库包含了A*算法的核心实现。
# 但在这个项目中，它似乎没有被其他C++代码直接链接，
# 可能是历史遗留或为了模块化。
add_library(a_star_search SHARED
  a_star_search.cc
)

# 使用pybind11_add_module宏来创建一个Python模块。
# 这个模块名为 "a_star"，源文件是 python_interface.cc。
# python_interface.cc 中包含了A* C++类到Python的绑定代码。
pybind11_add_module(a_star
  python_interface.cc
)

# 将 a_star_search 库链接到 a_star 模块。
# 这意味着 a_star 模块（Python绑定）可以使用 a_star_search 库中定义的代码。
# PRIVATE 关键字表示这个链接依赖是私有的，不会传递给链接到 a_star 的其他目标。
target_link_libraries(a_star PRIVATE a_star_search)
