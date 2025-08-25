#include "a_star/a_star_search.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h" // 用于绑定STL容器

namespace py = pybind11;

// PYBIND11_MODULE宏创建了一个Python可导入的模块。
// 第一个参数(a_star)是模块名，必须与CMakeLists.txt中定义的模块名一致。
// 第二个参数(m)是py::module_对象，是所有绑定的主入口。
PYBIND11_MODULE(a_star, m) {
  // 绑定HeuristicType枚举类型，使其在Python中可用
  py::enum_<HeuristicType>(m, "HeuristicType")
      .value("EUCLIDEAN", HeuristicType::kEuclidean, "欧几里得距离")
      .value("MANHATTAN", HeuristicType::kManhattan, "曼哈顿距离")
      .value("DIAGONAL", HeuristicType::kDiagonal, "对角距离")
      .export_values(); // 将枚举成员导出到模块的命名空间

  // 绑定Astar类
  auto pyAstar = py::class_<Astar>(m, "Astar", "A* search algorithm class");
  pyAstar
      // 绑定构造函数，并为参数提供默认值
      .def(py::init<HeuristicType>(), "Constructor",
           py::arg("h_type") = HeuristicType::kDiagonal)
      // 绑定成员函数
      .def("init", &Astar::Init, "Initialize A* searcher")
      .def("search", &Astar::Search, "Search for a path")
      .def("debug", &Astar::Debug, "Enable debug mode")
      .def("get_result_matrix", &Astar::GetResultMatrix, "Get the result path as a matrix")
      .def("get_cost_layer", &Astar::GetCostLayer, "Get cost map of a layer for debugging")
      .def("get_ele_layer", &Astar::GetEleLayer, "Get elevation map of a layer for debugging")
      .def("get_visited_set", &Astar::GetVisitedSet, "Get visited nodes for debugging");
}