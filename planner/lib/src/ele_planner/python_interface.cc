#include "ele_planner/offline_ele_planner.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

// PYBIND11_MODULE宏创建了一个Python可导入的模块。
// 第一个参数(ele_planner)是模块名，必须与CMakeLists.txt中定义的模块名一致。
// 第二个参数(m)是py::module_对象，是所有绑定的主入口。
PYBIND11_MODULE(ele_planner, m) {
  // 绑定OfflineElePlanner类，这是C++规划器的主要入口
  auto pyOfflineElePlanner =
      py::class_<OfflineElePlanner>(m, "OfflineElePlanner");
  pyOfflineElePlanner
      // 绑定构造函数，并为参数提供名称和默认值
      .def(py::init<double, bool>(), "Constructor",
           py::arg("max_heading_rate"),
           py::arg("use_quintic") = false)
      // 绑定各个成员函数
      .def("init_map", &OfflineElePlanner::InitMap, "Initialize the map data")
      .def("plan", &OfflineElePlanner::Plan, "Perform planning")
      .def("debug", &OfflineElePlanner::Debug, "Enable debug mode")
      .def("set_reference_height", &OfflineElePlanner::SetReferenceHeight, "Set reference height for the optimizer")
      .def("set_max_iterations", &OfflineElePlanner::set_max_iterations, "Set max iterations for the optimizer")
      // 以下getter函数用于从Python访问C++内部的各个模块，
      // 通常返回的是对内部对象的引用或常量引用，以避免不必要的拷贝。
      .def("get_path_finder", &OfflineElePlanner::get_path_finder,
           py::return_value_policy::reference_internal, "Get the A* path finder object")
      .def("get_map", &OfflineElePlanner::get_map,
           py::return_value_policy::reference_internal, "Get the map object")
      .def("get_trajectory_optimizer",
           &OfflineElePlanner::get_trajectory_optimizer,
           py::return_value_policy::reference_internal, "Get the trajectory optimizer object")
      .def("get_trajectory_optimizer_wnoj",
           &OfflineElePlanner::get_trajectory_optimizer_wnoj,
           py::return_value_policy::reference_internal, "Get the trajectory optimizer object (wnoj version)")
      .def("get_debug_path", &OfflineElePlanner::GetDebugPath, "Get the raw path from A* for debugging");
}