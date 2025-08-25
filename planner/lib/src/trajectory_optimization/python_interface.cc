#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer_wnoa.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoa_interpolator.hpp"

namespace py = pybind11;

// PYBIND11_MODULE宏创建了一个Python可导入的模块。
// 第一个参数(traj_opt)是模块名，必须与CMakeLists.txt中定义的模块名一致。
// 第二个参数(m)是py::module_对象，是所有绑定的主入口。
PYBIND11_MODULE(traj_opt, m) {
  // --- 绑定 GPMPOptimizer 类 ---
  // 这是带航向角优化的版本
  auto pyGPMPOptimizer = py::class_<GPMPOptimizer>(m, "GPMPOptimizer");
  pyGPMPOptimizer.def(py::init<>()) // 绑定默认构造函数
      .def("set_debug", &GPMPOptimizer::SetDebug, "开启调试模式")
      .def("get_result_matrix", &GPMPOptimizer::GetResultMatrix, "获取最终轨迹矩阵")
      .def("get_layers", &GPMPOptimizer::GetLayers, "获取优化后的层索引")
      .def("get_heights", &GPMPOptimizer::GetHeights, "获取优化后的高度")
      .def("get_ceilings", &GPMPOptimizer::GetResultCeiling, "获取天花板高度")
      .def("get_opt_init_value", &GPMPOptimizer::GetOptInitValue, "获取优化变量的初始值")
      .def("get_opt_init_layer", &GPMPOptimizer::GetOptInitLayer, "获取初始层索引")
      .def("get_heading_rate", &GPMPOptimizer::GetHeadingRate, "获取航向变化率");

  // --- 绑定 GPMPOptimizerWnoa 类 ---
  // 这是不带航向角优化的版本
  auto pyGPMPOptimizerWnoa =
      py::class_<GPMPOptimizerWnoa>(m, "GPMPOptimizerWnoa");
  pyGPMPOptimizerWnoa.def(py::init<>()) // 绑定默认构造函数
      .def("set_debug", &GPMPOptimizerWnoa::SetDebug, "开启调试模式")
      .def("gp_prior_test", &GPMPOptimizerWnoa::GPPriorTest, "运行GP先验测试")
      .def("get_result_matrix", &GPMPOptimizerWnoa::GetResultMatrix, "获取最终轨迹矩阵")
      .def("get_layers", &GPMPOptimizerWnoa::GetLayers, "获取优化后的层索引")
      .def("get_heights", &GPMPOptimizerWnoa::GetHeights, "获取优化后的高度")
      .def("get_opt_init_value", &GPMPOptimizerWnoa::GetOptInitValue, "获取优化变量的初始值")
      .def("get_opt_init_layer", &GPMPOptimizerWnoa::GetOptInitLayer, "获取初始层索引");
}