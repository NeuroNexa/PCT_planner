#include "map_manager/dense_elevation_map.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

// PYBIND11_MODULE宏创建了一个Python可导入的模块。
// 第一个参数(py_map_manager)是模块名，必须与CMakeLists.txt中定义的模块名一致。
// 第二个参数(m)是py::module_对象，是所有绑定的主入口。
PYBIND11_MODULE(py_map_manager, m) {
  // 定义DenseElevationMap类的Python绑定
  auto pyDenseElevationMap =
      py::class_<DenseElevationMap>(m, "DenseElevationMap");

  // 绑定DenseElevationMap类的构造函数和成员函数
  pyDenseElevationMap.def(py::init<>()) // 绑定默认构造函数
      // .def(<Python方法名>, <C++函数指针>, <可选的参数说明>)
      .def("update_layer", &DenseElevationMap::UpdateLayer, "Update layer index")
      .def("update_layer_safe", &DenseElevationMap::UpdateLayerSafe, "Safely update layer index with height hint")
      .def("set_debug", &DenseElevationMap::SetDebug, "Set debug flag");
}