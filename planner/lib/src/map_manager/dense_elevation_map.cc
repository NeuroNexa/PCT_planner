#include "map_manager/dense_elevation_map.h"

#include <cmath>
#include <iostream>

// 初始化地图数据
void DenseElevationMap::Init(const double resolution, const int num_layers,
                             const Eigen::MatrixXd& cost_map,
                             const Eigen::MatrixXd& ele_mask,
                             const Eigen::MatrixXd& height,
                             const Eigen::MatrixXd& ceiling,
                             const Eigen::MatrixXd& grad_x,
                             const Eigen::MatrixXd& grad_y) {
  // 设置地图参数
  resolution_ = resolution;
  resolution_inv_ = 1.0 / resolution;
  max_layers_ = num_layers;
  max_x_ = cost_map.cols();
  max_y_ = cost_map.rows() / num_layers; // 每层的y方向大小
  xy_size_ = max_x_ * max_y_;

  // 复制地图数据
  cost_ = cost_map;
  ele_mask_ = ele_mask;
  height_ = height;
  ceiling_ = ceiling;
  grad_x_ = grad_x;
  grad_y_ = grad_y;

  // 打印地图尺寸信息
  printf("max layers: %d, max_x: %d, max_y: %d\n", max_layers_, max_x_, max_y_);
}

// 获取栅格的真实成本，可能会在相邻层之间切换以找到最低成本
double DenseElevationMap::GetRealCost(int layer, double x, double y,
                                      Eigen::Vector2d* grad, int* new_layer) {
  const int col = index(x);
  int row = index(y) + layer * max_y_;
  double cost = cost_(row, col);

  // 如果成本低于安全阈值，认为该点不在层的边界附近，直接返回当前值
  if (cost < safe_cost_threshold_) {
    if (grad != nullptr) {
      *grad = Eigen::Vector2d(grad_x_(row, col), grad_y_(row, col));
    }
    return cost;
  }

  // 如果成本较高，可能在层边界，检查上层和下层
  // double ele_value = ele_mask_(row, col);
  double this_height = height_(row, col);
  int real_row = row;
  int real_layer = layer;

  // 检查下层
  if (layer > 0) {
    int lower_row = row - max_y_;
    double lower_height = height_(lower_row, col);
    // 如果与下层的高度差很小
    if (abs(this_height - lower_height) < resolution_) {
      double lower_cost = cost_(lower_row, col);
      // 如果下层的成本更低，则切换到下层
      if (lower_cost < cost) {
        cost = lower_cost;
        real_row = lower_row;
        real_layer = layer - 1;
      }
    }
  }

  // 检查上层
  if (layer < max_layers_ - 1) {
    int upper_row = row + max_y_;
    double upper_height = height_(upper_row, col);
    // 如果与上层的高度差很小
    if (abs(this_height - upper_height) < resolution_) {
      double upper_cost = cost_(upper_row, col);
      // 如果上层的成本更低，则切换到上层
      if (upper_cost < cost) {
        cost = upper_cost;
        real_row = upper_row;
        real_layer = layer + 1;
      }
    }
  }

  // 返回最终选择的层的梯度和层索引
  if (grad != nullptr) {
    *grad = Eigen::Vector2d(grad_x_(real_row, col), grad_y_(real_row, col));
  }
  if (new_layer != nullptr) {
    *new_layer = real_layer;
  }

  return cost;
}

// 安全地获取真实成本，使用高度提示来更新层
double DenseElevationMap::GetRealCostSafe(int layer, double x, double y,
                                          const double height_hint) {
  int real_layer = UpdateLayerSafe(layer, x, y, height_hint);
  return cost_(index_y_safe(y) + real_layer * max_y_, index_x_safe(x));
}

// 更新层索引，逻辑与GetRealCost类似，但不返回值
int DenseElevationMap::UpdateLayer(const int layer, const double x,
                                   const double y) {
  const int col = index(x);
  int row = index(y) + layer * max_y_;
  double cost = cost_(row, col);
  int real_layer = layer;
  double lower_cost = 99;
  double upper_cost = 99;
  double lower_height = -99;
  double upper_height = -99;

  // 成本低，直接返回当前层
  if (cost < safe_cost_threshold_) {
    if (debug_) {
      printf("cost < safe_cost_threshold_!, cost: %f\n", cost);
    }
    return layer;
  }

  // double ele_value = ele_mask_(row, col);
  double this_height = height_(row, col);

  // 检查下层
  if (layer > 0) {
    int lower_row = row - max_y_;
    lower_height = height_(lower_row, col);
    if (abs(this_height - lower_height) < resolution_ || this_height < -50) {
      lower_cost = cost_(row - max_y_, col);
      if (lower_cost + offset_ < cost) {
        real_layer = layer - 1;
        cost = lower_cost;
      }
    }
  }

  // 检查上层
  if (layer < max_layers_ - 1) {
    int upper_row = row + max_y_;
    upper_height = height_(upper_row, col);
    if (abs(this_height - upper_height) < resolution_ || this_height < -50) {
      upper_cost = cost_(row + max_y_, col);
      if (upper_cost + offset_ < cost) {
        real_layer = layer + 1;
      }
    }
  }

  if (debug_) {
    printf(
        "layer: %d, x: %f, y: %f, row: %d, col: %d, cost: %f, lower_cost: %f, "
        "upper_cost: %f, height: %f\n, lower_height: %f, upper_height: %f\n",
        layer, x, y, index(y), col, cost_(row, col), lower_cost, upper_cost,
        height_(row, col), lower_height, upper_height);
  }

  return real_layer;
}

// 安全地更新层索引，使用高度提示来辅助判断
int DenseElevationMap::UpdateLayerSafe(const int layer, const double x,
                                       const double y,
                                       const double height_hint) {
  const int col = index_x_safe(x);
  int row = index_y_safe(y) + layer * max_y_;
  double cost = cost_(row, col);
  int real_layer = layer;
  double this_height = GetHeight(layer, x, y);
  // 判断当前栅格是否"不安全"：高度无效或与提示高度差异过大
  bool unsafe_grid =
      (this_height < -50) || (abs(this_height - height_hint) > 5 * resolution_);

  // 如果栅格安全且成本低，直接返回
  if ((!unsafe_grid) && (cost < safe_cost_threshold_)) {
    if (debug_) {
      printf("cost < safe_cost_threshold_!, cost: %f, height: %f\n", cost,
             height_(row, col));
    }
    return layer;
  }

  // 对于不安全的栅格，成本和高度都不可靠，需要检查相邻层
  double upper_height = -200;
  double lower_height = -200;
  double lower_cost = 1000;
  double upper_cost = 1000;
  double min_cost = unsafe_grid ? 1000 : cost;

  // 检查下层
  if (layer > 0) {
    lower_height = height_(row - max_y_, col);
    lower_cost = cost_(row - max_y_, col);
    // 如果下层高度与提示高度接近
    if (abs(height_hint - lower_height) < 5 * resolution_) {
      // 并且下层是连续的或成本更低，或者当前栅格不安全但下层是安全的
      if ((abs(this_height - lower_height) < 1.5 * resolution_ &&
           lower_cost < cost) ||
          (unsafe_grid && lower_cost < 2 * safe_cost_threshold_)) {
        real_layer = layer - 1;
        min_cost = lower_cost;
      }
    }
  }

  // 检查上层
  if (layer < max_layers_ - 1) {
    upper_height = height_(row + max_y_, col);
    upper_cost = cost_(row + max_y_, col);
    if (abs(height_hint - upper_height) < 5 * resolution_) {
      if ((abs(this_height - upper_height) < 1.5 * resolution_ &&
           upper_cost < cost) ||
          (unsafe_grid && upper_cost < 2 * safe_cost_threshold_)) {
        // 只有当上层成本比当前最低成本还低时才更新
        if (upper_cost < min_cost) {
          real_layer = layer + 1;
          min_cost = upper_cost;
        }
      }
    }
  }

  if (debug_) {
    printf(
        "layer: %d -> %d, x: %f, y: %f, row: %d, col: %d, cost: %f, "
        "lower_cost: %f, "
        "upper_cost: %f, height: %f\n, hint: %f, lower_height: %f, "
        "upper_height: %f\n",
        layer, real_layer, x, y, index(y), col, cost_(row, col), lower_cost,
        upper_cost, height_(row, col), height_hint, lower_height, upper_height);
  }

  return real_layer;
}

// 获取指定栅格的高度
double DenseElevationMap::GetHeight(const int layer, const double x,
                                    const double y) {
  return height_(index_y_safe(y) + layer * max_y_, index_x_safe(x));
}

// 安全地获取高度，如果查询点的高度与提示高度差异过大，则返回提示高度
double DenseElevationMap::GetHeightSafe(const int layer, const double x,
                                        const double y,
                                        const double height_hint) {
  double new_height =
      height_(index_y_safe(y) + layer * max_y_, index_x_safe(x));

  if (abs(new_height - height_hint) > 6 * resolution_) {
    return height_hint;
  }

  return new_height;
}

// 获取指定栅格的天花板高度
double DenseElevationMap::GetCeiling(const int layer, const double x,
                                     const double y) {
  return ceiling_(index_y_safe(y) + layer * max_y_, index_x_safe(x));
}

// 使用双线性插值获取指定浮点坐标的成本值
double DenseElevationMap::GetValueBilinear(const int layer, const double x,
                                           const double y,
                                           Eigen::Vector2d* grad) {
  // 找到左下角的整数坐标
  double x_lb = std::max(std::floor(x - 0.5), 0.0);
  double y_lb = std::max(std::floor(y - 0.5), 0.0);

  // 获取周围四个点的成本值
  double value[2][2];
  for (int ix = 0; ix < 2; ++ix) {
    for (int iy = 0; iy < 2; ++iy) {
      value[ix][iy] = GetRealCost(layer, x_lb + ix, y_lb + iy, grad);
    }
  }

  // 计算插值权重
  Eigen::Vector2d diff(x - x_lb, y - y_lb);

  // 在x方向上进行两次线性插值
  double y0 = (1 - diff(0)) * value[0][0] + diff(0) * value[1][0];
  double y1 = (1 - diff(0)) * value[0][1] + diff(0) * value[1][1];
  // 在y方向上进行两次线性插值 (用于计算梯度)
  double x0 = (1 - diff(1)) * value[0][0] + diff(1) * value[0][1];
  double x1 = (1 - diff(1)) * value[1][0] + diff(1) * value[1][1];

  // 计算梯度
  if (grad) {
    (*grad)(0) = x1 - x0;
    (*grad)(1) = y1 - y0;
  }

  // 在y方向上对x方向的插值结果进行插值
  return (1 - diff(1)) * y0 + diff(1) * y1;
}

// 安全地使用双线性插值获取成本值，使用高度提示
double DenseElevationMap::GetValueBilinearSafe(const int layer, const double x,
                                               const double y,
                                               const double height_hint,
                                               Eigen::Vector2d* grad) {
  double x_lb = std::max(std::floor(x - 0.5), 0.0);
  double y_lb = std::max(std::floor(y - 0.5), 0.0);

  // 获取周围四个点的安全成本值
  double value[2][2];
  for (int ix = 0; ix < 2; ++ix) {
    for (int iy = 0; iy < 2; ++iy) {
      value[ix][iy] = GetRealCostSafe(layer, x_lb + ix, y_lb + iy, height_hint);
    }
  }

  Eigen::Vector2d diff(x - x_lb, y - y_lb);

  double y0 = (1 - diff(0)) * value[0][0] + diff(0) * value[1][0];
  double y1 = (1 - diff(0)) * value[0][1] + diff(0) * value[1][1];
  double x0 = (1 - diff(1)) * value[0][0] + diff(1) * value[0][1];
  double x1 = (1 - diff(1)) * value[1][0] + diff(1) * value[1][1];

  if (grad) {
    (*grad)(0) = x1 - x0;
    (*grad)(1) = y1 - y0;
  }

  return (1 - diff(1)) * y0 + diff(1) * y1;
}