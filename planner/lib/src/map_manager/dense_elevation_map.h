#pragma once // 防止头文件被重复包含

#include <Eigen/Dense> // 引入Eigen库，用于矩阵和向量操作

/**
 * @class DenseElevationMap
 * @brief 管理一个密集的多层高程图。
 *
 * 这个类存储了环境的多个属性，如成本、高程、天花板高度和梯度。
 * 它提供了通过双线性插值等方法安全地访问这些地图数据的功能。
 * 地图数据被存储为Eigen::MatrixXd矩阵，其中行索引结合了层(layer)和y坐标。
 */
class DenseElevationMap {
 public:
  DenseElevationMap() = default; // 默认构造函数
  ~DenseElevationMap() = default; // 默认析构函数

  /**
   * @brief 初始化地图。
   * @param resolution 地图分辨率。
   * @param num_layers 地图的层数。
   * @param cost_map 成本图。
   * @param ele_mask 高程掩码，用于标记有效的高程区域。
   * @param height 地面高度图。
   * @param ceiling 天花板高度图。
   * @param grad_x 成本图在x方向的梯度。
   * @param grad_y 成本图在y方向的梯度。
   */
  void Init(const double resolution, const int num_layers,
            const Eigen::MatrixXd& cost_map, const Eigen::MatrixXd& ele_mask,
            const Eigen::MatrixXd& height, const Eigen::MatrixXd& ceiling,
            const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y);

  /**
   * @brief 使用双线性插值获取指定位置的成本值。
   * @param layer 初始层索引。
   * @param x, y 查询点的x, y坐标（栅格坐标）。
   * @param grad (可选) 返回计算出的梯度。
   * @return 插值得到的成本值。
   */
  double GetValueBilinear(const int layer, const double x, const double y,
                          Eigen::Vector2d* grad = nullptr);

  /**
   * @brief 安全地使用双线性插值获取成本值，会根据高度提示自动选择正确的层。
   * @param layer 初始层索引。
   * @param x, y 查询点的x, y坐标。
   * @param height_hint 高度提示，用于在多层之间选择最合适的层。
   * @param grad (可选) 返回计算出的梯度。
   * @return 插值得到的成本值。
   */
  double GetValueBilinearSafe(const int layer, const double x, const double y,
                              const double height_hint,
                              Eigen::Vector2d* grad = nullptr);

  /**
   * @brief 根据位置更新层索引。
   * @return 更新后的层索引。
   */
  int UpdateLayer(const int layer, const double x, const double y);

  /**
   * @brief 安全地根据位置和高度提示更新层索引。
   * @return 更新后的层索引。
   */
  int UpdateLayerSafe(const int layer, const double x, const double y,
                      const double height_hint);

  /**
   * @brief 获取指定位置的高度。
   * @return 高度值。
   */
  double GetHeight(const int layer, const double x, const double y);

  /**
   * @brief 安全地获取指定位置的高度。
   * @return 高度值。
   */
  double GetHeightSafe(const int layer, const double x, const double y,
                       const double height_hint);

  /**
   * @brief 获取指定位置的天花板高度。
   * @return 天花板高度值。
   */
  double GetCeiling(const int layer, const double x, const double y);

  /**
   * @brief 内联函数，获取指定位置的名义成本（不进行插值）。
   * @return 对应栅格的成本值。
   */
  double inline GetNominalCost(int layer, double x, double y) {
    auto idx = CoordsToIndex(layer, x, y);
    return cost_(idx[0], idx[1]);
  };

  /**
   * @brief 内联函数，将(层, x, y)坐标转换为矩阵的行列索引。
   * @return 包含行和列索引的数组 {row, col}。
   */
  std::array<int, 2> inline CoordsToIndex(int layer, double x, double y) {
    int col = index(x);
    int row = index(y) + layer * max_y_;
    return {row, col};
  }

  /**
   * @brief 设置调试模式。
   * @param flag true表示开启调试，false表示关闭。
   */
  void SetDebug(const bool flag) { debug_ = flag; }

 private:
  // 将浮点坐标转换为整数索引
  int inline index(double coord) { return static_cast<int>(coord); }

  // 安全地获取x方向的索引，防止越界
  int inline index_x_safe(double coord) {
    return std::min(std::max(index(coord), 0), max_x_ - 1);
  }

  // 安全地获取y方向的索引，防止越界
  int inline index_y_safe(double coord) {
    return std::min(std::max(index(coord), 0), max_y_ - 1);
  }

  // 获取真实成本（内部函数），可能会更新层
  double GetRealCost(int layer, double x, double y,
                     Eigen::Vector2d* grad = nullptr,
                     int* real_layer = nullptr);

  // 安全地获取真实成本（内部函数）
  double GetRealCostSafe(int layer, double x, double y,
                         const double height_hint);

 private:
  // --- 成员变量 ---
  bool debug_ = false; // 调试标志
  double resolution_ = 0.0; // 地图分辨率
  double resolution_inv_ = 0.0; // 分辨率的倒数
  int max_layers_ = 0; // 最大层数
  int max_x_ = 0; // x方向最大索引
  int max_y_ = 0; // y方向最大索引
  int xy_size_ = 0; // 单层地图的大小 (max_x * max_y)
  double offset_ = 0; // 坐标偏移

  double safe_cost_threshold_ = 10; // 安全成本阈值

  // 地图数据矩阵
  Eigen::MatrixXd cost_;     // 成本图
  Eigen::MatrixXd ele_mask_; // 高程掩码
  Eigen::MatrixXd height_;   // 高度图
  Eigen::MatrixXd ceiling_;  // 天花板图
  Eigen::MatrixXd grad_x_;   // x方向梯度
  Eigen::MatrixXd grad_y_;   // y方向梯度
};