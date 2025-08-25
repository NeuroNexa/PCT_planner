# 定义场景的点云数据配置
class ScenePCD():
    # 点云文件名
    file_name = None


# 定义场景的地图参数配置
class SceneMap():
    # 地图分辨率 (米/像素)
    resolution = 0.10
    # 地面高度 (米)
    ground_h = 0.0
    # 切片高度差 (米)
    slice_dh = 0.5


# 定义可通行性分析的参数配置
class SceneTrav():
    # 卷积核大小，用于分析邻域
    kernel_size = 7
    # 最小可通过垂直间隔 (米)
    interval_min = 0.50
    # 可自由通过的垂直间隔 (米)
    interval_free = 0.65
    # 最大可通过坡度
    slope_max = 0.36
    # 最大可通过台阶高度 (米)
    step_max = 0.20
    # 可站立区域的最小比例
    standable_ratio = 0.20
    # 障碍物的代价值
    cost_barrier = 50.0

    # 安全边际 (米)，用于路径规划
    safe_margin = 0.4
    # 膨胀距离 (米)，用于障碍物膨胀
    inflation = 0.2