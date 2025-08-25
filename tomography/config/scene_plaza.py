from .scene import ScenePCD, SceneMap, SceneTrav

# "Plaza" 场景的具体配置
# 这是一个复杂的室外广场场景，用于评估重复轨迹生成。
class ScenePlaza():
    # 点云数据配置
    pcd = ScenePCD()
    pcd.file_name = 'plaza3_10.pcd'  # 使用的点云文件名

    # 地图参数配置
    map = SceneMap()
    map.resolution = 0.10  # 地图分辨率
    map.ground_h = 0.0     # 地面高度
    map.slice_dh = 0.5     # 切片高度差

    # 可通行性分析参数配置
    trav = SceneTrav()
    trav.kernel_size = 7          # 邻域分析的卷积核大小
    trav.interval_min = 0.50      # 最小可通过垂直间隔
    trav.interval_free = 0.65     # 自由通过的垂直间隔
    trav.slope_max = 0.36         # 最大可通过坡度
    trav.step_max = 0.17          # 最大可通过台阶高度
    trav.standable_ratio = 0.2    # 可站立区域的最小比例
    trav.cost_barrier = 50.0      # 障碍物的代价值
    trav.safe_margin = 0.4        # 安全边际
    trav.inflation = 0.2          # 障碍物膨胀距离

