from .scene import ScenePCD, SceneMap, SceneTrav
import os

# 自定义场景的具体配置
class SceneCustom():
    # 点云数据配置
    pcd = ScenePCD()
    # 地图参数配置
    map = SceneMap()
    # 可通行性分析参数配置
    trav = SceneTrav()

    def __init__(self):
        """
        初始化自定义场景配置。
        这里的参数是通用的默认值，可以根据需要进行调整。
        """
        # 默认点云文件名，将在 `update` 方法中被覆盖
        self.pcd.file_name = None

        # 默认地图参数
        self.map.resolution = 0.10  # 地图分辨率
        self.map.ground_h = 0.0     # 地面高度
        self.map.slice_dh = 0.5     # 切片高度差

        # 默认可通行性分析参数
        self.trav.kernel_size = 7          # 邻域分析的卷积核大小
        self.trav.interval_min = 0.50      # 最小可通过垂直间隔
        self.trav.interval_free = 0.65     # 自由通过的垂直间隔
        self.trav.slope_max = 0.40         # 最大可通过坡度
        self.trav.step_max = 0.17          # 最大可通过台阶高度
        self.trav.standable_ratio = 0.20   # 可站立区域的最小比例
        self.trav.cost_barrier = 50.0      # 障碍物的代价值
        self.trav.safe_margin = 0.4        # 安全边际
        self.trav.inflation = 0.2          # 障碍物膨胀距离

    def update(self, pcd_path):
        """
        根据提供的PCD文件路径更新场景配置。

        Args:
            pcd_path (str): 自定义PCD文件的路径。
        """
        # 使用提供的路径更新PCD文件名
        self.pcd.file_name = pcd_path
        print(f"场景已更新，使用PCD文件: {self.pcd.file_name}")
