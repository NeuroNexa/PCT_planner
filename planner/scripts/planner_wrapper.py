import os
import sys
import pickle
import numpy as np

from utils import *

# 导入C++绑定的库
sys.path.append('../')
from lib import a_star, ele_planner, traj_opt

# 获取项目根目录
rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'

# TomogramPlanner类：一个封装了整个规划流程的Python接口。
# 它负责加载地图、初始化C++规划器、调用规划算法并将结果转换回世界坐标系。
class TomogramPlanner(object):
    def __init__(self, cfg):
        """
        初始化规划器封装器。

        Args:
            cfg (Config): 规划器的配置对象。
        """
        self.cfg = cfg

        # 从配置中读取参数
        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate
        self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir

        # 初始化地图和规划器相关的成员变量
        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None
        self.elev_g = None # 地面高程图

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)

    def loadTomogram(self, tomo_file):
        """
        从pickle文件加载预处理好的断层扫描图。

        Args:
            tomo_file (str): 断层扫描图文件名 (不含扩展名)。
        """
        with open(self.tomo_dir + tomo_file + '.pickle', 'rb') as handle:
            data_dict = pickle.load(handle)

            # 解析数据
            tomogram = np.asarray(data_dict['data'], dtype=np.float32)
            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        # 分离出不同的数据层
        trav = tomogram[0]      # 可通行性成本图
        trav_gx = tomogram[1]   # 可通行性成本图的x方向梯度
        trav_gy = tomogram[2]   # 可通行性成本图的y方向梯度
        elev_g = tomogram[3]    # 地面高程图
        self.elev_g = np.nan_to_num(elev_g, nan=-100) # 将NaN替换为-100
        elev_c = tomogram[4]    # 天花板高程图
        elev_c = np.nan_to_num(elev_c, nan=1e6) # 将NaN替换为极大值

        # 使用加载的地图数据初始化C++规划器
        self.initPlanner(trav, trav_gx, trav_gy, self.elev_g, elev_c)
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        """
        根据地图数据，初始化C++核心规划器。
        """
        # --- 计算 "gateway" ---
        # Gateway用于识别可以切换层的区域（如斜坡、楼梯）。
        # 这是通过比较相邻层的可通行性成本和高程差异来实现的。
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        # 向上走的gateway
        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0 # 可通行性成本显著降低
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:])) # 高程差异小
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        # 向下走的gateway
        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0 # 可通行性成本显著增加
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1])) # 高程差异小
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2  # 向上标记
        gateway[gateway_dn] = -2 # 向下标记

        # 实例化C++规划器对象
        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        # 调用C++对象的init_map方法，传入所有地图数据
        self.planner.init_map(
            20, 15, self.resolution, self.n_slice, 0.2,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
        )

    def plan(self, start_pos, end_pos):
        """
        执行完整的路径规划流程。

        Args:
            start_pos (np.ndarray): 起点坐标 [x, y, z]。
            end_pos (np.ndarray): 终点坐标 [x, y, z]。

        Returns:
            np.ndarray or None: 规划成功则返回3D轨迹点数组，否则返回None。
        """
        # 将世界坐标转换为栅格索引
        self.start_idx = self.pos_to_idx_3d(start_pos)
        self.end_idx = self.pos_to_idx_3d(end_pos)

        print(f"Start index: {self.start_idx}")
        print(f"End index: {self.end_idx}")

        # 调用C++规划器的plan方法
        self.planner.plan(self.start_idx, self.end_idx, True)

        # --- 获取A*搜索结果 ---
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            print("A* search failed to find a path.")
            return None

        # --- 获取轨迹优化结果 ---
        # 根据配置选择不同的优化器
        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix() # 优化后的轨迹
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        if len(traj_raw) == 0:
            print("Trajectory optimization failed.")
            return None

        # --- 结果后处理 ---
        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        # 将轨迹从栅格坐标转换为世界坐标
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        return traj_3d

    def pos_to_idx_3d(self, pos):
        """
        将3D世界坐标 (x, y, z) 转换为3D栅格索引 (slice, u, v)。
        """
        # 首先，从(x, y)获取2D栅格索引(u, v)
        idx_2d = self.pos2idx(pos[:2])
        u, v = idx_2d[0], idx_2d[1]

        z_pos = pos[2]

        # 找到在(u, v)位置处，高程最接近z_pos的层的索引
        if self.elev_g is not None:
            # 获取(u, v)单元在所有层的高程值
            # 注意: Numpy索引是(row, col)，对应(v, u)。
            heights_at_cell = self.elev_g[:, v, u]
            # 找到绝对差最小的层的索引
            slice_idx = np.argmin(np.abs(heights_at_cell - z_pos))
        else:
            # 如果高程图不可用，则使用旧方法
            slice_idx = int(round((z_pos - self.slice_h0) / self.slice_dh))
            # 确保索引在有效范围内
            slice_idx = np.clip(slice_idx, 0, self.n_slice - 1)

        return np.array([slice_idx, u, v], dtype=np.int32)

    def pos2idx(self, pos):
        """
        将2D世界坐标 (x, y) 转换为2D栅格索引 (u, v)。
        """
        pos_relative = pos - self.center
        idx = np.round(pos_relative / self.resolution).astype(np.int32) + self.offset
        # 注意：这里的(u, v)对应于(y, x)或(row, col)的索引顺序。
        # planner的(u, v)可能对应numpy的(y, x)访问。
        idx = np.array([idx[1], idx[0]], dtype=np.int32) # u, v
        return idx