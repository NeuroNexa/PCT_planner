import numpy as np
import cupy as cp

from kernels import *

# Tomogram类封装了从点云生成多层可通行性地图（断层扫描图）的整个流程。
# 它利用CuPy和自定义的CUDA内核在GPU上进行加速计算。
class Tomogram(object):
    def __init__(self, cfg):
        """
        初始化Tomogram对象。

        Args:
            cfg (object): 包含地图和可通行性参数的配置对象。
        """
        # 从配置中读取参数
        self.resolution = cfg.map.resolution
        self.slice_dh = cfg.map.slice_dh
        self.half_trav_k_size = int(cfg.trav.kernel_size / 2)
        self.interval_min = cfg.trav.interval_min
        self.interval_free = cfg.trav.interval_free
        # 根据最大坡度计算可站立的台阶高度
        self.step_stand = 1.2 * self.resolution * np.tan(cfg.trav.slope_max)
        self.step_cross = cfg.trav.step_max
        # 计算可站立邻域的栅格数阈值
        self.standable_th = int(cfg.trav.standable_ratio * (2 * self.half_trav_k_size + 1) ** 2) - 1
        self.cost_barrier = float(cfg.trav.cost_barrier)
        self.safe_margin = cfg.trav.safe_margin
        self.inflation = cfg.trav.inflation
        # 计算膨胀操作的半核大小
        self.half_inf_k_size = int((self.safe_margin + self.inflation) / self.resolution)

    def initKernel(self):
        """
        初始化所有需要的CUDA内核。
        """
        # 初始化断层扫描图生成内核
        self.tomography_kernel = tomographyKernel(
            self.resolution, 
            self.map_dim_x, 
            self.map_dim_y,
            self.n_slice_init,
            self.slice_h0,
            self.slice_dh
        )

        # 初始化可通行性成本计算内核
        self.trav_kernel = travKernel(
            self.map_dim_x,
            self.map_dim_y,
            self.half_trav_k_size,
            self.interval_min,
            self.interval_free,
            self.step_cross, 
            self.step_stand, 
            self.standable_th, 
            self.cost_barrier
        )

        # 初始化成本图膨胀内核
        self.inflation_kernel = inflationKernel(
            self.map_dim_x,
            self.map_dim_y,
            self.half_inf_k_size
        )

        # 创建用于膨胀操作的查找表（score table）
        self.inf_table = cp.zeros(
            (2 * self.half_inf_k_size + 1, 2 * self.half_inf_k_size + 1), 
            dtype=cp.float32
        )
        for i in range(self.inf_table.shape[0]):
            for j in range(self.inf_table.shape[1]):
                dist = np.sqrt(
                    (self.resolution * (i - self.half_inf_k_size)) ** 2 + \
                    (self.resolution * (j - self.half_inf_k_size)) ** 2
                )
                # 计算权重，距离越近权重越高
                self.inf_table[i, j] = np.clip(
                    1 - (dist - self.inflation) / (self.safe_margin + self.resolution),
                    a_min=0.0, a_max=1.0
                )

    def initBuffers(self):
        """
        在GPU上初始化所有需要的缓冲区。
        """
        self.layers_g = cp.zeros((self.n_slice_init, self.map_dim_x, self.map_dim_y), dtype=cp.float32) # 地面高度层
        self.layers_c = cp.zeros((self.n_slice_init, self.map_dim_x, self.map_dim_y), dtype=cp.float32) # 天花板高度层
        self.grad_mag_sq = cp.zeros((self.n_slice_init, self.map_dim_x, self.map_dim_y), dtype=cp.float32) # 梯度平方
        self.grad_mag_max = cp.zeros((self.n_slice_init, self.map_dim_x, self.map_dim_y), dtype=cp.float32) # 最大梯度
        self.trav_cost = cp.zeros((self.n_slice_init, self.map_dim_x, self.map_dim_y), dtype=cp.float32) # 可通行性成本
        self.inflated_cost = cp.zeros((self.n_slice_init, self.map_dim_x, self.map_dim_y), dtype=cp.float32) # 膨胀后的成本

    def initMappingEnv(self, center, map_dim_x, map_dim_y, n_slice_init, slice_h0):
        """
        初始化建图环境，包括地图尺寸、中心点等，并初始化缓冲区和内核。
        """
        self.center = cp.array(center, dtype=cp.float32)
        self.map_dim_x = int(map_dim_x)
        self.map_dim_y = int(map_dim_y)
        self.n_slice_init = int(n_slice_init)
        self.slice_h0 = float(slice_h0)
        
        self.initBuffers()
        self.initKernel()

    def clearMap(self):
        """
        清空并重置所有GPU缓冲区，为下一次计算做准备。
        """
        self.layers_g *= 0.
        self.layers_c *= 0.
        self.layers_g += -1e6  # 初始化地面高度为一个很小的值
        self.layers_c += 1e6   # 初始化天花板高度为一个很大的值

        self.grad_mag_sq *= 0.
        self.grad_mag_max *= 0.
        self.trav_cost *= 0.
        self.inflated_cost *= 0.

    def point2map(self, points):
        """
        核心函数：将输入的点云转换为多层可通行性地图。

        Args:
            points (np.ndarray): 输入的点云数据 (N, 3)。

        Returns:
            tuple: 包含可通行性图、梯度、地面层、天花板层和GPU计时信息的元组。
        """
        # 将点云数据传输到GPU
        points = cp.asarray(points)
        # 移除包含NaN值的点
        points = points[~cp.isnan(points).any(axis=1)]
        self.clearMap()

        # --- 步骤1: 生成断层扫描图 (Tomogram) ---
        start_gpu = cp.cuda.Event()
        end_gpu = cp.cuda.Event()
        start_gpu.record()
        
        self.tomography_kernel(
            points, self.center, 
            self.layers_g, self.layers_c,
            size=(points.shape[0])
        )

        # 计算地面高度的梯度
        diff_x_sq = cp.maximum(
            (self.layers_g[:, 1:-1, :] - self.layers_g[:, :-2, :]) ** 2, 
            (self.layers_g[:, 1:-1, :] - self.layers_g[:,  2:, :]) ** 2
        )
        diff_y_sq = cp.maximum(
            (self.layers_g[:, :, 1:-1] - self.layers_g[:, :, :-2]) ** 2, 
            (self.layers_g[:, :, 1:-1] - self.layers_g[:, :,  2:]) ** 2
        )
        self.grad_mag_sq[:, 1:-1, 1:-1] = diff_x_sq[:, :, 1:-1] + diff_y_sq[:, 1:-1, :]
        self.grad_mag_max[:, 1:-1, 1:-1] = cp.maximum(diff_x_sq[:, :, 1:-1], diff_y_sq[:, 1:-1, :])
        
        # 计算垂直可通行空间 (天花板 - 地面)
        interval = (self.layers_c - self.layers_g)

        end_gpu.record()
        end_gpu.synchronize()
        gpu_t_map = cp.cuda.get_elapsed_time(start_gpu, end_gpu)

        # --- 步骤2: 计算可通行性 (Traversability) ---
        start_gpu = cp.cuda.Event()
        end_gpu = cp.cuda.Event()
        start_gpu.record()

        # 使用可通行性内核计算成本
        self.trav_kernel(
            interval, self.grad_mag_sq, self.grad_mag_max,
            self.trav_cost,
            size=(self.n_slice_init * self.map_dim_x * self.map_dim_y)
        )

        # 使用膨胀内核对成本图进行膨胀
        self.inflation_kernel(
            self.trav_cost, self.inf_table,
            self.inflated_cost,
            size=(self.n_slice_init * self.map_dim_x * self.map_dim_y)
        )

        end_gpu.record()
        end_gpu.synchronize()
        gpu_t_trav = cp.cuda.get_elapsed_time(start_gpu, end_gpu)

        # --- 步骤3: 层简化 (Layer Simplification) ---
        start_gpu = cp.cuda.Event()
        end_gpu = cp.cuda.Event()
        start_gpu.record()

        # 简化层，只保留具有显著变化的层
        idx_simp = [0]
        if self.layers_g.shape[0] > 1:
            l_idx, m_idx = 0, 1
            diff_h = self.layers_g[1:] - self.layers_g[:-1]
            while m_idx < self.n_slice_init - 2:
                mask_l_g = self.layers_g[m_idx] - self.layers_g[l_idx] > 0
                mask_l_t = self.inflated_cost[l_idx] > self.inflated_cost[m_idx]
                mask_u_g = diff_h[m_idx] > 0
                mask_t = self.inflated_cost[m_idx] < self.cost_barrier
                unique = (mask_l_g | mask_l_t) & mask_u_g & mask_t
                if cp.any(unique):
                    idx_simp.append(m_idx)
                    l_idx = m_idx
                m_idx += 1
            idx_simp.append(m_idx)

        # 计算简化后可通行性成本图的梯度
        trav_grad_x = (self.inflated_cost[idx_simp][:, 2:, :] - self.inflated_cost[idx_simp][:, :-2, :])
        trav_grad_y = (self.inflated_cost[idx_simp][:, :, 2:] - self.inflated_cost[idx_simp][:, :, :-2])
        
        end_gpu.record()
        end_gpu.synchronize()
        gpu_t_simp = cp.cuda.get_elapsed_time(start_gpu, end_gpu)
        
        gpu_t_all = gpu_t_map + gpu_t_trav + gpu_t_simp
        #print("CuPy GPU time (ms):", gpu_t_all)

        # --- 数据回传到CPU ---
        # 获取简化后的可通行性成本图
        layers_t = self.inflated_cost[idx_simp].get()
        # 获取简化后的地面高度层，无效值设为NaN
        layers_g = cp.where(
            self.layers_g[idx_simp] > -1e6, 
            self.layers_g[idx_simp],
            cp.nan
        ).get()
        # 获取简化后的天花板高度层，无效值设为NaN
        layers_c = cp.where(
            self.layers_c[idx_simp] < 1e6, 
            self.layers_c[idx_simp], 
            cp.nan
        ).get()
        # 获取可通行性成本的梯度
        trav_gx = np.zeros_like(layers_g)
        trav_gx[:, 1:-1, :] = trav_grad_x.get()
        trav_gy = np.zeros_like(layers_g)
        trav_gy[:, :, 1:-1] = trav_grad_y.get()

        # 整理GPU计时信息
        t_gpu = {
            't_map': gpu_t_map, 
            't_trav': gpu_t_trav, 
            't_simp': gpu_t_simp, 
        }
        
        return layers_t, trav_gx, trav_gy, layers_g, layers_c, t_gpu