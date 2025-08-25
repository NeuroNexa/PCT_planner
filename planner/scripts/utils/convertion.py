import numpy as np

def transTrajGrid2Map(grid_dim, center, resolution, traj_grid):
    """
    将轨迹从栅格坐标系转换到地图（世界）坐标系。

    Args:
        grid_dim (list/tuple): 栅格地图的维度 [rows, cols] or [y, x]。
        center (list/tuple): 地图中心在世界坐标系下的坐标 [x, y]。
        resolution (float): 地图分辨率 (米/栅格)。
        traj_grid (np.ndarray): N x 3 的数组，表示在栅格坐标系下的轨迹点 (v, u, z_grid)，
                                其中 v 对应行（y），u 对应列（x）。

    Returns:
        np.ndarray: N x 3 的数组，表示在地图坐标系下的轨迹点 (x, y, z)。
    """
    # 注意：这里的索引和坐标轴对应关系比较微妙
    # grid_dim[0] 是 y 方向的维度 (rows)
    # grid_dim[1] 是 x 方向的维度 (cols)
    # traj_grid 的坐标是 (v, u, z_grid)，v 对应 y，u 对应 x

    # 计算栅格坐标系的原点偏移量
    # offset 的顺序是 [offset_v, offset_u, 0]，对应 [y_offset, x_offset, 0]
    offset = np.array([grid_dim[0] // 2, grid_dim[1] // 2, 0])

    # 世界坐标系的中心点，注意顺序是 [center_x, center_y]
    # center_ 的顺序是 [center_y, center_x, 0.5]
    center_ = np.array([center[1], center[0], 0.5])

    # 1. 将栅格坐标的原点移动到中心
    # 2. 按分辨率进行缩放
    # 3. 移动到世界坐标系的正确位置
    # 注意 traj_grid 的列顺序 (v, u, z) 减去 offset 的列顺序 (v, u, z)
    traj_map_intermediate = (traj_grid - offset) * resolution + center_

    # traj_map_intermediate 的列顺序是 (y_map, x_map, z_map)
    # 需要转换成标准的 (x, y, z) 顺序
    traj_map = np.stack([traj_map_intermediate[:, 1], traj_map_intermediate[:, 0], traj_map_intermediate[:, 2]], axis=1)

    return traj_map
