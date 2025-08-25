import numpy as np
from sensor_msgs.msg import PointField

# 定义ROS PointCloud2消息中点的字段
# 包含x, y, z坐标和intensity（强度）值
POINT_FIELDS_XYZI = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('intensity', 12, PointField.FLOAT32, 1)
]


def GRID_POINTS_XYZI(resolution, dim_x, dim_y):
    """
    生成用于可视化的网格点原型。
    这个函数创建一个基础的网格，用于在RViz中显示地图的各个层。

    Args:
        resolution (float): 地图分辨率
        dim_x (int): 地图在x方向的维度
        dim_y (int): 地图在y方向的维度

    Returns:
        tuple:
            - index_proto (np.ndarray): 网格点的索引数组 (N, 2)
            - point_proto (np.ndarray): 网格点的坐标数组 (N, 4)，格式为 (x, y, z, intensity)
    """
    # 创建网格索引
    index_proto = np.zeros((dim_x * dim_y, 2), dtype=int)
    lx = np.linspace(0, dim_x - 1, dim_x, dtype=int)
    ly = np.linspace(0, dim_y - 1, dim_y, dtype=int)
    ix, iy = np.meshgrid(lx, ly)
    index_proto[:, 0] = ix.flatten()
    index_proto[:, 1] = iy.flatten()

    # 创建点坐标原型
    point_proto = np.zeros((dim_x * dim_y, 4), dtype=np.float32)
    point_proto[:, :2] = index_proto[:, :2].astype(np.float32, copy=True)
    # 将坐标中心移到 (0,0)
    point_proto[:, 0] -= 0.5 * dim_x
    point_proto[:, 1] -= 0.5 * dim_y
    # 应用分辨率
    point_proto[:, :2] *= resolution
    # 默认强度值为1.0
    point_proto[:, 3] = 1.0

    return index_proto, point_proto