from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def traj2ros(traj):
    """
    将轨迹数组转换为ROS的nav_msgs/Path消息。

    Args:
        traj (np.ndarray): N x 3 的轨迹数组，每一行代表一个路径点 (x, y, z)。

    Returns:
        nav_msgs.msg.Path: 用于在RViz中可视化的ROS路径消息。
    """
    # 创建一个Path消息实例
    path_msg = Path()
    # 设置消息头，指定坐标系为"map"
    path_msg.header.frame_id = "map"

    # 遍历轨迹中的每一个路径点
    for waypoint in traj:
        # 为每个路径点创建一个PoseStamped消息
        pose = PoseStamped()
        pose.header.frame_id = "map"
        # 设置位置坐标
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = waypoint[2]
        # 设置姿态，这里使用单位四元数，表示没有旋转
        pose.pose.orientation.w = 1
        # 将位姿添加到路径消息中
        path_msg.poses.append(pose)

    return path_msg