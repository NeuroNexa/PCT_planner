import sys
import argparse
import numpy as np
import os

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config

# --- 命令行参数解析 ---
parser = argparse.ArgumentParser()
parser.add_argument('--scene', type=str, help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\']')
# 添加 --pcd 参数，用于指定自定义PCD文件路径
parser.add_argument('--pcd', type=str, help='Path to a custom PCD file.')
args = parser.parse_args()

# --- 全局配置和变量 ---
cfg = Config()

# 根据参数选择对应的断层扫描图文件
if args.pcd:
    # 如果使用自定义PCD，则从路径中提取文件名作为断层扫描图的名称
    tomo_file = os.path.splitext(os.path.basename(args.pcd))[0]
elif args.scene:
    # 根据预设场景选择断层扫描图文件
    if args.scene == 'Spiral':
        tomo_file = 'spiral0.3_2'
    elif args.scene == 'Building':
        tomo_file = 'building2_9'
    else:
        tomo_file = 'plaza3_10'
else:
    # 如果两个参数都未提供，则打印错误信息并退出
    parser.error("Either --scene or --pcd argument must be provided.")
    sys.exit(1)

# ROS路径发布者
path_pub = rospy.Publisher("/pct_path", Path, latch=True, queue_size=1)
# 实例化规划器
planner = TomogramPlanner(cfg)

# 全局变量，用于存储起点和终点
start_pos = None
end_pos = None
click_count = 0

def point_callback(msg):
    """
    ROS回调函数，用于处理从RViz中点击的点。
    交替设置起点和终点。
    """
    global start_pos, end_pos, click_count

    click_count += 1

    # 第一次点击（奇数次）设置起点
    if click_count % 2 == 1:
        start_pos = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
        print(f"起点已设置为: ({start_pos[0]:.2f}, {start_pos[1]:.2f}, {start_pos[2]:.2f})")
        print("请在RViz中选择终点。")
    # 第二次点击（偶数次）设置终点并开始规划
    else:
        end_pos = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
        print(f"终点已设置为: ({end_pos[0]:.2f}, {end_pos[1]:.2f}, {end_pos[2]:.2f})")
        # 调用规划函数
        pct_plan()
        print("\n如需规划新路径，请在RViz中选择新的起点。")


def pct_plan():
    """
    执行路径规划并发布结果。
    """
    global start_pos, end_pos
    if start_pos is None or end_pos is None:
        print("未设置起点或终点。")
        return

    print("正在规划路径...")
    # 调用规划器核心的plan方法
    traj_3d = planner.plan(start_pos, end_pos)

    # 如果规划成功，发布轨迹
    if traj_3d is not None:
        path_pub.publish(traj2ros(traj_3d))
        print("轨迹已发布。")
    else:
        print("未能找到路径。")


if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node("pct_planner", anonymous=True)

    # 加载指定的断层扫描图
    planner.loadTomogram(tomo_file)

    # 订阅RViz中的 "/clicked_point" 话题
    rospy.Subscriber("/clicked_point", PointStamped, point_callback)

    print("规划器已初始化。请使用 'Publish Point' 在RViz中选择起点。")

    # 保持节点运行
    rospy.spin()