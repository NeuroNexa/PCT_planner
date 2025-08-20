import sys
import argparse
import numpy as np

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config

parser = argparse.ArgumentParser()
parser.add_argument('--scene', type=str, default='Spiral', help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\']')
args = parser.parse_args()

cfg = Config()

if args.scene == 'Spiral':
    tomo_file = 'spiral0.3_2'
elif args.scene == 'Building':
    tomo_file = 'building2_9'
else:
    tomo_file = 'plaza3_10'

path_pub = rospy.Publisher("/pct_path", Path, latch=True, queue_size=1)
planner = TomogramPlanner(cfg)

start_pos = None
end_pos = None
click_count = 0

def point_callback(msg):
    global start_pos, end_pos, click_count

    click_count += 1

    if click_count % 2 == 1:
        start_pos = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
        print(f"Start point set to: ({start_pos[0]:.2f}, {start_pos[1]:.2f}, {start_pos[2]:.2f})")
        print("Please select the end point in RViz.")
    else:
        end_pos = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
        print(f"End point set to: ({end_pos[0]:.2f}, {end_pos[1]:.2f}, {end_pos[2]:.2f})")
        pct_plan()
        print("\nTo plan a new path, please select a new start point in RViz.")


def pct_plan():
    global start_pos, end_pos
    if start_pos is None or end_pos is None:
        print("Start or end point not set.")
        return

    print("Planning path...")
    traj_3d = planner.plan(start_pos, end_pos)
    if traj_3d is not None:
        path_pub.publish(traj2ros(traj_3d))
        print("Trajectory published")
    else:
        print("Failed to find a path.")


if __name__ == '__main__':
    rospy.init_node("pct_planner", anonymous=True)

    planner.loadTomogram(tomo_file)

    rospy.Subscriber("/clicked_point", PointStamped, point_callback)

    print("Planner initialized. Please select the start point in RViz using 'Publish Point'.")

    rospy.spin()