#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import pickle
import numpy as np
import open3d as o3d
  
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from tomogram import Tomogram

# 将上级目录添加到系统路径，以便导入config模块
sys.path.append('../')
from config import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from config import Config

# 获取项目根目录
rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


# Tomography类：负责整个断层扫描图的构建流程，包括数据加载、处理、保存和可视化。
class Tomography(object):
    def __init__(self, cfg, scene_cfg):
        """
        初始化Tomography对象。

        Args:
            cfg (Config): 全局配置对象。
            scene_cfg (object): 特定场景的配置对象。
        """
        # 从配置中读取参数
        self.export_dir = rsg_root + cfg.map.export_dir
        self.pcd_file = scene_cfg.pcd.file_name
        self.resolution = scene_cfg.map.resolution
        self.ground_h = scene_cfg.map.ground_h
        self.slice_dh = scene_cfg.map.slice_dh

        self.center = np.zeros(2, dtype=np.float32)
        # 实例化Tomogram处理类
        self.tomogram = Tomogram(scene_cfg)
        # 加载点云数据
        points = self.loadPCD(self.pcd_file)

        # 处理点云，生成断层扫描图
        self.process(points)

    def initROS(self):
        """
        初始化ROS节点和发布者，用于可视化。
        """
        self.map_frame = cfg.ros.map_frame

        # 初始化原始点云发布者
        pointcloud_topic = cfg.ros.pointcloud_topic
        self.pointcloud_pub = rospy.Publisher(pointcloud_topic, PointCloud2, latch=True, queue_size=1)

        # 初始化地面层(G)和天花板层(C)的发布者列表
        self.layer_G_pub_list = []
        self.layer_C_pub_list = []
        layer_G_topic = cfg.ros.layer_G_topic
        layer_C_topic = cfg.ros.layer_C_topic
        for i in range(self.n_slice):
            layer_G_pub = rospy.Publisher(layer_G_topic + str(i), PointCloud2, latch=True, queue_size=1)
            self.layer_G_pub_list.append(layer_G_pub)
            layer_C_pub = rospy.Publisher(layer_C_topic + str(i), PointCloud2, latch=True, queue_size=1)
            self.layer_C_pub_list.append(layer_C_pub)

        # 初始化最终断层扫描图的发布者
        tomogram_topic = cfg.ros.tomogram_topic
        self.tomogram_pub = rospy.Publisher(tomogram_topic, PointCloud2, latch=True, queue_size=1)

    def loadPCD(self, pcd_file):
        """
        从文件加载PCD点云数据，并初始化地图参数。

        Args:
            pcd_file (str): PCD文件名。

        Returns:
            np.ndarray: 加载的点云数据。
        """
        pcd = o3d.io.read_point_cloud(rsg_root + "/rsc/pcd/" + pcd_file)
        points = np.asarray(pcd.points).astype(np.float32)
        rospy.loginfo("PCD points: %d", points.shape[0])

        # 确保点云是3维的
        if points.shape[1] > 3:
            points = points[:, :3]

        # 计算点云的边界和地图尺寸
        self.points_max = np.max(points, axis=0)
        self.points_min = np.min(points, axis=0)           
        self.points_min[-1] = self.ground_h
        self.map_dim_x = int(np.ceil((self.points_max[0] - self.points_min[0]) / self.resolution)) + 4
        self.map_dim_y = int(np.ceil((self.points_max[1] - self.points_min[1]) / self.resolution)) + 4
        n_slice_init = int(np.ceil((self.points_max[2] - self.points_min[2]) / self.slice_dh))
        self.center = (self.points_max[:2] + self.points_min[:2]) / 2
        self.slice_h0 = self.points_min[-1] + self.slice_dh

        # 初始化Tomogram对象的建图环境
        self.tomogram.initMappingEnv(self.center, self.map_dim_x, self.map_dim_y, n_slice_init, self.slice_h0)

        rospy.loginfo("Map center: [%.2f, %.2f]", self.center[0], self.center[1])
        rospy.loginfo("Dim_x: %d", self.map_dim_x)
        rospy.loginfo("Dim_y: %d", self.map_dim_y)
        rospy.loginfo("Num slices init: %d", n_slice_init)

        # 获取用于可视化的网格点原型
        self.VISPROTO_I, self.VISPROTO_P = \
            GRID_POINTS_XYZI(self.resolution, self.map_dim_x, self.map_dim_y)

        return points
        
    def process(self, points):
        """
        处理点云，生成断层扫描图，并进行性能基准测试。

        Args:
            points (np.ndarray): 输入的点云数据。
        """
        t_map = 0.0
        t_trav = 0.0
        t_simp = 0.0
        t_all = 0.0
        n_repeat = 10 # 重复运行次数，用于基准测试

        """ 
        GPU时间基准测试说明：
        为了获得准确的时间测量，这里使用了CUDA事件同步。
        函数会重复运行n_repeat次，以计算每个模块的平均处理时间。
        第一次运行（热身）的时间不计入，以减少计时波动和初始调用的开销。
        更多细节请参考: https://docs.cupy.dev/en/stable/user_guide/performance.html
        """
        for i in range(n_repeat + 1):
            t_start = time.time()
            # 核心处理步骤：调用Tomogram类将点云转换为地图
            layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, t_gpu = self.tomogram.point2map(points)

            # 从第二次运行开始累加时间
            if i > 0:
                t_map += t_gpu['t_map']
                t_trav += t_gpu['t_trav']
                t_simp += t_gpu['t_simp']
                t_all += (time.time() - t_start) * 1e3

        rospy.loginfo("Num slices simp: %d", layers_g.shape[0])
        rospy.loginfo("Num repeats (for benchmarking only): %d", n_repeat)
        rospy.loginfo(" -- avg t_map  (ms): %f", t_map / n_repeat)
        rospy.loginfo(" -- avg t_trav (ms): %f", t_trav / n_repeat)
        rospy.loginfo(" -- avg t_simp (ms): %f", t_simp / n_repeat)
        rospy.loginfo(" -- avg t_all  (ms): %f", t_all / n_repeat)

        self.n_slice = layers_g.shape[0]

        # 导出生成的断层扫描图
        map_file = os.path.splitext(self.pcd_file)[0]
        self.exportTomogram(np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c)), map_file)

        # 初始化ROS发布者并发布所有可视化信息
        self.initROS()
        self.publishPoints(points)
        self.publishLayers(self.layer_G_pub_list, layers_g, layers_t) # 发布地面层
        self.publishLayers(self.layer_C_pub_list, layers_c, None)    # 发布天花板层
        self.publishTomogram(layers_g, layers_t) # 发布最终的断层扫描图

    def exportTomogram(self, tomogram, map_file):
        """
        将生成的断层扫描图数据保存到pickle文件。

        Args:
            tomogram (np.ndarray): 包含所有图层数据的数组。
            map_file (str): 地图文件名（不含扩展名）。
        """
        data_dict = {
            'data': tomogram.astype(np.float16), # 存储为float16以节省空间
            'resolution': self.resolution,
            'center': self.center,
            'slice_h0': self.slice_h0,
            'slice_dh': self.slice_dh,
        }
        file_name = map_file + '.pickle'
        with open(self.export_dir + file_name, 'wb') as handle:
            pickle.dump(data_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

        rospy.loginfo("Tomogram exported: %s", file_name)

    def publishPoints(self, points):
        """
        将原始点云发布为ROS PointCloud2消息。
        """
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame

        point_msg = pc2.create_cloud_xyz32(header, points)
        self.pointcloud_pub.publish(point_msg)

    def publishLayers(self, pub_list, layers, color=None):
        """
        将单个图层（如地面、天花板）作为点云进行可视化发布。

        Args:
            pub_list (list): 对应的ROS发布者列表。
            layers (np.ndarray): 图层的高度数据。
            color (np.ndarray, optional): 图层的颜色/强度数据。
        """
        header = Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame

        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        for i in range(layers.shape[0]):
            # 将z值设置为图层的高度
            layer_points[:, 2] = layers[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            if color is not None:
                # 将强度值设置为颜色数据
                layer_points[:, 3] = color[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            else:
                layer_points[:, 3] = 1.0
        
            # 过滤掉无效点并发布
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, valid_points)
            pub_list[i].publish(points_msg) 

    def publishTomogram(self, layers_g, layers_t):
        """
        将最终的、经过简化的多层断层扫描图发布为单个PointCloud2消息。
        """
        header = Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame

        n_slice = layers_g.shape[0]
        vis_g = layers_g.copy() # 可视化的地面层
        vis_t = layers_t.copy() # 可视化的可通行性成本
        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        global_points = None
        # 合并所有图层为一个点云
        for i in range(n_slice - 1):
            # 过滤掉被上一层遮挡的点
            mask_h = (vis_g[i + 1] - vis_g[i]) < self.slice_dh
            vis_g[i, mask_h] = np.nan
            vis_t[i + 1, mask_h] = np.minimum(vis_t[i, mask_h], vis_t[i + 1, mask_h])

            layer_points[:, 2] = vis_g[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            layer_points[:, 3] = vis_t[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            if global_points is None:
                global_points = valid_points
            else:
                global_points = np.concatenate((global_points, valid_points), axis=0)

        # 添加最后一层
        layer_points[:, 2] = vis_g[-1, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
        layer_points[:, 3] = vis_t[-1, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
        valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
        global_points = np.concatenate((global_points, valid_points), axis=0)
        
        points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, global_points)
        self.tomogram_pub.publish(points_msg)


if __name__ == '__main__':
    import argparse

    # 设置命令行参数解析
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\']')
    args = parser.parse_args()

    # 加载配置
    cfg = Config()
    # 动态导入指定场景的配置
    scene_cfg = getattr(__import__('config'), 'Scene' + args.scene)

    # 初始化ROS节点
    rospy.init_node('pointcloud_tomography', anonymous=True)

    # 创建Tomography实例，开始处理
    mapping = Tomography(cfg, scene_cfg)

    # 保持节点运行
    rospy.spin()