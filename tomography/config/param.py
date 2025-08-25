# 定义ROS相关的配置
class ConfigROS():
    # 地图坐标系框架
    map_frame = "map"

    # 全局点云话题
    pointcloud_topic = "/global_points"
    # 地形可通行层（G层）话题
    layer_G_topic = "/layer_G_"
    # 天花板/障碍物层（C层）话题
    layer_C_topic = "/layer_C_"
    # 断层扫描图话题
    tomogram_topic = "/tomogram"


# 定义地图相关的配置
class ConfigMap():
    # 断层扫描图导出目录
    export_dir = "/rsc/tomogram/"


# 主配置类，整合所有配置
class Config():
    # ROS配置
    ros = ConfigROS()
    # 地图配置
    map = ConfigMap()