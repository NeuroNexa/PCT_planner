# 定义规划器相关的配置
class ConfigPlanner():
    # 是否使用五次多项式曲线进行轨迹优化。
    # 如果为True，则使用一种不考虑航向角（no-heading-objective）的优化器。
    # 如果为False，则使用另一种优化器。
    use_quintic = True
    # 最大航向变化率（单位可能是 rad/s 或 deg/s，需要根据C++代码确认）
    max_heading_rate = 10


# 定义规划器封装器（Wrapper）相关的配置
class ConfigWrapper():
    # 存储断层扫描图（tomogram）结果的目录
    tomo_dir = '/rsc/tomogram/'


# 主配置类，整合所有配置
class Config():
    # 规划器配置
    planner = ConfigPlanner()
    # 封装器配置
    wrapper = ConfigWrapper()