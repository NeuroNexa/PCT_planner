# PCT Planner: 使用自定义PCD文件

## 概述

本文档旨在指导用户如何使用PCT Planner处理自定义的点云数据（PCD文件）。通过遵循这些步骤，您将能够为您自己的环境构建断层扫描图并生成导航轨迹。

## 功能改进

为了支持自定义PCD文件，我们进行了以下改进：

1.  **`--pcd` 命令行参数**: 在 `tomography.py` 和 `plan.py` 脚本中添加了 `--pcd` 参数，允许用户直接指定PCD文件的路径。
2.  **自定义场景配置**: 创建了 `tomography/config/scene_custom.py` 文件，用于管理自定义场景的参数。您可以根据需要修改此文件以微调您的场景。
3.  **动态路径处理**: 修改了代码，以正确处理绝对和相对PCD文件路径，并自动生成相应的断层扫描图文件名。

## 使用说明

### 1. 准备您的PCD文件

确保您的PCD文件可用，并且您拥有其完整路径。例如，您可以将PCD文件放置在项目的任何位置，或者使用系统中的绝对路径。

### 2. 构建断层扫描图

要为您的自定义PCD文件构建断层扫描图，请按照以下步骤操作：

- 启动 `roscore` 和 `RViz`（使用 `rsc/rviz/pct_ros.rviz` 配置）。
- 打开终端，导航到 `tomography/scripts/` 目录。
- 运行 `tomography.py` 脚本，并使用 `--pcd` 参数指定您的PCD文件路径。

**示例：**

```bash
cd tomography/scripts/
python3 tomography.py --pcd /path/to/your/custom.pcd
```

或者，如果您的PCD文件位于项目根目录下的 `my_pcds` 文件夹中：

```bash
python3 tomography.py --pcd ../../my_pcds/custom.pcd
```

脚本将加载您的PCD文件，生成断层扫描图，并将其保存到 `rsc/tomogram/` 目录下。文件名将根据您的PCD文件名自动生成（例如，`custom.pickle`）。

### 3. 生成轨迹

断层扫描图构建完成后，您可以为其生成导航轨迹：

- 确保 `roscore` 和 `RViz` 仍在运行。
- 打开一个新的终端，并设置 `LD_LIBRARY_PATH` 环境变量：

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/YOUR/DIRECTORY/TO/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib
```

- 导航到 `planner/scripts/` 目录。
- 运行 `plan.py` 脚本，并使用 `--pcd` 参数指定您在**上一步**中使用的相同PCD文件路径。

**示例：**

```bash
cd planner/scripts/
python3 plan.py --pcd /path/to/your/custom.pcd
```

- 现在，您可以在RViz中使用 "Publish Point" 工具选择起点和终点，规划器将自动为您生成并显示轨迹。

## 参数微调 (可选)

如果您发现默认参数不适用于您的特定环境，您可以修改 `tomography/config/scene_custom.py` 文件中的参数。可调整的参数包括：

- **地图参数 (`SceneMap`)**:
  - `resolution`: 地图分辨率。
  - `ground_h`: 地面高度。
  - `slice_dh`: 切片高度差。
- **可通行性参数 (`SceneTrav`)**:
  - `slope_max`: 最大可通过的坡度。
  - `step_max`: 最大可通过的台阶高度。
  - `safe_margin`: 安全边际。

根据您环境的特点（例如，车辆的爬坡能力、安全要求等）调整这些参数，以获得最佳的规划结果。
