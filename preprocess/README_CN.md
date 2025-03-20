# RS-AC-3DGS 数据预处理

## 概述

本仓库包含用于将 SLAM 数据转换为 3D 高斯散点（3D Gaussian Splatting, 3DGS）格式的预处理工具。它处理 ROS bag 文件和 LIVO SLAM 结果，以生成兼容的 3DGS 训练数据。

## 目录结构

```bash
rs_ac_3dgs/preprocess/
├── parser/
│   ├── src/
│   │   └── main.cpp
│   └── CMakeLists.txt
├── data/
│   └── [dataset_name]/
│       ├── filter_images/
│       ├── utm_V_opt_Cam_pose_W.txt
│       └── rgb_scan_all.pcd
└── slam2gsdata.py
```

## 先决条件

- ROS（机器人操作系统）
- Python 3.6+
- OpenCV
- CMake
- **RS LIVO SLAM 结果**

## 构建说明

1. 创建并构建解析器：
```bash
cd rs_ac_3dgs/preprocess/parser
mkdir build && cd build
cmake ..
make -j64
```

## 使用方法

### 1. 图像提取和位姿处理

首先，从 ROS bag 文件中提取图像，并处理 LIVO SLAM 结果的位姿数据：

```bash
cd rs_ac_3dgs/preprocess/
./parser/build/devel/lib/extract_images/parser <bag_file_path> <output_image_path> <pose_file_path> <sample_rate>
```

示例：
```bash
./parser/build/devel/lib/extract_images/parser /path/to/file.bag data/[dataset_name]/filter_images /path/to/utm_V_opt_Cam_pose_W.txt 2
```

ROS bag 话题（可能因版本更新而有所不同）：
- `/rs_camera/rgb`：RGB 图像
- `/rs_imu`：IMU 数据  
- `/rs_lidar/points`：点云数据

### 2. 数据准备

将 LIVO SLAM 结果文件复制到数据集目录：

```bash
cd rs_ac_3dgs/preprocess/
cp /path/to/utm_V_opt_Cam_pose_W.txt data/[dataset_name]
cp /path/to/rgb_map.pcd data/[dataset_name]
cp /path/to/calibration.yaml data/[dataset_name]
```

### 3. 生成 3DGS 数据

#### 运行转换脚本：

```bash
cd rs_ac_3dgs/preprocess
python3 slam2gsdata.py --data_dir data/[dataset_name]
```

该脚本的参数：
- `--data_dir`：数据集目录路径（例如 "data/dataset_name"），需确保包含以下内容：
  - `filter_images/`：包含提取的图像
  - `utm_V_opt_Cam_pose_W.txt`：LIVO SLAM 的相机位姿文件
  - `rgb_map.pcd`：LIVO SLAM 生成的点云地图
  - `calibration.yaml`：相机标定参数

## 深度数据解析

### 构建说明

```bash
cd rs_ac_3dgs/preprocess/catkin_depth_parser
catkin_make
```

### 使用方法

#### 1. 提取深度图

修改 `meta_config.yaml` 文件以更新 DEPTH_ROOT 和 SCENE_ROOT 路径。

示例：
```bash
DEPTH_ROOT: data/[dataset_name]/filter_depths/
SCENE_ROOT: data/[dataset_name]/scene/images
CALIBRATION_PATH: data/[dataset_name]/calibration.yaml
```

终端 1
```bash
cd rs_ac_3dgs/preprocess/catkin_depth_parser
source devel/setup.bash
roslaunch motion_correction motion_correction_metas.launch config_file:=/media/sti/beac7682-9b6e-492f-8983-5a9832652bca/code_rs/catkin_ws/src/motion_correct/config/meta_004_config.yaml
```

终端 2
```bash
rosbag play /path/to/bag/file.bag
```

提取的图像将存储在 `data/[dataset_name]/filter_depths/` 目录下。

#### 2. 添加深度图到 3DGS 数据

运行 `add_depth_to_gsdata.py` 脚本，将深度图整合到 3DGS 数据集中。

```bash
python3 add_depth_to_gsdata.py --data_dir data/[dataset_name]
```

#### 3. 验证深度图

运行 `test_depth_vaild.py` 脚本，验证深度图的有效性。

```bash
python3 test_depth_vaild.py --data_dir data/[dataset_name]
```

**注意**：恢复深度图至真实深度可参考 `test_depth_vaild.py` 脚本中的 `unwarpped_depth` 函数。

## 输出结果

完成预处理流程后，您将得到：
- 兼容 Colmap 格式的转换位姿数据

## 许可证

本项目采用 MIT 许可证，详情请参阅 [LICENSE](LICENSE) 文件。

## 参考

- [3D Gaussian Splatting](https://github.com/graphdeco-inria/gaussian-splatting)

