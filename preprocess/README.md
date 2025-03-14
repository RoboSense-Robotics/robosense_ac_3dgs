# RS-AC-3DGS Data Preprocessing

## Overview

This repository contains preprocessing tools for converting SLAM data to 3D Gaussian Splatting format. It processes ROS bag files and LIVO SLAM results to generate compatible data for 3D Gaussian Splatting training.

## Directory Structure

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

## Prerequisites

- ROS (Robot Operating System)
- Python 3.6+
- OpenCV
- CMake
- **RS LIVO SLAM Result**

## Build Instructions

1. Create and build the parser:
```bash
cd rs_ac_3dgs/preprocess/parser
mkdir build && cd build
cmake ..
make -j64
```

## Usage

### 1. Image Extraction and Pose Processing

First, extract images from ROS bag files and process pose data from LIVO SLAM results:

```bash
cd rs_ac_3dgs/preprocess/
./parser/build/devel/lib/extract_images/parser <bag_file_path> <output_image_path> <pose_file_path> <sample_rate>
```

Example:

```bash
./parser/build/devel/lib/extract_images/parser /path/to/file.bag data/[dataset_name]/filter_images /path/to/utm_V_opt_Cam_pose_W.txt 2
```

ROS bag topics (may vary depending on version updates):
- `/rs_camera/rgb`: RGB images
- `/rs_imu`: IMU data  
- `/rs_lidar/points`: Point cloud data

### 2. Data Preparation

Copy required LIVO SLAM result files to the dataset directory:

```bash
cd rs_ac_3dgs/preprocess/
cp /path/to/utm_V_opt_Cam_pose_W.txt data/[dataset_name]
cp /path/to/rgb_map.pcd data/[dataset_name]
cp /path/to/calibration.yaml data/[dataset_name]
```

### 3. Make 3DGS data

#### Run the conversion script:

```bash
cd rs_ac_3dgs/preprocess
python3 slam2gsdata.py --data_dir data/[dataset_name]
```

The script takes the following arguments:
- `--data_dir`: Path to your dataset directory (e.g., "data/dataset_name") ,
confirm including:
  - `filter_images/`: Directory with extracted images
  - `utm_V_opt_Cam_pose_W.txt`: Camera pose file from LIVO SLAM
  - `rgb_map.pcd`: Point cloud map from LIVO SLAM 
  - `calibration.yaml`: Camera calibration parameters

## Depth Parser

### Build Instructions

```bash
cd rs_ac_3dgs/preprocess/catkin_depth_parser
catkin_make
```

### Usage

### 1. Extract Depth Maps
Modify the meta_config.yaml file to update the DEPTH_ROOT and SCENE_ROOT paths.

For example:
```bash
DEPTH_ROOT: data/[dataset_name]/filter_depths/
SCENE_ROOT: data/[dataset_name]/scene/images
CALIBRATION_PATH: data/[dataset_name]/calibration.yaml
```

Terminal 1
```bash
cd rs_ac_3dgs/preprocess/catkin_depth_parser
source devel/setup.bash
roslaunch motion_correction motion_correction_metas.launch config_file:=/media/sti/beac7682-9b6e-492f-8983-5a9832652bca/code_rs/catkin_ws/src/motion_correct/config/meta_004_config.yaml
```

Terminal 2
```bash
rosbag play /path/to/bag/file.bag
```

The extracted images will be stored in data/[dataset_name]/filter_depths/.

### 2. Add Depth Maps to 3DGS Data

Run the add_depth_to_gsdata.py script to integrate the depth maps into the 3DGS dataset.

```bash
python3 add_depth_to_gsdata.py --data_dir data/[dataset_name]
```

### 3. Test Depth Validity

Run the test_depth_vaild.py script to test the validity of the depth maps.

```bash
python3 test_depth_vaild.py --data_dir data/[dataset_name]
```

**Note: recovery the depth map and to real depths can refer to the function `unwarpped_depth` in the test_depth_vaild.py script.**

## Output

After running the preprocessing pipeline, you will get:
- Transformed pose data in Colmap-compatible format

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## References

- [3D Gaussian Splatting](https://github.com/graphdeco-inria/gaussian-splatting)


