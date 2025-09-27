# Introduction
本工程为点云、IMU、相机运动补偿模块

# Install
将代码放在catkin工程的src目录下
```
catkin_make
```

# Run
```
source ./devel/setup.bash
roslaunch demo_lidar2cam.launch config_file:=/path/to/your/config.yaml
```

# Config
关于配置文件的说明见 `config/robot_config.yaml`