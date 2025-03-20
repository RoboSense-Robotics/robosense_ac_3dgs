## RS-AC-3DGS

### 概述

RS-AC-3DGS 提供工具，将 Active Camera 的 SLAM 结果转换为 3D Gaussian Splatting 格式。该工具包支持使用流行的 3D Gaussian Splatting 方法进行训练，并可视化结果。

### 安装

你可以选择使用本仓库，或者直接克隆官方的 3D Gaussian Splatting 仓库：

```bash
# 选项 1：使用本仓库
git clone git clone https://github.com/RoboSense-Robotics/robosense_ac_3dgs.git

# 选项 2：使用官方仓库
git clone https://github.com/graphdeco-inria/gaussian-splatting.git
```

详细的安装指南，请参考 [3D Gaussian Splatting README](https://github.com/graphdeco-inria/gaussian-splatting#readme)。

### 使用方法

#### 1. 数据预处理

请按照 [preprocess/README_CN.md](preprocess/README_CN.md) 中的说明，将 SLAM 数据转换为 3DGS 格式。

#### 2. 训练模型

```bash
cd gaussian-splatting
python train.py -s ../preprocess/data/[dataset_name]/scene
```

#### 3. 结果可视化

有关可视化设置，请参考官方仓库的 [Interactive Viewers](https://github.com/graphdeco-inria/gaussian-splatting#interactive-viewers) 章节。

本仓库中的 `gaussian-splatting/SIBR_viewers` 使用 **fossa_compatibility** 分支。如果你需要使用其他分支，可以从官方仓库下载并安装 (https://gitlab.inria.fr/sibr/sibr_core.git)。

```bash
cd gaussian-splatting/SIBR_viewers/install/bin
./SIBR_gaussianViewer_app -m /path/to/output/3D-Gaussian-Splatting/[exp_name] --rendering-size 1920 1080
```

### 其他可视化工具

- **视频录制**：
  - 请参考 [YouTube 教程](https://www.youtube.com/watch?v=xxQr61ifoqM)

- **在线查看**：
  - [SuperSplat Editor](https://playcanvas.com/supersplat/editor/) - 基于 Web 的可视化工具

### 许可证

本项目基于 MIT 许可证发布，详情请查看 [LICENSE](LICENSE) 文件。

### 鸣谢

- [3D Gaussian Splatting](https://github.com/graphdeco-inria/gaussian-splatting)

### 联系方式

如果有任何问题或需要支持，请在 [GitHub Issue](https://github.com/RoboSense-Robotics/robosense_ac_3dgs/issues) 页面提交问题。