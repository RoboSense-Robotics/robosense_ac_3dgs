# RS-AC-3DGS

## Overview

RS-AC-3DGS provides tools to process and convert SLAM results from Active Camera to 3D Gaussian Splatting format. The toolkit then enables training using popular 3D Gaussian Splatting methods and visualization of the results.


## Installation

You can either use this repository or directly clone the official 3D Gaussian Splatting repository:
```bash
# Option 1: Use this repository
git clone https://github.com/RoboSense-Robotics/robosense_ac_3dgs.git

# Option 2: Use official repository
git clone https://github.com/graphdeco-inria/gaussian-splatting.git
```

For detailed installation instructions, please refer to the [3D Gaussian Splatting README](https://github.com/graphdeco-inria/gaussian-splatting#readme).


## Usage

### 1. Data Preprocessing

Follow the instructions in [preprocess/README.md](preprocess/README.md) to convert SLAM data to 3DGS format.

### 2. Training

```bash
cd gaussian-splatting
python train.py -s ../preprocess/data/[dataset_name]/scene
```

### 3. Visualization

For visualization setup, refer to the [Interactive Viewers](https://github.com/graphdeco-inria/gaussian-splatting#interactive-viewers) section in the original repository.

The gaussian-splatting/SIBR_viewers in this repository uses the **fossa_compatibility** branch. If you need to use other branches, you can download and install them from the official repository (https://gitlab.inria.fr/sibr/sibr_core.git).

```bash
cd gaussian-splatting/SIBR_viewers/install/bin
./SIBR_gaussianViewer_app -m /path/to/output/3D-Gaussian-Splatting/[exp_name] --rendering-size 1920 1080
```

#### Additional Visualization Tools

1. Video Recording:
   - Follow the tutorial at [YouTube Guide](https://www.youtube.com/watch?v=xxQr61ifoqM)

2. Online Viewer:
   - [SuperSplat Editor](https://playcanvas.com/supersplat/editor/) - Alternative web-based visualization tool

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [3D Gaussian Splatting](https://github.com/graphdeco-inria/gaussian-splatting)

## Contact

For questions and support, please [open an issue](http://gitlab.robosense.cn/super_sensor_sdk/ros2_sdk/rs_ac_3dgs/issues) on our GitHub repository.



