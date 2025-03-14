#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SLAM数据转换为3D Gaussian Splatting格式的工具

本工具用于将SLAM系统的输出（点云和相机轨迹）转换为3D Gaussian Splatting训练所需的格式。
支持处理点云降采样、图像去畸变，以及坐标系转换等功能。
"""

import os
import cv2
import collections
import open3d as o3d
import numpy as np
import pandas as pd
import yaml
import argparse
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
from concurrent.futures import ProcessPoolExecutor, as_completed

# 定义数据结构
Camera = collections.namedtuple(
    "Camera", ["id", "model", "width", "height", "params"])
Image = collections.namedtuple(
    "Image", ["id", "qvec", "tvec", "camera_id", "name", "xys", "point3D_ids"])
Point3D = collections.namedtuple(
    "Point3D", ["id", "xyz", "rgb", "error", "image_ids", "point2D_idxs"])

def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='SLAM数据转3DGS格式工具')
    parser.add_argument('--data_dir', type=str, required=True,
                      help='数据目录，包含filter_images、pose文件和点云文件')
    return parser.parse_args()

def read_pcd(file_path, voxel_size=0.05):
    """
    读取并降采样点云数据

    参数:
        file_path (str): 点云文件路径
        voxel_size (float): 体素大小，用于降采样

    返回:
        points (np.ndarray): 点云坐标 (N,3)
        colors (np.ndarray): 点云颜色 (N,3)，如果存在的话
    """
    pcd = o3d.io.read_point_cloud(file_path)
    print(f"读取到 {len(np.asarray(pcd.points))} 个点")
    
    # 体素降采样
    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    points = np.asarray(down_pcd.points)
    print(f"降采样后剩余 {len(points)} 个点")

    if down_pcd.has_colors():
        colors = np.asarray(down_pcd.colors)
        return points, colors
    else:
        return points, None

def process_pointcloud(config, pcmap_path, pose_dict):
    """处理点云数据，生成3DGS格式的points3D和images数据"""
    points, colors = read_pcd(pcmap_path)
    if colors is None:
        raise ValueError("点云文件中没有颜色信息")

    # 生成points3D字典
    points3D = {}
    for i, (point, color) in enumerate(tqdm(zip(points, colors), total=len(points), desc="处理点云")):
        point_id = i + 1
        points3D[point_id] = Point3D(
            id=point_id,
            xyz=np.array(point),
            rgb=(np.array(color)*255).astype(np.uint8),
            error=np.array(1.5),
            image_ids=np.array([1]),
            point2D_idxs=np.array([point_id])
        )

    # 生成images字典
    images = {}
    for image_id, (timestamp, info) in enumerate(tqdm(list(pose_dict.items()), desc="处理图像位姿"), 1):
        xys = np.array([(config['cam_width']//2, config['cam_height']//2) for _ in range(len(points))]) if image_id == 1 else np.empty((0, 2))
        point3D_ids = np.array([i for i in range(1, len(points)+1)]) if image_id == 1 else np.empty((0,))
        
        images[image_id] = Image(
            id=image_id,
            qvec=np.array(info["orientation"]),
            tvec=np.array(info["position"]),
            camera_id=1,
            name=info["image_name"],
            xys=xys,
            point3D_ids=point3D_ids
        )

    print(f"处理完成: {len(points3D)} 个点, {len(images)} 张图像")
    return points3D, images

def process_image(args):
    """处理单张图像的去畸变"""
    input_path, params, output_path = args
    mapx, mapy, _ = params  # ROI不再使用
    
    try:
        img = cv2.imread(input_path)
        if img is None:
            print(f"无法读取图像: {input_path}")
            return
            
        # 使用remap进行去畸变
        undistorted_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
        cv2.imwrite(output_path, undistorted_img)
    except Exception as e:
        print(f"处理图像 {input_path} 时出错: {e}")

def process_images(config, image_folder, output_folder):
    """批量处理图像去畸变"""
    # 构建相机矩阵
    w, h = config['cam_width'], config['cam_height']
    camera_matrix = np.array([
        [config['cam_fx'], 0, config['cam_cx']],
        [0, config['cam_fy'], config['cam_cy']],
        [0, 0, 1]
    ])
    
    # 畸变系数
    dist_coeffs = np.array([
        config.get('cam_d0', 0.0),
        config.get('cam_d1', 0.0),
        config.get('cam_d2', 0.0),
        config.get('cam_d3', 0.0),
        config.get('cam_d4', 0.0),
        config.get('cam_d5', 0.0),
        config.get('cam_d6', 0.0),
        config.get('cam_d7', 0.0),
    ])
    
    # 使用单位矩阵作为R矩阵
    R = np.eye(3, dtype=np.float64)
    
    # 直接使用原始相机矩阵作为新的相机矩阵（与C++版本保持一致）
    new_camera_matrix = camera_matrix.copy()
    
    # 初始化去畸变映射（与C++版本保持一致）
    mapx, mapy = cv2.initUndistortRectifyMap(
        camera_matrix,  # 原始相机内参
        dist_coeffs,    # 畸变系数
        R,              # 单位矩阵作为校正矩阵
        camera_matrix,  # 使用原始相机矩阵作为新的投影矩阵
        (w, h),        # 图像尺寸
        cv2.CV_32FC1   # 与C++版本的CV_16SC2不同，但效果相同
    )

    # 打印相机参数和畸变矫正参数
    print("\n相机内参和畸变矫正参数:")
    print(f"原始相机内参矩阵:\n{camera_matrix}")
    print(f"畸变系数: {dist_coeffs}")
    print(f"图像尺寸: {w}x{h}")

    # 并行处理图像
    os.makedirs(output_folder, exist_ok=True)
    image_names = os.listdir(image_folder)
    
    with ProcessPoolExecutor(max_workers=32) as executor:
        futures = []
        for image_name in image_names:
            input_path = os.path.join(image_folder, image_name)
            output_path = os.path.join(output_folder, image_name)
            futures.append(executor.submit(
                process_image, 
                (input_path, (mapx, mapy, None), output_path)  # ROI设为None，不进行裁剪
            ))

        for future in tqdm(as_completed(futures), total=len(image_names), desc="图像去畸变"):
            future.result()
    
    # 返回新的相机参数
    return {1: Camera(id=1, model='PINHOLE', 
                     width=w, height=h,
                     params=[camera_matrix[0,0], camera_matrix[1,1],
                            camera_matrix[0,2], camera_matrix[1,2]])}

def transform_pose(q, t):
    """
    转换位姿数据

    参数:
        q (list): 四元数 [qw, qx, qy, qz]
        t (list): 平移向量 [x, y, z]
        I2C (bool): 是否进行IMU到相机的转换

    返回:
        tuple: (转换后的四元数, 转换后的平移向量)
    """
    # 四元数取逆
    q_inv = np.array([q[0], -q[1], -q[2], -q[3]])
    # 转换为scipy的旋转表示
    rotation = R.from_quat([q_inv[1], q_inv[2], q_inv[3], q_inv[0]])  # [x, y, z, w]
    t_inv = -rotation.as_matrix() @ np.array(t).reshape(3, 1)
    t_inv = t_inv.reshape(-1)
    
    return q_inv, t_inv

def main():
    args = parse_args()
    
    # 检查输入文件
    source_image_folder = os.path.join(args.data_dir, "filter_images")
    pose_file = os.path.join(args.data_dir, "utm_V_opt_Cam_pose_W.txt")
    pcmap_path = os.path.join(args.data_dir, "rgb_map.pcd")
    config_path = os.path.join(args.data_dir, "calibration.yaml")
    
    for path in [source_image_folder, pose_file, pcmap_path, config_path]:
        if not os.path.exists(path):
            raise FileNotFoundError(f"找不到文件: {path}")
    
    # 读取相机参数
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    config = config["ADDTION_INFO"]
    print(config)

    # 创建输出目录
    scene_dir = os.path.join(args.data_dir, "scene")
    os.makedirs(scene_dir, exist_ok=True)
    
    output_image_folder = os.path.join(scene_dir, "images")
    os.makedirs(output_image_folder, exist_ok=True)
    
    sparse_dir = os.path.join(scene_dir, "sparse", "0")
    os.makedirs(sparse_dir, exist_ok=True)
    
    # 读取位姿数据
    pose_data = pd.read_csv(pose_file, delim_whitespace=True, header=None,
                           names=["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
    
    pose_dict = {
        row["timestamp"]: {
            "position": [row["x"], row["y"], row["z"]],
            "orientation": [row["qw"], row["qx"], row["qy"], row["qz"]],
            "image_name": f"{int(row['timestamp'] * 1e6)}000.jpg"
        }
        for _, row in pose_data.iterrows()
    }
    
    # 过滤掉没有图像的位姿
    filtered_pose_dict = {}
    for ts, info in pose_dict.items():
        image_path = os.path.join(source_image_folder, info["image_name"])
        if not os.path.exists(image_path):
            continue
        filtered_pose_dict[ts] = info
    print(f"原姿态数量/过滤后姿态数量: {len(pose_dict)}/{len(filtered_pose_dict)}")

    # 转换位姿
    print("转换位姿数据...")
    for info in filtered_pose_dict.values():
        q_inv, t_inv = transform_pose(info["orientation"], info["position"])
        info["position"] = t_inv
        info["orientation"] = q_inv
    
    # 处理图像和生成数据
    print("处理图像...")
    cameras = process_images(config, source_image_folder, output_image_folder)
    
    print("处理点云和位姿...")
    points3D, images = process_pointcloud(config, pcmap_path, filtered_pose_dict)
    
    # 保存结果
    from colmap_utils import write_cameras_binary, write_images_binary, write_points3D_binary
    
    print("保存结果...")
    write_cameras_binary(cameras, os.path.join(sparse_dir, "cameras.bin"))
    write_images_binary(images, os.path.join(sparse_dir, "images.bin"))
    write_points3D_binary(points3D, os.path.join(sparse_dir, "points3D.bin"))
    
    print("处理完成!")

if __name__ == "__main__":
    main()
    
    





