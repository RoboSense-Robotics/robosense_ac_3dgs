import os
import numpy as np
import cv2
from PIL import Image
import yaml
import argparse

from tqdm import tqdm

from colmap_utils import read_cameras_binary, read_images_binary, read_points3D_binary

from concurrent.futures import ProcessPoolExecutor, as_completed

# # depth_path = "/mnt/road/bev_mapping/data_3dgs/garage/1702454446.650667_L.png"
# depth_path = "/media/sti/beac7682-9b6e-492f-8983-5a9832652bca/1211/data/test/1/1733838162529865000.png"
# depth = Image.open(depth_path)  # LA

# depth_np = np.array(depth)
# depth_la_np = np.array(depth.convert("LA"))
# print(depth_np.shape)
# aaa= 0

def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='SLAM数据转3DGS格式工具')
    parser.add_argument('--data_dir', type=str, required=True,
                      help='数据目录，包含filter_images、pose文件和点云文件')
    return parser.parse_args()

def get_camera_params(config):
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

    return mapx, mapy, None

def process_image(args):
    """处理单张图像的去畸变"""
    input_path, params, output_path = args
    mapx, mapy, _ = params  # ROI不再使用
    
    try:
        img = cv2.imread(input_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            print(f"无法读取图像: {input_path}")
            return
            
        # 使用remap进行去畸变
        undistorted_img = cv2.remap(img, mapx, mapy, cv2.INTER_NEAREST)
        cv2.imwrite(output_path, undistorted_img)
    except Exception as e:
        print(f"处理图像 {input_path} 时出错: {e}")

def main():
    args = parse_args()

    # 检查图像和深度文件是否对齐
    scene_dir = os.path.join(args.data_dir, "scene")
    images_bin_file = os.path.join(scene_dir, "sparse/0/images.bin")
    images = read_images_binary(images_bin_file)
    for id, image in tqdm(images.items(), total=len(images), desc="Checking image files"):
        image_path = os.path.join(scene_dir, "images", image.name)
        depth_name = image.name.split(".")[0] + ".png"
        depth_path = os.path.join(args.data_dir, "filter_depths", depth_name)
        if not os.path.exists(image_path):
            print(f"Image file {image_path} not found")
            raise FileNotFoundError
        if not os.path.exists(depth_path):
            print(f"Depth file {depth_path} not found")
            raise FileNotFoundError

    # 将深度图加入3dgs数据
    config_path = os.path.join(args.data_dir, "calibration.yaml")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    config = config["ADDTION_INFO"]
    params = get_camera_params(config)

    output_depth_folder = os.path.join(scene_dir, "depths")
    os.makedirs(output_depth_folder, exist_ok=True)
    
    from concurrent.futures import ProcessPoolExecutor, as_completed
    with ProcessPoolExecutor(max_workers=32) as executor:
        # 提交所有任务
        futures = {}
        for id, image in tqdm(images.items(), total=len(images), desc="Checking image files"):
            depth_name = image.name.split(".")[0] + ".png"
            depth_path = os.path.join(args.data_dir, "filter_depths", depth_name)
            output_depth_path = os.path.join(output_depth_folder, depth_name)
            futures[executor.submit(process_image, (depth_path, params, output_depth_path))] = depth_name
        
        # 使用 tqdm 显示进度条
        for future in tqdm(as_completed(futures), total=len(images), desc="Undistorting images"):
            image_name = futures[future]
            try:
                future.result()
            except Exception as e:
                print(f"图像 {image_name} 处理失败: {e}")





        
if __name__ == "__main__":
    main()
