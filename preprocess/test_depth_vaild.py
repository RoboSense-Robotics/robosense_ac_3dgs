import os
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from PIL import Image
import argparse
from tqdm import tqdm

from colmap_utils import read_cameras_binary, read_images_binary, read_points3D_binary

def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='SLAM数据转3DGS格式工具')
    parser.add_argument('--data_dir', type=str, required=True,
                      help='数据目录，包含filter_images、pose文件和点云文件')
    return parser.parse_args()

def unwarpped_depth(depth):
    """将归一化的深度值还原为真实深度值
    
    Args:
        depth (ndarray): 归一化的深度图，取值范围[0,1]
        
    Returns:
        ndarray: 还原后的真实深度值，单位为米
        
    Notes:
        使用分段函数进行深度还原:
        - 当depth < 0.5时，使用线性映射: depth * 20.0 
        - 当depth >= 0.5时，使用非线性映射: 10/(1-depth)
        这样可以在近距离保持线性精度，远距离采用非线性映射以获得更大的深度范围
    """
    return np.where(depth < 0.5, 2 * depth * 10.0, 10 / (1.0 - depth) / 2)


def main():
    # data_dir = "data/cartoon_001_v2"

    args = parse_args()
    data_dir = args.data_dir
    
    output_folder = os.path.join(data_dir, "scene/test_depth/0")
    os.makedirs(output_folder, exist_ok=True)

    scene_dir = os.path.join(data_dir, "scene")
    images_bin_file = os.path.join(scene_dir, "sparse/0/images.bin")
    images = read_images_binary(images_bin_file)
    for id, image in tqdm(images.items(), total=len(images), desc="Checking image files"):
        if not id%5==0:
            continue
        image_path = os.path.join(scene_dir, "images", image.name)
        # 读取图片
        image_map = np.array(Image.open(image_path))[:, :, ::-1]
        # 读取深度图
        depth_name = image.name.split(".")[0] + ".png"
        depth_path = os.path.join(scene_dir, "depths", depth_name)

        depth_map = np.array(Image.open(depth_path).convert("LA"))
        depth = depth_map[:, :, 0] / 255.0
        depth = unwarpped_depth(depth)  # 1080 * 1920
        depth_mask = (depth_map[:, :, 1] / 255.0).astype(np.uint8)

        depths = depth[depth_mask==1]
        print("min depth:", np.min(depths))
        print("max depth:", np.max(depths))
        min_depth = np.percentile(depths, 2)  # 去除异常值
        max_depth = np.percentile(depths, 75)
        print("pick min_depth:", min_depth)
        print("pick max_depth:", max_depth)
        normalized_depth = (depths - min_depth) / (max_depth - min_depth)
        normalized_depth = np.clip(normalized_depth, 0, 1)
        
        # 增强对比度
        colormap = cv2.COLORMAP_JET
        normalized_depth = cv2.equalizeHist((normalized_depth * 255).astype(np.uint8)) / 255.0
        depth_colored = cv2.applyColorMap((normalized_depth * 255).astype(np.uint8), colormap)
        depth_colored = depth_colored.reshape(-1, 3)
        depth_viz = image_map.copy()
        depth_xy_coord = np.where(depth_mask==1)
        for i in range(len(depth_xy_coord[0])):
            y, x = depth_xy_coord[0][i], depth_xy_coord[1][i]
            color = depth_colored[i].tolist()
            cv2.circle(depth_viz, (x, y), radius=2, color=color, thickness=-1)

        output_image_file = os.path.join(output_folder, f"test_depth_{id}.jpg")
        cv2.imwrite(output_image_file, depth_viz)





if __name__ == "__main__":
    main()