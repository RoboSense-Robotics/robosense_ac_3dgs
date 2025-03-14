import os
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from tqdm import tqdm

from colmap_utils import read_cameras_binary, read_images_binary, read_points3D_binary


def pick_select_image_points3D(cameras, images, points3D, idx, source_path):
    image = images[idx]
    
    pose_dict = {
        "id": image.id,
        "position": image.tvec,
        "orientation": image.qvec,
        "image_name": image.name
    }
    image_file = os.path.join(source_path, "../../images", image.name)
    print(image_file)
    img = cv2.imread(image_file)
    
    points, colors = [], []
    for id, point3D in points3D.items():
        points.append(point3D.xyz)
        colors.append(point3D.rgb)
    points = np.array(points)
    colors = np.array(colors)
    
    camera = cameras[image.camera_id]
    fc_param = camera.params
    camera_matrix = np.array([[fc_param[0], 0, fc_param[2]], [0, fc_param[1], fc_param[3]], [0, 0, 1]])
    params = {
        'camera_matrix': camera_matrix,
        'dist_coeffs': np.zeros(5),
        'w': camera.width,
        'h': camera.height
    }
    print(camera_matrix)
    
    return img, pose_dict, points, colors, params

def paint_points_on_image(img, points, colors, camera_params, camera_pose, zero=False, inverse=False):
    """
    在图像上绘制投影的点云。

    参数:
        img (numpy.ndarray): 要绘制的图像。
        points (numpy.ndarray): Nx3的3D点坐标数组。
        colors (numpy.ndarray): Nx3的颜色数组。
        camera_params (dict): 相机参数。
        camera_pose (dict): 相机位姿。

    返回:
        numpy.ndarray: 绘制了点云的图像。
    """
    # projected_points = project_points_to_image(points, camera_params, camera_pose, inverse=inverse)
    projected_points, colors, depths = project_points_to_image_with_depth(points, colors, camera_params, camera_pose)
    
    # 过滤掉投影到图像外部的点
    h, w = camera_params['h'], camera_params['w']
    valid_indices = (
        (projected_points[:, 0] >= 0) & (projected_points[:, 0] < w) &
        (projected_points[:, 1] >= 0) & (projected_points[:, 1] < h)
    )
    projected_points = projected_points[valid_indices]
    colors = colors[valid_indices]

    if zero:
        img = np.zeros_like(img)
    for i in range(len(projected_points)):
        x, y = projected_points[i].ravel()
        color = colors[i]
        # OpenCV uses BGR, so we need to reverse the RGB order
        color_bgr = (int(color[2] * 255), int(color[1] * 255), int(color[0] * 255))
        cv2.circle(img, (int(x), int(y)), 2, color_bgr, -1)
    
    return img

def paint_points_on_image2(img, points, colors, camera_params, camera_pose, zero=False):
    """
    在图像上根据深度渲染投影的点云
    深度范围固定为0到50。
    
    参数:
        img (numpy.ndarray): 要绘制的图像。
        points (numpy.ndarray): Nx3的3D点坐标数组。
        colors (numpy.ndarray): Nx3的颜色数组。（此参数将被忽略，因为我们使用深度渲染）
        camera_params (dict): 相机参数。
        camera_pose (dict): 相机位姿。
        zero (bool): 是否将图像清零后再绘制。
    
    返回:
        numpy.ndarray: 绘制了深度点云的图像。
    """
    # 投影3D点到图像平面，并获取深度值
    projected_points, colors, depths = project_points_to_image_with_depth(points, colors, camera_params, camera_pose)
    h, w = camera_params['h'], camera_params['w']
    
    # 自适应计算深度范围
    min_depth = np.percentile(depths, 2)  # 去除异常值
    max_depth = np.percentile(depths, 75)
    normalized_depth = (depths - min_depth) / (max_depth - min_depth)
    normalized_depth = np.clip(normalized_depth, 0, 1)
    
    # 使用VIRIDIS色彩映射以获得更好的视觉效果和专业性
    # VIRIDIS提供了更好的感知线性度和色觉缺陷友好性
    colormap = cv2.COLORMAP_JET
    
    # 增强对比度
    normalized_depth = cv2.equalizeHist((normalized_depth * 255).astype(np.uint8)) / 255.0
    depth_colored = cv2.applyColorMap((normalized_depth * 255).astype(np.uint8), colormap)
    
    # 创建高质量渲染
    zero_img = np.zeros((h, w, 3), dtype=np.uint8)
    for i in range(len(projected_points)):
        x, y = projected_points[i].astype(int)
        color = depth_colored[i].reshape(-1)
        
        # 使用抗锯齿圆形和渐变效果
        cv2.circle(zero_img, (x, y), 3, color.tolist(), -1, cv2.LINE_AA)
        cv2.circle(zero_img, (x, y), 2, (color * 1.2).clip(0, 255).astype(np.uint8).tolist(), -1, cv2.LINE_AA)
    
    if zero:
        img = zero_img
    else:
        img = cv2.addWeighted(img, 0.5, zero_img, 0.5, 0)

    return img

def project_points_to_image_with_depth(points, colors, camera_params, camera_pose):
    """
    投影3D点到图像平面，并返回对应的深度值，同时剔除投影到图像外部或具有负深度的点。
    
    参数:
        points (numpy.ndarray): Nx3的3D点坐标数组。
        camera_params (dict): 相机参数，包括内参矩阵、图像高度h和宽度w。
        camera_pose (dict): 相机位姿，包括旋转四元数和位置。
    
    返回:
        tuple:
            projected_points (numpy.ndarray): Mx2的有效2D投影点坐标（M ≤ N）。
            depths (numpy.ndarray): M的有效深度值数组。
    """
    camera_matrix = camera_params['camera_matrix']
    h, w = camera_params['h'], camera_params['w']

    # 提取相机位姿
    position = camera_pose['position']
    orientation = camera_pose['orientation']  # [qw, qx, qy, qz]

    # 转换四元数为旋转矩阵
    rotation = R.from_quat([orientation[1], orientation[2], orientation[3], orientation[0]])  # [x, y, z, w]
    R_mat = rotation.as_matrix()
    tvec = np.array(position).reshape((3, 1))
    
    # 将点从世界坐标系转换到相机坐标系
    cam_points = (R_mat @ points.T) + tvec  # 3xN
    cam_points = cam_points.T  # Nx3
    
    # 透视投影
    projected = (camera_matrix @ cam_points.T).T  # Nx3
    
    # 归一化
    projected_points = projected[:, :2] / projected[:, 2].reshape(-1, 1)  # Nx2
    
    depths = cam_points[:, 2]  # Nx1
    
    # 筛选有效点
    # 条件1: 深度值大于0
    valid_depth_mask = depths > 0
    
    # 条件2: 投影点在图像内部
    x = projected_points[:, 0]
    y = projected_points[:, 1]
    valid_x_mask = (x >= 0) & (x < w)
    valid_y_mask = (y >= 0) & (y < h)
    
    # 综合所有有效条件
    valid_mask = valid_depth_mask & valid_x_mask & valid_y_mask
    
    # 应用掩码
    projected_points_filtered = projected_points[valid_mask]
    depths_filtered = depths[valid_mask]
    colors_filtered = colors[valid_mask]
    
    return projected_points_filtered, colors_filtered, depths_filtered

def main():
    data_dir = "data/new_building_001_v2/scene"
    source_path = os.path.join(data_dir, "sparse/0")
    
    cameras_bin_file = os.path.join(source_path, "cameras.bin")
    images_bin_file = os.path.join(source_path, "images.bin")
    points3D_bin_file = os.path.join(source_path, "points3D.bin")
    
    cameras = read_cameras_binary(cameras_bin_file)
    images = read_images_binary(images_bin_file)
    points3D = read_points3D_binary(points3D_bin_file)

    print("num_cameras:", len(cameras))
    print("num_images:", len(images))
    print("num_points3D:", len(points3D))
    
    output_folder = os.path.join(data_dir, "test_waican/0")
    os.makedirs(output_folder, exist_ok=True)
    id_list = list(images.keys())
    for i, id in tqdm(enumerate(id_list), desc="Processing images", total=len(id_list)):
        if not i%5==0:
            continue
  
        img, test_camera_pose, points, colors, camera_params = pick_select_image_points3D(cameras, images, points3D, id, source_path)

        # 使用深度渲染在图像上绘制点云
        img = paint_points_on_image2(img, points, colors/255.0, camera_params, test_camera_pose, zero=False)
        output_image_file = os.path.join(output_folder, f"test_pcd_on_image_with_depth_{id}_2.jpg")
        cv2.imwrite(output_image_file, img)

        # 渲染点云图像
        # img = paint_points_on_image(img, points, colors/255.0, camera_params, test_camera_pose, zero=True, inverse=False)
        # output_image_file = os.path.join(output_folder, f"test_pcd_on_image_with_img_{id}.jpg")
        # cv2.imwrite(output_image_file, img)


if __name__ == "__main__":
    main()