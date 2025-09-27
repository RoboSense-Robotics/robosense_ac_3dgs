/******************************************************************************
 * Copyright 2024 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include <functional>
#include <yaml-cpp/yaml.h>
#include "source_driver_ros.hpp"
#include "yaml_reader.hpp"
using namespace robosense::lidar;

static SourceDriverRos* robo_ptr = nullptr;
static void sigHandler(int sig)
{
  if (robo_ptr)
  {
    delete robo_ptr;
    robo_ptr = nullptr;
  }
  ros::shutdown();
}

MotionDriverConfig parse_config(const YAML::Node& cfg)
{
  MotionDriverConfig res_cfg;
  const YAML::Node& lidar_cfg  = cfg["LIDAR"];
  const YAML::Node& cam_cfg    = cfg["CAMERA"];
  const YAML::Node& motion_cfg = cfg["MOTION_CORRECT"];
  
  std::string calibration_path = cfg["CALIBRATION_PATH"].as<std::string>();
  YAML::Node calibration_cfg = YAML::LoadFile(calibration_path);
  TransformXYZQuat T_Lidar2IMU;
  TransformXYZQuat T_Cam2IMU;
  {
    const YAML::Node& lidar2imu_cfg = calibration_cfg["Sensor"]["Lidar"]["extrinsic"];
    std::cout << "lidar2imu_cfg: " << lidar2imu_cfg << std::endl;
    yamlRead<double>(lidar2imu_cfg["quaternion"], "x", T_Lidar2IMU.qx, 0.0);
    yamlRead<double>(lidar2imu_cfg["quaternion"], "y", T_Lidar2IMU.qy, 0.0);
    yamlRead<double>(lidar2imu_cfg["quaternion"], "z", T_Lidar2IMU.qz, 0.0);
    yamlRead<double>(lidar2imu_cfg["quaternion"], "w", T_Lidar2IMU.qw, 1.0);
    yamlRead<double>(lidar2imu_cfg["translation"], "x", T_Lidar2IMU.x, 0.0);
    yamlRead<double>(lidar2imu_cfg["translation"], "y", T_Lidar2IMU.y, 0.0);
    yamlRead<double>(lidar2imu_cfg["translation"], "z", T_Lidar2IMU.z, 0.0);
    
  }
  {
    const YAML::Node& cam2imu_cfg = calibration_cfg["Sensor"]["Camera"]["extrinsic"];
    std::cout << "cam2imu_cfg: " << cam2imu_cfg << std::endl;
    yamlRead<double>(cam2imu_cfg["quaternion"], "x", T_Cam2IMU.qx, 0.0);
    yamlRead<double>(cam2imu_cfg["quaternion"], "y", T_Cam2IMU.qy, 0.0);
    yamlRead<double>(cam2imu_cfg["quaternion"], "z", T_Cam2IMU.qz, 0.0);
    yamlRead<double>(cam2imu_cfg["quaternion"], "w", T_Cam2IMU.qw, 1.0);
    yamlRead<double>(cam2imu_cfg["translation"], "x", T_Cam2IMU.x, 0.0);
    yamlRead<double>(cam2imu_cfg["translation"], "y", T_Cam2IMU.y, 0.0);
    yamlRead<double>(cam2imu_cfg["translation"], "z", T_Cam2IMU.z, 0.0);
  }

  {
    // Use the previously loaded T_Lidar2IMU and T_Cam2IMU to calculate T_Lidar2Cam
    // T_Lidar2Cam = T_Cam2IMU.inverse() * T_Lidar2IMU
    
    // First, assign T_Lidar2IMU from the calibration file
    res_cfg.ldiar_config.T_Lidar2IMU = T_Lidar2IMU;
    
    // Calculate T_Lidar2Cam using the transformation relationship
    // Convert quaternions and translations to Eigen matrices for easier manipulation
    Eigen::Quaterniond q_lidar2imu(T_Lidar2IMU.qw, T_Lidar2IMU.qx, T_Lidar2IMU.qy, T_Lidar2IMU.qz);
    Eigen::Vector3d t_lidar2imu(T_Lidar2IMU.x, T_Lidar2IMU.y, T_Lidar2IMU.z);
    
    Eigen::Quaterniond q_cam2imu(T_Cam2IMU.qw, T_Cam2IMU.qx, T_Cam2IMU.qy, T_Cam2IMU.qz);
    Eigen::Vector3d t_cam2imu(T_Cam2IMU.x, T_Cam2IMU.y, T_Cam2IMU.z);
    
    // Calculate inverse of Cam2IMU transformation
    Eigen::Quaterniond q_imu2cam = q_cam2imu.inverse();
    Eigen::Vector3d t_imu2cam = -(q_imu2cam * t_cam2imu);
    
    // Calculate Lidar2Cam transformation
    Eigen::Quaterniond q_lidar2cam = q_imu2cam * q_lidar2imu;
    Eigen::Vector3d t_lidar2cam = q_imu2cam * t_lidar2imu + t_imu2cam;
    
    // Assign the calculated transformation to T_Lidar2Cam
    res_cfg.ldiar_config.T_Lidar2Cam.qx = q_lidar2cam.x();
    res_cfg.ldiar_config.T_Lidar2Cam.qy = q_lidar2cam.y();
    res_cfg.ldiar_config.T_Lidar2Cam.qz = q_lidar2cam.z();
    res_cfg.ldiar_config.T_Lidar2Cam.qw = q_lidar2cam.w();
    res_cfg.ldiar_config.T_Lidar2Cam.x = t_lidar2cam.x();
    res_cfg.ldiar_config.T_Lidar2Cam.y = t_lidar2cam.y();
    res_cfg.ldiar_config.T_Lidar2Cam.z = t_lidar2cam.z();
    
    yamlRead<std::string>(lidar_cfg, "ROS_TOPIC", res_cfg.ldiar_config.ros_topic, "/rslidar_points_meta");
    yamlRead<std::string>(lidar_cfg, "IMU_ROS_TOPIC", res_cfg.ldiar_config.imu_ros_topic, "/rslidar_imuData_meta");
  }
  
  {
    const YAML::Node& cam_intrinsic_cfg = calibration_cfg["Sensor"]["Camera"];
    std::vector<double> distortion_coeffs;
    for (size_t i = 0; i < cam_intrinsic_cfg["intrinsic"]["dist_coeff"].size(); ++i)
    {
      distortion_coeffs.push_back(cam_intrinsic_cfg["intrinsic"]["dist_coeff"][i].as<double>());
    }
    res_cfg.camera_config.intrinsics.distortion_coeffs = distortion_coeffs;
    
    std::vector<double> intrinsic_matrix;
    if (cam_intrinsic_cfg["intrinsic"] && cam_intrinsic_cfg["intrinsic"]["int_matrix"])
    {
      for (size_t i = 0; i < cam_intrinsic_cfg["intrinsic"]["int_matrix"].size(); ++i)
      {
        intrinsic_matrix.push_back(cam_intrinsic_cfg["intrinsic"]["int_matrix"][i].as<double>());
      }
    }
    res_cfg.camera_config.intrinsics.camera_matrix = intrinsic_matrix;
    yamlRead<std::string>(cam_cfg, "ROS_TOPIC", res_cfg.camera_config.ros_topic, "/image_rgb");
  }
  
  {
    yamlRead<bool>(motion_cfg, "MOTION_CORRECT",                res_cfg.motion_config.motion_correct,                true);
    yamlRead<bool>(motion_cfg, "USING_IMU_LINEAR_ACCELERATION", res_cfg.motion_config.using_imu_linear_acceleration, false);
    yamlRead<bool>(motion_cfg, "USING_ODOM_LINEAR_VELOCITY",    res_cfg.motion_config.using_odom_linear_velocity,    false);
    yamlRead<bool>(motion_cfg, "FRAME_TAIL",                    res_cfg.motion_config.frame_tail,                    false);
    yamlRead<std::size_t>(motion_cfg, "NUM_DRIFT",              res_cfg.motion_config.num_drift,                     50);
    yamlRead<std::string>(motion_cfg, "PROJECTION_ROOT",        res_cfg.motion_config.projection_root,               "");
    yamlRead<std::string>(motion_cfg, "DEPTH_ROOT",             res_cfg.motion_config.depth_root,                    "");
    yamlRead<std::string>(motion_cfg, "SCENE_ROOT",             res_cfg.motion_config.scene_root,                    "");
    yamlRead<std::string>(motion_cfg, "ORI_POINTS_TOPIC",       res_cfg.motion_config.ori_points_topic,              "/rslidar_points_origin");
    yamlRead<std::string>(motion_cfg, "CM_POINTS_TOPIC",        res_cfg.motion_config.motion_points_topic,           "/rslidar_points_motion");
    yamlRead<std::string>(motion_cfg, "CM_RGB_POINTS_TOPIC",    res_cfg.motion_config.motion_rgb_points_topic,       "/rslidar_points_motion_rgb");
    yamlRead<std::string>(motion_cfg, "ORI_RGB_POINTS_TOPIC",   res_cfg.motion_config.ori_rgb_points_topic,          "/rslidar_points_origin_rgb");
    yamlRead<std::string>(motion_cfg, "ORI_IMG_TOPIC",          res_cfg.motion_config.ori_img_topic,                 "/camera/image_raw/proj_ori");
    yamlRead<std::string>(motion_cfg, "PROJ_IMG_TOPIC",         res_cfg.motion_config.motion_proj_img_topic,         "/camera/image_raw/proj_motion");
  }
  return res_cfg;
}

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "use as: ./demo_lidar2cam /path/to/config.yaml" << std::endl;
  }
  // std::string robot_cfg_path = "/home/sti/sdb/robot-worksapce/catkin-robot/config/meta04_config.yaml";
  const std::string robot_cfg_path = argv[1];
  
  YAML::Node cfg = YAML::LoadFile(robot_cfg_path);
  std::cout << cfg << std::endl;

  ros::init(argc, argv, "rs_motion_correct", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  signal(SIGINT, sigHandler);
  
  MotionDriverConfig motion_cfg = parse_config(cfg);
  // std::cout << "projection_root: " << motion_cfg.projection_root <<std::endl;
  SourceDriverRos robo(motion_cfg);
  robo_ptr = &robo;

  const std::string pts_topic = motion_cfg.ldiar_config.ros_topic;
  auto pts_callback = std::bind(&SourceDriverRos::subPointCloudCallback, &robo, std::placeholders::_1);
  ros::Subscriber pts_sub = nh.subscribe<sensor_msgs::PointCloud2>(pts_topic, 100, pts_callback);

  const std::string imu_topic = motion_cfg.ldiar_config.imu_ros_topic;
  auto imu_callback = std::bind(&SourceDriverRos::subIMUCallback, &robo, std::placeholders::_1);
  ros::Subscriber pts_mc_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 100, imu_callback);

  const std::string cam_topic = motion_cfg.camera_config.ros_topic;
  auto cam_callback = std::bind(&SourceDriverRos::subCameraCallback, &robo, std::placeholders::_1);
  ros::Subscriber cam_sub = nh.subscribe<sensor_msgs::Image>(cam_topic, 300, cam_callback);

  ros::spin();
}
