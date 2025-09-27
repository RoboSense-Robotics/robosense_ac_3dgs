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
#pragma once
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <rosbag/bag.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <signal.h>
#include <condition_variable>

#include <thread>
#include <cstdint>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <filesystem>

#include "sensor_msgs/Imu.h"
#include "imu_module.hpp"

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                                   std::uint16_t, ring, ring)(double, timestamp, timestamp))
                                                   
struct PointXYZIRGBT
{
  PCL_ADD_POINT4D;
  float intensity;
  PCL_ADD_RGB;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRGBT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, rgb, rgb)
                                  (std::uint32_t, rgba, rgba)
                                  (double, timestamp, timestamp))

struct PointXYZIST
{
  PCL_ADD_POINT4D;
  float intensity;
  int segment;  // store segmentation result
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIST, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                                   int, segment, segment)(double, timestamp, timestamp))

struct PointXYZIRTRGB
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  double timestamp;
  uchar r;
  uchar g;
  uchar b;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRTRGB,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint16_t, ring, ring)(double, timestamp,
                                                                 timestamp)(uchar, r, r)(uchar, g, g)(uchar, b, b))

bool fileExistsInDirectory(const std::string& directory, const std::string& file_name) {
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.path().filename() == file_name) {
            return true;  // 找到文件，返回 true
        }
    }
    return false;  // 未找到文件，返回 false
}

                                                                 
namespace robosense
{
namespace lidar
{
template <typename T>
class SyncVector
{
public:
  SyncVector() = default;
  ~SyncVector() = default;

  void add(const T &element)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.push_back(element);
  }
  
  T front() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (data_.empty())
    {
      throw std::out_of_range("Vector is empty");
    }
    return data_.front();
  }

  T back() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (data_.empty())
    {
      throw std::out_of_range("Vector is empty");
    }
    return data_.back();
  }


  bool remove(const T &element)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::find(data_.begin(), data_.end(), element);
    if (it != data_.end())
    {
      data_.erase(it);
      return true;
    }
    return false;
  }

  T get(size_t index) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index < data_.size())
    {
      return data_[index];
    }
    throw std::out_of_range("Index out of range");
  }

  size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.size();
  }

private:
  std::vector<T> data_;
  mutable std::mutex mutex_;
};

template <typename T>
class SyncQueue
{
public:
  inline size_t push(const T& value)
  {
#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY
     bool empty = false;
#endif
     size_t size = 0;

    {
      std::lock_guard<std::mutex> lg(mtx_);
#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY
      empty = queue_.empty();
#endif
      queue_.push(value);
      size = queue_.size();
    }

#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY
    if (empty)
      cv_.notify_one();
#endif

    return size;
  }

  inline T pop()
  {
    T value;

    std::lock_guard<std::mutex> lg(mtx_);
    if (!queue_.empty())
    {
      value = queue_.front();
      queue_.pop();
    }

    return value;
  }

  inline T popWait(unsigned int usec = 1000000)
  {
    //
    // Low latency, or low CPU usage, that is the question. 
    //                                            - Hamlet

#ifdef ENABLE_WAIT_IF_QUEUE_EMPTY
    T value;

    {
      std::lock_guard<std::mutex> lg(mtx_);
      if (!queue_.empty())
      {
        value = queue_.front();
        queue_.pop();
        return value;
      }
    }

    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    return value;
#else

    T value;

    std::unique_lock<std::mutex> ul(mtx_);
    cv_.wait_for(ul, std::chrono::microseconds(usec), [this] { return (!queue_.empty()); });

    if (!queue_.empty())
    {
      value = queue_.front();
      queue_.pop();
    }

    return value;
#endif
  }

  inline void clear()
  {
    std::queue<T> empty;
    std::lock_guard<std::mutex> lg(mtx_);
    swap(empty, queue_);
  }

  inline bool empty() {
    std::lock_guard<std::mutex> lg(mtx_);
    return queue_.empty();
  }
  
  inline size_t size() {
    std::lock_guard<std::mutex> lg(mtx_);
    return queue_.size();
  }

private:
  std::queue<T> queue_;
  std::mutex mtx_;
#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY
  std::condition_variable cv_;
#endif
};

struct TransformXYZQuat
{
  TransformXYZQuat()=default;
  double qx;
  double qy;
  double qz;
  double qw;
  double x;
  double y;
  double z;
};

struct Intrinsics
{
  Intrinsics()=default;
  std::vector<double> distortion_coeffs;             // Distortion coefficients
  std::vector<double> camera_matrix;             // Intrinsic matrix values (fx, 0, cx; 0, fy, cy; 0, 0, 1)
  std::string camera_model;          // Type of camera model

  // Constructor to initialize intrinsics with values
  Intrinsics(std::vector<double> d_values, std::vector<double> k_values, std::string model)
    : distortion_coeffs(d_values), camera_matrix(k_values), camera_model(model) {}
};

struct MotionConfig
{
  MotionConfig()=default;
  bool motion_correct;                  // Whether to perform motion correction
  bool using_imu_linear_acceleration;   // Whether to use IMU linear acceleration data for motion compensation
  bool using_odom_linear_velocity;      // Whether to use ODOM data as linear velocity compensation, if true, it will override the displacement calculated from IMU linear acceleration
  bool frame_tail;                      // Whether to compensate points to the end time of the point cloud frame
  std::size_t num_drift;                       // The number of IMU data required to calculate drift
  std::string depth_root;              // Path to save depth images (commented out)
  std::string scene_root;              // Path of scene (commented out)
  std::string projection_root;         // Path to save data (commented out)
  std::string ori_points_topic;         // The topic name for obtaining RGB from original point cloud + motion-corrected point cloud
  std::string motion_points_topic;         // The topic name for obtaining RGB from original point cloud + motion-corrected point cloud
  std::string motion_rgb_points_topic;         // The topic name for obtaining RGB from original point cloud + motion-corrected point cloud
  std::string ori_rgb_points_topic;         // The topic name for obtaining RGB from original point cloud + motion-corrected point cloud
  std::string ori_img_topic;         // The topic name for obtaining RGB from original point cloud + motion-corrected point cloud
  std::string motion_proj_img_topic;         // The topic name for obtaining RGB from original point cloud + motion-corrected point cloud
};

struct CameraConfig
{
  CameraConfig()=default;
  std::string ros_topic;              // ROS topic for the camera
  Intrinsics  intrinsics;               // Intrinsic parameters for the camera
  // Constructor to initialize camera config with values
  CameraConfig(std::string topic, std::string root, Intrinsics intr)
    : ros_topic(topic), intrinsics(intr) {}
};

struct LidarConfig
{
  LidarConfig()=default;
  std::string ros_topic;              // ROS topic for the LIDAR
  std::string imu_ros_topic;           // ROS topic for the IMU
  TransformXYZQuat T_Lidar2Cam;    // Transformation from LIDAR to Camera
  TransformXYZQuat T_Lidar2IMU;    // Transformation from LIDAR to IMU
  // Constructor to initialize LIDAR config with values
  LidarConfig(std::string topic, std::string imu_topic)
      : ros_topic(topic), imu_ros_topic(imu_topic) {}
};

struct MotionDriverConfig
{
  MotionDriverConfig()=default;
  MotionConfig motion_config;
  CameraConfig camera_config;
  LidarConfig  ldiar_config;
};


void adjustBrightness(const cv::Mat& inputImage, cv::Mat& outputImage, int brightness)
{
  // 转换图像到CV_32F类型，以便于进行加法操作避免溢出
  inputImage.convertTo(outputImage, CV_32F);

  // 加上亮度值
  outputImage += brightness;

  // 转换回原来的类型，确保像素值在正确的范围内
  outputImage.convertTo(outputImage, inputImage.type());
}

void transformPointCloud(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud_in,
                        pcl::PointCloud<PointXYZIRT>::Ptr& cloud_out, const Eigen::Matrix4d& transform)
{
  cloud_out->points.resize(cloud_in->points.size());

  for (size_t i = 0; i < cloud_in->points.size(); ++i)
  {
    Eigen::Vector4d pt(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z, 1.0);
    Eigen::Vector4d pt_transformed = transform * pt;

    cloud_out->points[i].x = pt_transformed.x();
    cloud_out->points[i].y = pt_transformed.y();
    cloud_out->points[i].z = pt_transformed.z();
    cloud_out->points[i].intensity = cloud_in->points[i].intensity;
    cloud_out->points[i].ring = cloud_in->points[i].ring;
    cloud_out->points[i].timestamp = cloud_in->points[i].timestamp;
  }

  cloud_out->width = cloud_in->width;
  cloud_out->height = cloud_in->height;
  cloud_out->is_dense = cloud_in->is_dense;
}

IMUData fromRosMsg(const sensor_msgs::Imu& imu_msg)
{
  IMUData data;

  // 从 ROS 消息中提取时间戳
  data.timestamp = imu_msg.header.stamp.toSec();

  // 从 ROS 消息中提取角速度
  data.angular_velocity_x = imu_msg.angular_velocity.x;
  data.angular_velocity_y = imu_msg.angular_velocity.y;
  data.angular_velocity_z = imu_msg.angular_velocity.z;

  // 从 ROS 消息中提取线性加速度
  data.linear_acceleration_x = imu_msg.linear_acceleration.x;
  data.linear_acceleration_y = imu_msg.linear_acceleration.y;
  data.linear_acceleration_z = imu_msg.linear_acceleration.z;

  return data;
}

class SourceDriverRos
{
  public:
  // SourceDriverRos(const YAML::Node& _cfg);
  SourceDriverRos(const MotionDriverConfig& _cfg);
  ~SourceDriverRos();
  void subPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  // void subCameraCallback(const sensor_msgs::CompressedImageConstPtr& msg);
  void subCameraCallback(const sensor_msgs::Image::ConstPtr& msg);
  void subIMUCallback(const sensor_msgs::ImuConstPtr& msg);
  // void subOdomCallback(const ros_adapter::HunterStatus& msg);
  
  void processPointCloud();
  void processCompressedImage();
  int segmentPointCloud(const pcl::PointCloud<PointXYZIRT>::Ptr pc_origin, const Eigen::Matrix4f& _extrinsic);
  int segmentPts(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals,
                std::vector<pcl::PointIndices>& seg_indices);
  void start();
  void stop();

  private:
  std::shared_ptr<IMUModule> imu_module_;
  std::shared_ptr<IMUModule> imu_tmp_module_;
  std::shared_ptr<IMUModule> imu_image_module_;
  Eigen::Matrix4d T_imu_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_cam_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_cam_imu_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity(); // Lidar to base
  
  cv::Mat distortion_coeffs_;
  cv::Mat camera_intrisic_;
  std::string projection_root_;
  std::string depth_root_;
  std::string scene_root_;
  std::string fusion_pcd_root_;
  bool motion_correct_;

  SyncQueue<std::shared_ptr<sensor_msgs::PointCloud2>> point_cloud_queue_;
  // SyncQueue<ros_adapter::HunterStatus> hunter_status_queue_;
  
  SyncVector<pcl::PointCloud<PointXYZIRT>::Ptr> point_cloud_free_vec_;
  SyncVector<pcl::PointCloud<PointXYZIRT>::Ptr> mc_point_cloud_vec_;

  // SyncQueue<std::shared_ptr<sensor_msgs::CompressedImage>> cam_queue_;
  SyncQueue<std::shared_ptr<sensor_msgs::Image>> cam_queue_;
  SyncVector<std::shared_ptr<sensor_msgs::Image>> cam_vec_;
  std::thread point_cloud_process_thread_;
  std::thread cam_process_thread_;
  bool to_exit_process_;
  bool to_exit_cam_process_;
  bool using_imu_linear_acceleration_;
  bool using_odom_linear_velocity_;
  bool frame_tail_;
  std::size_t num_drift_;
  
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher pub_ori_points_;  // 原始点云
  ros::Publisher pub_motion_points_;  // 运动矫正后点云
  ros::Publisher pub_motion_rgb_points_; // 运动校正后点云 + 对应投影RGB
  ros::Publisher pub_ori_rgb_points_;  // 原始点云 + 运动矫正后点云对应投影RGB
  ros::Publisher pub_ori_img_;  // 原始图像投影
  ros::Publisher pub_motion_proj_img_; // 运动矫正后投影图像

  bool findNearestPoint(double _cam_stamp, std::size_t& _search_idx);
  void drawProjImage(cv::Mat& _img, const pcl::PointCloud<PointXYZIRT>::Ptr& _cm_pts, const Eigen::Matrix4d& _transform,
                    std::vector<cv::Point2f>& _image_points);
  void drawDepthImage(cv::Mat& _img, const pcl::PointCloud<PointXYZIRT>::Ptr& _cm_pts, const Eigen::Matrix4d& _transform,
                    std::vector<cv::Point2f>& _image_points);
  void drawDepthImageDebug(cv::Mat& _img, const pcl::PointCloud<PointXYZIRT>::Ptr& _cm_pts, const Eigen::Matrix4d& _transform,
                    std::vector<cv::Point2f>& _image_points);
};

SourceDriverRos::SourceDriverRos(const MotionDriverConfig& _cfg)
{  
  const LidarConfig& lidar_cfg   = _cfg.ldiar_config;
  const CameraConfig& cam_cfg    = _cfg.camera_config;
  const MotionConfig& motion_cfg = _cfg.motion_config;
  
  // T_lidar2imu
  {
    const TransformXYZQuat& lidar2imu_cfg = lidar_cfg.T_Lidar2IMU;
    Eigen::Quaterniond q_lidar2imu(lidar2imu_cfg.qw, lidar2imu_cfg.qx, lidar2imu_cfg.qy, lidar2imu_cfg.qz);
    std::cout << "lidar2imu: w " << q_lidar2imu.w() << " x "  << q_lidar2imu.x() << " y "  << q_lidar2imu.y() << " z "  << q_lidar2imu.z() << std::endl;
    T_imu_lidar_.block<3, 3>(0, 0) = q_lidar2imu.toRotationMatrix();
    T_imu_lidar_.block<3, 1>(0, 3) = Eigen::Vector3d(lidar2imu_cfg.x, lidar2imu_cfg.y, lidar2imu_cfg.z);
  }

  // T_lidar2cam
  {
    const TransformXYZQuat& lidar2cam_cfg = lidar_cfg.T_Lidar2Cam;
    Eigen::Quaterniond q_lidar2cam(lidar2cam_cfg.qw, lidar2cam_cfg.qx, lidar2cam_cfg.qy, lidar2cam_cfg.qz);
    std::cout << "lidar2cam: w " << q_lidar2cam.w() << " x "  << q_lidar2cam.x() << " y "  << q_lidar2cam.y() << " z "  << q_lidar2cam.z() << std::endl;
    T_cam_lidar_.block<3, 3>(0, 0) = q_lidar2cam.toRotationMatrix();
    T_cam_lidar_.block<3, 1>(0, 3) = Eigen::Vector3d(lidar2cam_cfg.x, lidar2cam_cfg.y, lidar2cam_cfg.z);
  }

  // 畸变系数
  {
    std::vector<double> distortion_coeffs = cam_cfg.intrinsics.distortion_coeffs;
    distortion_coeffs_ = cv::Mat(distortion_coeffs).clone();
  }

  // 内参
  {
    std::vector<double> intrinsic_matrix = cam_cfg.intrinsics.camera_matrix;
    camera_intrisic_ = cv::Mat(3, 3, CV_64F, intrinsic_matrix.data()).clone();
  }
  
  // 运动矫正参数
  {
    projection_root_               = motion_cfg.projection_root;
    depth_root_                    = motion_cfg.depth_root;
    scene_root_                    = motion_cfg.scene_root;
    motion_correct_                = motion_cfg.motion_correct;
    using_imu_linear_acceleration_ = motion_cfg.using_imu_linear_acceleration;
    using_odom_linear_velocity_    = motion_cfg.using_odom_linear_velocity;
    frame_tail_                    = motion_cfg.frame_tail;
    num_drift_                     = motion_cfg.num_drift;
  }
  
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  // 话题设置
  {
    pub_ori_points_ = nh_->advertise<sensor_msgs::PointCloud2>(motion_cfg.ori_points_topic, 10);  // 原始点云
    pub_motion_points_ = nh_->advertise<sensor_msgs::PointCloud2>(motion_cfg.motion_points_topic, 10); // 运动矫正后点云
    pub_motion_rgb_points_ = nh_->advertise<sensor_msgs::PointCloud2>(motion_cfg.motion_rgb_points_topic, 10);  // 运动校正后点云 + 对应投影RGB
    pub_ori_rgb_points_ = nh_->advertise<sensor_msgs::PointCloud2>(motion_cfg.ori_rgb_points_topic, 10); // 原始点云 + 运动矫正后点云对应投影RGB
    pub_ori_img_ = nh_->advertise<sensor_msgs::Image>(motion_cfg.ori_img_topic, 30); // 原始点云投影后图像
    pub_motion_proj_img_ = nh_->advertise<sensor_msgs::Image>(motion_cfg.motion_proj_img_topic, 30); // 运动矫正后投影图像
  }
  
  log_info("------------------------------------------------------");
  log_info("MOTION_CORRECT                 : " + std::to_string(motion_correct_));
  log_info("USING_IMU_LINEAR_ACCELERATION  : " + std::to_string(using_imu_linear_acceleration_));
  log_info("USING_ODOM_LINEAR_VELOCITY     : " + std::to_string(using_odom_linear_velocity_));
  log_info("FRAME_TAIL                     : " + std::to_string(frame_tail_));
  log_info("ORI_POINTS_TOPIC               : " + pub_ori_points_.getTopic());
  log_info("CM_POINTS_TOPIC                : " + pub_motion_points_.getTopic());
  log_info("ORI_RGB_POINTS_TOPIC           : " + pub_ori_rgb_points_.getTopic());
  log_info("PROJ_IMG_TOPIC                 : " + pub_motion_proj_img_.getTopic());
  log_info("------------------------------------------------------");
  
  T_cam_imu_ = T_cam_lidar_ * T_imu_lidar_.inverse();
  
  // TODO: set lidar frame
  imu_module_ = std::make_shared<IMUModule>(num_drift_, using_imu_linear_acceleration_, 9.81);
  imu_image_module_ = std::make_shared<IMUModule>(num_drift_, using_imu_linear_acceleration_, 9.81);
// #define TMP_IMU_MODULE
#ifdef TMP_IMU_MODULE
  imu_tmp_module_ = std::make_shared<IMUModule>(num_drift_, using_imu_linear_acceleration_, 9.81);
#endif

  start();
  point_cloud_process_thread_ = std::thread(std::bind(&SourceDriverRos::processPointCloud, this));
  cam_process_thread_ = std::thread(std::bind(&SourceDriverRos::processCompressedImage, this));
}

inline void SourceDriverRos::start()
{
  to_exit_cam_process_ = false;
  to_exit_process_ = false;
}

inline void SourceDriverRos::stop()
{
  to_exit_process_ = true;
  point_cloud_process_thread_.join();
  cam_process_thread_.join();
}

inline SourceDriverRos::~SourceDriverRos() { stop(); }

inline void SourceDriverRos::subPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  point_cloud_queue_.push(std::make_shared<sensor_msgs::PointCloud2>(*msg));
}

inline void SourceDriverRos::subCameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cam_queue_.push(std::make_shared<sensor_msgs::Image>(*msg));
}

inline void SourceDriverRos::subIMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  auto imu_data = fromRosMsg(*msg);
  imu_module_->addFrameData(imu_data);
#ifdef TMP_IMU_MODULE
  imu_tmp_module_->addFrameData(imu_data);
#endif
  imu_image_module_->addFrameData(imu_data);
}

void SourceDriverRos::processPointCloud()
{
  while (!to_exit_process_)
  {
    auto start = std::chrono::high_resolution_clock::now();
    std::shared_ptr<sensor_msgs::PointCloud2> msg = point_cloud_queue_.popWait(1000);
    if (msg.get() == NULL)
    {
      continue;
    }
    
    pcl::PointCloud<PointXYZIRT>::Ptr ori_cloud(new pcl::PointCloud<PointXYZIRT>());
    pcl::fromROSMsg(*msg, *ori_cloud);
    if (!motion_correct_)
    {
      sensor_msgs::PointCloud2 ori_cloud_ros;
      pcl::toROSMsg(*ori_cloud, ori_cloud_ros);
      ori_cloud_ros.header = msg->header;
      pub_ori_points_.publish(ori_cloud_ros);
      mc_point_cloud_vec_.add(ori_cloud);
      point_cloud_free_vec_.add(ori_cloud);
      continue;
    }
    
    pcl::PointCloud<PointXYZIRT>::Ptr cm_cloud(new pcl::PointCloud<PointXYZIRT>());
    // pcl::copyPointCloud(*ori_cloud, *cm_cloud);

    int point_index = 0;
    double head_stamp = ori_cloud->points.front().timestamp;
    double tail_stamp = ori_cloud->points.back().timestamp;
    
    double points_stamp = frame_tail_ ? tail_stamp : head_stamp;
    // if (tail_stamp - head_stamp > 0.12)
    // {
    //   continue;
    // }
    
    Eigen::Matrix4d T_lidar_imu = T_imu_lidar_.inverse();
    log_debug("Montion Correct Process, head stamp: " + std::to_string(head_stamp) + " tail stamp: " + std::to_string(tail_stamp) + " using stamp: " + std::to_string(points_stamp));

    MCStatus status = MCStatus::SKIPT;
    std::size_t count = 0, count_thres = 10;
    while (count++ < count_thres && !imu_module_->setHeadStamp(points_stamp, status, frame_tail_))
    {
      if (MCStatus::WAITING == status)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      break;
    }

#ifdef TMP_IMU_MODULE
    double points_stamp_inv = frame_tail_ ? head_stamp : tail_stamp;
    std::size_t sidx = 0;
    std::size_t search_idx_inv = 0;
    Eigen::Matrix4d g_trans;
    imu_module_->calIMURot(head_stamp, tail_stamp, g_trans, sidx);
    imu_tmp_module_->setHeadStamp(points_stamp_inv, status, !frame_tail_);
#endif

    if (MCStatus::SKIPT == status || MCStatus::WAITING == status)
    {
      log_warning("Montion Correct Warning, SetHeadStamp Failed find Frames, Skip Montion Correct!");
      continue;
    }
    
    log_info("SetHeadStamp success find Frames, pts head: " + std::to_string(head_stamp) 
         + " pts tail: " + std::to_string(tail_stamp) 
         + " window head: " + std::to_string(imu_module_->imu_window_.front().imu_data.timestamp) 
         + " window tail: " + std::to_string(imu_module_->imu_window_.back().imu_data.timestamp) 
         + " window size: " + std::to_string(imu_module_->imu_window_.size()));
    
    std::size_t search_idx = 0;
    Eigen::Matrix4d T_i1_i2 = Eigen::Matrix4d::Identity();
    for (std::size_t i = 0, num_pts = msg->width * msg->height; i < num_pts; ++i)
    {
      const auto& current_point = ori_cloud->points.at(point_index);

      double cur_stamp = current_point.timestamp;
      if (cur_stamp < head_stamp || cur_stamp > tail_stamp)
      {
        // std::ostringstream oss;
        // oss << std::fixed << std::setprecision(9)
        //     << "Montion Correct Warning, points stamp error than head_stamp or tail_stamp:  " << head_stamp << " "
        //     << cur_stamp << " " << tail_stamp;
        // log_warning(oss.str());
        // point_index++;
        // continue;
      }

      // auto start_trans = std::chrono::high_resolution_clock::now();
      // Eigen::Matrix4d T_i1_i2 = Eigen::Matrix4d::Identity();
      if (!imu_module_->calHeadIMURot(cur_stamp, T_i1_i2, search_idx, frame_tail_))
      {
        log_warning("Montion Correct Warning, cal calHeadIMURot failed " + std::to_string(head_stamp) + " " +
                    std::to_string(cur_stamp) + " " + std::to_string(tail_stamp));
        point_index++;
        continue;
      }

#ifdef TMP_IMU_MODULE
      Eigen::Matrix4d T_i1_i2_inv = Eigen::Matrix4d::Identity();
      imu_tmp_module_->calHeadIMURot(cur_stamp, T_i1_i2_inv, search_idx_inv, !frame_tail_);
      Eigen::Matrix4d g_trans_cal = frame_tail_ ? (T_i1_i2_inv * T_i1_i2) : (T_i1_i2 * T_i1_i2_inv);
      Eigen::Matrix4d delta_trans = g_trans_cal * g_trans.inverse();
      Eigen::AngleAxisd delta_vec(delta_trans.block<3, 3>(0, 0));
      Eigen::AngleAxisd g_trans_vec(g_trans.block<3, 3>(0, 0));
      Eigen::AngleAxisd g_trans_cal_vec(g_trans_cal.block<3, 3>(0, 0));
      // std::cout << "----------> delta angle: " << delta_vec.angle() * (180.0 / M_PI) 
      //           << " g_trans angle: " << g_trans_vec.angle() * (180.0 / M_PI) 
      //           << " g_trans_cal angle: " << g_trans_cal_vec.angle() * (180.0 / M_PI) 
      // << std::endl;
#endif
      if (frame_tail_)
      {
        Eigen::Matrix<double, 4, 4> _T_i1_i2_inv = T_i1_i2.eval();
        T_i1_i2 = _T_i1_i2_inv.inverse();
      }
      
      if (using_odom_linear_velocity_)
      {
        // TODO: 计算
      }

      // auto end_trans = std::chrono::high_resolution_clock::now();
      // std::chrono::duration<double, std::milli> duration = end_trans - start_trans;
      Eigen::Vector4d pt2(current_point.x, current_point.y, current_point.z, 1.0);

      Eigen::Matrix4d T_l1_l2 = T_lidar_imu * T_i1_i2 * T_imu_lidar_;
      Eigen::Vector4d trans_pt1 = T_l1_l2 * pt2;
      
      PointXYZIRT cm_pt;
      cm_pt.x = trans_pt1[0];
      cm_pt.y = trans_pt1[1];
      cm_pt.z = trans_pt1[2];
      cm_pt.intensity = current_point.intensity;
      cm_pt.ring = point_index;
      cm_pt.timestamp = points_stamp;
      cm_cloud->push_back(cm_pt);
      point_index++;
    }
    sensor_msgs::PointCloud2 cm_cloud_ros;
    pcl::toROSMsg(*cm_cloud, cm_cloud_ros);

    sensor_msgs::PointCloud2 ori_cloud_ros;
    pcl::toROSMsg(*ori_cloud, ori_cloud_ros);

    cm_cloud_ros.header = msg->header;
    cm_cloud_ros.header.stamp = ros::Time().fromSec(points_stamp);
    ori_cloud_ros.header = msg->header;
    
    pub_motion_points_.publish(cm_cloud_ros);
    pub_ori_points_.publish(ori_cloud_ros);
    mc_point_cloud_vec_.add(cm_cloud);
    point_cloud_free_vec_.add(ori_cloud);
    
// #define SAVE_PCD_MC
#ifdef SAVE_PCD_MC
    Eigen::AngleAxisd angle_axis(T_i1_i2.block<3, 3>(0, 0));
    double angle_velocity = (180.0 / M_PI) * angle_axis.angle() / (tail_stamp - head_stamp);
    if (mc_point_cloud_vec_.size() % 10 == 0 && !projection_root_.empty())
    {
      std::string ori_pcd_filename =
          projection_root_ + std::to_string(tail_stamp) + "_" + std::to_string(angle_velocity) + "_ori.pcd";
      std::string cm_pcd_filename =
          projection_root_ + std::to_string(tail_stamp) + "_" + std::to_string(angle_velocity) + "_cm.pcd";
      pcl::io::savePCDFile(cm_pcd_filename, *cm_cloud, true);
      pcl::io::savePCDFile(ori_pcd_filename, *ori_cloud, true);
    }
#endif
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    log_debug("MC time: " + std::to_string(duration.count()) + " ms");
  }
}

inline bool SourceDriverRos::findNearestPoint(double _cam_stamp,  std::size_t& _search_idx)
{
  std::size_t search_idx = _search_idx;
  if (mc_point_cloud_vec_.size() == 0)
  {
    std::cout << "mc_point_cloud_vec_ empty" << std::endl;
    return false;
  }

  double front_stamp = mc_point_cloud_vec_.front()->points.front().timestamp;
  double back_stamp = mc_point_cloud_vec_.back()->points.front().timestamp;

  // 如果时间戳差距过大就跳过
  if (_cam_stamp < front_stamp || _cam_stamp > back_stamp)
  {
    std::cout << std::fixed << std::setprecision(9) << "findNearestPoint"
              << " head_stamp: " << front_stamp << " cam_stamp: " << _cam_stamp << " back_stamp: " << back_stamp
              << std::endl;
    return false;
  }

  // 找到距离当前相机帧最近的Lidar帧
  while (search_idx > 1 && mc_point_cloud_vec_.get(search_idx - 1)->points.front().timestamp >= _cam_stamp)
  {
    search_idx--;
  }

  while (search_idx < mc_point_cloud_vec_.size() && _cam_stamp > mc_point_cloud_vec_.get(search_idx)->points.front().timestamp)
  {
    search_idx++;
  }

  if (search_idx >= 1 && _cam_stamp > mc_point_cloud_vec_.get(search_idx - 1)->points.front().timestamp &&
      search_idx < mc_point_cloud_vec_.size() &&
      _cam_stamp <= mc_point_cloud_vec_.get(search_idx)->points.front().timestamp)
  {
    auto lhs_pts = mc_point_cloud_vec_.get(search_idx - 1);
    auto rhs_pts = mc_point_cloud_vec_.get(search_idx);

    double lhs_diff = std::abs(_cam_stamp - lhs_pts->points.front().timestamp);
    double rhs_diff = std::abs(rhs_pts->points.front().timestamp - _cam_stamp);

    _search_idx = (lhs_diff < rhs_diff) ? search_idx - 1 : search_idx;
  }
  else
  {
    return false;
  }

  return true;
}

inline void labelImage(cv::Mat& img, const std::string& label)
{
  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 1.2;
  int thickness = 2;
  cv::Point textOrg(30, 120);
  cv::putText(img, label, textOrg, fontFace, fontScale, cv::Scalar(0, 0, 255), thickness, 8);
}

void SourceDriverRos::processCompressedImage()
{
  std::size_t search_idx = 0;
  std::size_t img_count = 0;
  std::size_t depth_count = 0;
  while (!to_exit_cam_process_)
  {
    // std::shared_ptr<sensor_msgs::CompressedImage> msg = cam_queue_.popWait(1000);
    auto start = std::chrono::high_resolution_clock::now();
    std::shared_ptr<sensor_msgs::Image> msg = cam_queue_.popWait(1000);
    if (msg.get() == NULL)
    {
      continue;
    }

    img_count++;
    double cam_stamp = msg->header.stamp.toSec();

    if (motion_correct_ &&
        (mc_point_cloud_vec_.size() == 0 || cam_stamp > mc_point_cloud_vec_.back()->points.front().timestamp))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(*msg, "bgr8");
    cv::Mat img = cv_image->image;

    pcl::PointCloud<PointXYZIRT>::Ptr ori_pts, cm_pts;
    if (motion_correct_)
    {
      while (!this->findNearestPoint(cam_stamp, search_idx))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      ori_pts = point_cloud_free_vec_.get(search_idx);
      cm_pts = mc_point_cloud_vec_.get(search_idx);
    }
    else
    {
      while (point_cloud_free_vec_.size() == 0 || mc_point_cloud_vec_.size() == 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      ori_pts = point_cloud_free_vec_.back();
      cm_pts = mc_point_cloud_vec_.back();
    }

    double pts_stamp = cm_pts->points.front().timestamp;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9) << "Find Nearest Point: " << std::to_string(pts_stamp)
        << " Camera: " << std::to_string(cam_stamp);
    log_info(oss.str());

    Eigen::Matrix4d T_ic_il = Eigen::Matrix4d::Identity(), T_il_ic = Eigen::Matrix4d::Identity();
    std::size_t imu_search_idx = 0;
    if (motion_correct_)
    {
      if (cam_stamp < pts_stamp)
      {
        if (!imu_image_module_->calIMURot(cam_stamp, pts_stamp, T_ic_il, imu_search_idx))
        {
          log_warning("Image proj CalIMURot Error, cam_stamp: " + std::to_string(cam_stamp) + " pts_stamp: " + std::to_string(pts_stamp));
        }
        T_il_ic = T_ic_il.inverse();
      }
      else if (cam_stamp > pts_stamp)
      {
        if (!imu_image_module_->calIMURot(pts_stamp, cam_stamp, T_il_ic, imu_search_idx))
        {
          log_warning("Image proj CalIMURot Error, cam_stamp: " + std::to_string(cam_stamp) + " pts_stamp: " + std::to_string(pts_stamp));
        }
        T_ic_il = T_il_ic.inverse();
      }

      if (imu_search_idx > 20)
      {
        imu_image_module_->clearData(imu_search_idx - 20);
        imu_search_idx = imu_search_idx - 20;
      }
    }

    Eigen::Matrix3d rotation_matrix = T_ic_il.block<3, 3>(0, 0);
    Eigen::AngleAxisd angle_axis(rotation_matrix);
    double angle = angle_axis.angle() * (180.0 / M_PI);
    double angle_velocity = angle / std::abs(pts_stamp - cam_stamp);
    Eigen::Vector3d translation = T_ic_il.block<3, 1>(0, 3);
    double linear_velocity = translation.norm() / std::abs(pts_stamp - cam_stamp);
    Eigen::Matrix4d T_cam_cl = T_cam_imu_ * T_ic_il * T_imu_lidar_;
    cv::Mat final_image;
// #define COMPLEX_PROJ
#ifdef COMPLEX_PROJ
    cv::Mat proj_img_mc_imu = img.clone(), proj_img_mc = img.clone(), proj_img_imu = img.clone(), proj_img = img.clone(); 
    std::vector<cv::Point2f> image_points, image_points_1, image_points_2, image_points_3;
    // 带动态补偿带IMU补偿
    drawProjImage(proj_img_mc_imu, cm_pts, T_cam_cl, image_points);
    // 带动态补偿不带IMU补偿
    drawProjImage(proj_img_mc, cm_pts, T_cam_lidar_, image_points_1);
    // 不带动态补偿带IMU补偿
    drawProjImage(proj_img_imu, ori_pts, T_cam_cl, image_points_2);
    // 不带动态补偿不带IMU补偿
    drawProjImage(proj_img, ori_pts, T_cam_lidar_, image_points_3);

// #define FLIP_IMG
#ifdef FLIP_IMG
    cv::Mat proj_img_mc_imu_dst, proj_img_mc_dst, proj_img_imu_dst, proj_img_dst;
    cv::flip(proj_img_mc_imu, proj_img_mc_imu_dst, -1);
    cv::flip(proj_img_mc, proj_img_mc_dst, -1);
    cv::flip(proj_img_imu, proj_img_imu_dst, -1);
    cv::flip(proj_img, proj_img_dst, -1);
    
    labelImage(proj_img_mc_imu_dst, "lt:" + std::to_string(pts_stamp) + " ct:" + std::to_string(cam_stamp)
        + " w: " + std::to_string(angle_velocity) + " x: " + std::to_string(linear_velocity));
    labelImage(proj_img_mc_dst, "wmc_lidar " + std::to_string(angle) + " " + std::to_string(translation.norm()));
    labelImage(proj_img_imu_dst, "womc_cam");
    labelImage(proj_img_dst, "womc_lidar");

    cv::Mat top_row, bottom_row, final_image;
    cv::hconcat(proj_img_mc_imu_dst, proj_img_mc_dst, top_row);
    cv::hconcat(proj_img_imu_dst, proj_img_dst, bottom_row);
    cv::vconcat(top_row, bottom_row, final_image);
#else
    labelImage(proj_img_mc_imu, "lt" + std::to_string(pts_stamp) + " ct:" + std::to_string(cam_stamp)
        + " w: " + std::to_string(angle_velocity) + " x: " + std::to_string(linear_velocity));
    labelImage(proj_img_mc, "wmc_lidar " + std::to_string(angle) + " " + std::to_string(translation.norm()));
    labelImage(proj_img_imu, "womc_cam");
    labelImage(proj_img, "womc_lidar");

    cv::Mat top_row, bottom_row;
    cv::hconcat(proj_img_mc_imu, proj_img_mc, top_row);
    cv::hconcat(proj_img_imu, proj_img, bottom_row);
    cv::vconcat(top_row, bottom_row, final_image);
#endif
#endif

// #define SIMPLE_PROJ
#ifdef SIMPLE_PROJ
    cv::Mat proj_img_mc_imu = img.clone();
    std::vector<cv::Point2f> image_points, image_points_1;
    // 带动态补偿带IMU补偿
    drawProjImage(proj_img_mc_imu, cm_pts, T_cam_cl, image_points);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pts(new pcl::PointCloud<pcl::PointXYZRGB>());
    Eigen::Matrix4d pub_trans = Eigen::Matrix4d::Identity();
    pub_trans << 1, 0, 0, 0,
                0, -1, 0, 0,
                0, 0, -1, 0,
                0, 0, 0, 1;

    for (size_t i = 0; i < image_points.size(); ++i)
    {
      pcl::PointXYZRGB pt;
      const auto& cm_pt = cm_pts->points[i];
      Eigen::Vector4d pt_eigen(cm_pt.x, cm_pt.y, cm_pt.z, 1.0);
      
      // const auto& ori_pt = ori_pts->points[i];
      // Eigen::Vector4d pt_eigen(ori_pt.x, ori_pt.y, ori_pt.z, 1.0);
      Eigen::Vector4d pt_trans = pub_trans * pt_eigen;

      pt.x = pt_trans[0];
      pt.y = pt_trans[1];
      pt.z = pt_trans[2];

      int u = image_points[i].x;
      int v = image_points[i].y;

      cv::Vec3b pixel_value(0.0, 0.0, 0.0);
      if (u >= 0 && u < img.cols && v >= 0 && v < img.rows)
      {
        pixel_value = img.at<cv::Vec3b>(v, u);
      }
      pt.r = pixel_value[2];
      pt.g = pixel_value[1];
      pt.b = pixel_value[0];
      rgb_pts->push_back(pt);
    }
    
    sensor_msgs::PointCloud2 rgb_pts_msg;
    pcl::toROSMsg(*rgb_pts, rgb_pts_msg);
    rgb_pts_msg.header.frame_id = "rslidar";
    rgb_pts_msg.header.stamp = msg->header.stamp;
    pub_motion_rgb_points_.publish(rgb_pts_msg);

    drawProjImage(img, ori_pts, T_cam_lidar_, image_points_1);
    cv::Mat proj_img_mc_imu_dst, img_dist;
    cv::flip(proj_img_mc_imu, proj_img_mc_imu_dst, -1);
    cv::flip(img, img_dist, -1);
    
    labelImage(proj_img_mc_imu_dst, "lidar_stamp" + std::to_string(pts_stamp) + " cam_stamp:" + std::to_string(cam_stamp) +
                                    " angle_velocity: " + std::to_string(angle_velocity));
    // labelImage(img, "womc_lidar");
    
    sensor_msgs::ImagePtr proj_mc_msg = cv_bridge::CvImage(msg->header, "bgr8", proj_img_mc_imu_dst).toImageMsg();
    sensor_msgs::ImagePtr proj_msg = cv_bridge::CvImage(msg->header, "bgr8", img_dist).toImageMsg();
    
    pub_proj_mc_.publish(proj_mc_msg);
    pub_ori_img_.publish(proj_msg);
    
    if (motion_correct_)
    {
      cv::hconcat(proj_img_mc_imu, img, final_image);
    }
    else
    {
      // cv::rotate(img, final_image, cv::ROTATE_180);
      final_image = img;
    }
#endif

#define ORI_IMG_RGB
#ifdef ORI_IMG_RGB
    std::vector<cv::Point2f> image_points;
    cv::Mat img_proj = img.clone();
    drawProjImage(img_proj, cm_pts, T_cam_cl, image_points);
    // cv::flip(img_proj, img_proj, -1);
    labelImage(img_proj, "lt:" + std::to_string(pts_stamp) + " ct:" + std::to_string(cam_stamp) + " w: " + std::to_string(angle_velocity) + " x: " + std::to_string(linear_velocity));
    
    cv::Mat img_depth = cv::Mat::zeros(img.size(), CV_8UC4);
    drawDepthImage(img_depth, cm_pts, T_cam_cl, image_points);
    
    // cv::Mat img_depth_debug = img.clone();
    // drawDepthImageDebug(img_depth_debug, cm_pts, T_cam_cl, image_points);

    // pcl::PointCloud<PointXYZIRTRGB>::Ptr rgb_pts(new pcl::PointCloud<PointXYZIRTRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pts(new pcl::PointCloud<pcl::PointXYZRGB>());
    Eigen::Matrix4d pub_trans = Eigen::Matrix4d::Identity();
    pub_trans << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
                
    for (size_t i = 0; i < image_points.size(); ++i)
    {
      // PointXYZIRTRGB pt;
      pcl::PointXYZRGB pt;
      const auto& cm_pt = ori_pts->points[i];
      Eigen::Vector4d pt_eigen(cm_pt.x, cm_pt.y, cm_pt.z, 1.0);
      Eigen::Vector4d pt_trans = pub_trans * pt_eigen;

      pt.x = pt_trans[0];
      pt.y = pt_trans[1];
      pt.z = pt_trans[2];
      // pt.intensity = cm_pt.intensity;
      // pt.ring = cm_pt.ring;
      // pt.timestamp = cm_pt.timestamp;
      
      int u = image_points[i].x;
      int v = image_points[i].y;

      cv::Vec3b pixel_value(0.0, 0.0, 0.0);
      if (u >= 0 && u < img.cols && v >= 0 && v < img.rows)
      {
        pixel_value = img.at<cv::Vec3b>(v, u);
      }
      pt.r = pixel_value[2];
      pt.g = pixel_value[1];
      pt.b = pixel_value[0];
      rgb_pts->push_back(pt);
    }
    
    sensor_msgs::PointCloud2 rgb_pts_msg;
    pcl::toROSMsg(*rgb_pts, rgb_pts_msg);
    rgb_pts_msg.header.frame_id = "rslidar";
    rgb_pts_msg.header.stamp = msg->header.stamp;
    pub_ori_rgb_points_.publish(rgb_pts_msg);
    
    sensor_msgs::ImagePtr proj_msg = cv_bridge::CvImage(msg->header, "bgr8", img_proj).toImageMsg();
    pub_motion_proj_img_.publish(proj_msg);
#endif
    std::cout << "project_root :"  << projection_root_<< std::endl;  
    if (!projection_root_.empty())
    {
      std::string file_name_pcd =
          projection_root_ + std::to_string(img_count) + "_" + std::to_string(angle_velocity) + ".pcd";
      // pcl::io::savePCDFile(file_name_pcd, *cm_pts, true);

      std::string file_name =
          projection_root_ + std::to_string(img_count) + "_" + std::to_string(angle_velocity) + "_" + std::to_string(linear_velocity) + ".png";
      cv::imwrite(file_name, final_image);
    }
    std::cout <<"depth_root :"  << depth_root_ << std::endl;
    if (!depth_root_.empty())
    {
      long long int_timestamp = static_cast<long long>(cam_stamp * 1e6);
      std::string file_name_depth = depth_root_  + "image_depths/" + std::to_string(int_timestamp) + "000.png";
      std::cout << "file_name_depth: " << file_name_depth << std::endl;
      if (!scene_root_.empty()){  
        // std::string target_directory = "/mnt/road/bev_mapping/data_3dgs/building_004/scene/images";
        std::string test_name_ply = depth_root_  +  "pc_depths/" + std::to_string(depth_count) + ".ply";
        std::string test_name = std::to_string(int_timestamp) + "000.jpg";
        std::cout << scene_root_ << '/'<< test_name << std::endl;
        if (fileExistsInDirectory(scene_root_, test_name)){
          std::cout << "file exists" << std::endl;
          cv::imwrite(file_name_depth, img_depth);
          pcl::io::savePLYFile(test_name_ply, *cm_pts);
          depth_count++;

        }
      }
      else{
        std::cout << "will saved" << std::endl;
        cv::imwrite(file_name_depth, img_depth);
      }
      
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << "Succ Process Camera, Cost time: " << duration.count() << " ms" << std::endl;
  }

  return;
}

void createColormapLUT(cv::Mat& colormap_lut, float min_z, float max_z)
{
  int lut_size = 256;  // You can adjust the LUT size for more granularity
  colormap_lut = cv::Mat(lut_size, 1, CV_8UC1);

  // Fill the LUT with values from 0 to 255
  for (int i = 0; i < lut_size; ++i)
  {
    colormap_lut.at<uchar>(i, 0) = static_cast<uchar>(i);
  }

  // Apply the colormap to the LUT
  cv::applyColorMap(colormap_lut, colormap_lut, cv::COLORMAP_JET);
}

int getColormapIndex(float depth, float min_z, float max_z, int lut_size)
{
  float normalized_depth = (depth - min_z) / (max_z - min_z);
  return static_cast<int>(normalized_depth * (lut_size - 1));
}

inline void SourceDriverRos::drawProjImage(cv::Mat& _img, const pcl::PointCloud<PointXYZIRT>::Ptr& _cm_pts,
                                          const Eigen::Matrix4d& _transform, std::vector<cv::Point2f>& _image_points)
{
  std::vector<cv::Point3f> transform_points;
  for (size_t i = 0; i < _cm_pts->points.size(); ++i)
  {
    Eigen::Vector4d pt(_cm_pts->points[i].x, _cm_pts->points[i].y, _cm_pts->points[i].z, 1.0);
    Eigen::Vector4d pt_cam = _transform * pt;

    // if (!std::isnan(pt_cam[0]) && !std::isnan(pt_cam[1]) && !std::isnan(pt_cam[2])) {
    //   transform_points.push_back(cv::Point3f(pt_cam[0], pt_cam[1], pt_cam[2]));
    // }
    transform_points.push_back(cv::Point3f(pt_cam[0], pt_cam[1], pt_cam[2]));
  }

  cv::projectPoints(transform_points, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), camera_intrisic_,
                    distortion_coeffs_, _image_points);
  
  float min_z = 0.0;
  float max_z = 15.0;
  int lut_size = 256;  // This should match the size used in createColormapLUT
  cv::Mat colormap_lut;
  createColormapLUT(colormap_lut, min_z, max_z);

  for (size_t i = 0; i < _image_points.size(); ++i)
  {
    int u = _image_points[i].x;
    int v = _image_points[i].y;
    if (u < 0 || u >= _img.cols || v < 0 || v >= _img.rows)
    {
      continue;
    }

    float depth = transform_points[i].z;
    depth = depth > max_z ? max_z : depth;
    depth = std::max(depth, min_z);
    float normalized_depth = (depth - min_z) / (max_z - min_z);
    int colormap_value = static_cast<int>(normalized_depth * 255);
    cv::Mat colormap_mat(1, 1, CV_8UC1, cv::Scalar(colormap_value));

    int colormap_index = getColormapIndex(depth, min_z, max_z, lut_size);
    cv::Vec3b color = colormap_lut.at<cv::Vec3b>(colormap_index, 0);
    cv::Scalar point_color(color[0], color[1], color[2]);  // BGR format

    cv::Vec3b pixel_value(0.0, 0.0, 0.0);
    cv::circle(_img, _image_points[i], 3, point_color, -1);
  }

  return;
}

inline void SourceDriverRos::drawDepthImageDebug(cv::Mat& _img, const pcl::PointCloud<PointXYZIRT>::Ptr& _cm_pts,
                                          const Eigen::Matrix4d& _transform, std::vector<cv::Point2f>& _image_points)
{
  std::vector<cv::Point3f> transform_points;
  for (size_t i = 0; i < _cm_pts->points.size(); ++i)
  {
    Eigen::Vector4d pt(_cm_pts->points[i].x, _cm_pts->points[i].y, _cm_pts->points[i].z, 1.0);
    Eigen::Vector4d pt_cam = _transform * pt;

    // if (!std::isnan(pt_cam[0]) && !std::isnan(pt_cam[1]) && !std::isnan(pt_cam[2])) {
    //   transform_points.push_back(cv::Point3f(pt_cam[0], pt_cam[1], pt_cam[2]));
    // }
    transform_points.push_back(cv::Point3f(pt_cam[0], pt_cam[1], pt_cam[2]));
  }

  cv::projectPoints(transform_points, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), camera_intrisic_,
                    distortion_coeffs_, _image_points);
  for (size_t i = 0; i < _image_points.size(); ++i)
  {
    int u = _image_points[i].x;
    int v = _image_points[i].y;
    if (u < 0 || u >= _img.cols || v < 0 || v >= _img.rows)
    {
      continue;
    }

    float depth = transform_points[i].z;
    if (depth < 10.0f) {
        depth = depth / 20.0f;
    } else {
        depth = (2.0f - 10.0f / depth) / 2.0f;
    }
    // _img.at<cv::Vec4b>(v, u) = cv::Vec4b(depth * 255, depth * 255, depth * 255, 255);
    cv::Scalar point_color(depth * 255, depth * 255, depth * 255);
    cv::circle(_img, _image_points[i], 3, point_color, -1);
  }

  return;
}

inline void SourceDriverRos::drawDepthImage(cv::Mat& _img, const pcl::PointCloud<PointXYZIRT>::Ptr& _cm_pts,
                                          const Eigen::Matrix4d& _transform, std::vector<cv::Point2f>& _image_points)
{
  std::vector<cv::Point3f> transform_points;
  for (size_t i = 0; i < _cm_pts->points.size(); ++i)
  {
    Eigen::Vector4d pt(_cm_pts->points[i].x, _cm_pts->points[i].y, _cm_pts->points[i].z, 1.0);
    Eigen::Vector4d pt_cam = _transform * pt;

    // if (!std::isnan(pt_cam[0]) && !std::isnan(pt_cam[1]) && !std::isnan(pt_cam[2])) {
    //   transform_points.push_back(cv::Point3f(pt_cam[0], pt_cam[1], pt_cam[2]));
    // }
    transform_points.push_back(cv::Point3f(pt_cam[0], pt_cam[1], pt_cam[2]));
  }
  
  cv::Mat zero_distortion = cv::Mat::zeros(1, 5, CV_64F);
  cv::projectPoints(transform_points, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), 
                   camera_intrisic_, zero_distortion, _image_points);
  for (size_t i = 0; i < _image_points.size(); ++i)
  {
    int u = _image_points[i].x;
    int v = _image_points[i].y;
    if (u < 0 || u >= _img.cols || v < 0 || v >= _img.rows)
    {
      continue;
    }

    float depth = transform_points[i].z;
    if (depth < 10.0f) {
        depth = depth / 20.0f;
    } else {
        depth = (2.0f - 10.0f / depth) / 2.0f;
    }
    _img.at<cv::Vec4b>(v, u) = cv::Vec4b(depth * 255, depth * 255, depth * 255, 255);
  }

  return;
}

};  // namespace robotic
};  // namespace robosense
