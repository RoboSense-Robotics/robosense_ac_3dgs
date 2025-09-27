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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>
#include <sstream>
// #include "yaml_reader.hpp"
#ifdef RS_DRIVER
// #include "rs_driver/msg/imu_data_msg.hpp"
// #include "rs_driver/api/lidar_driver.hpp"
#include "rs_driver/common/rs_log.hpp"
#else

#endif

namespace robosense
{
namespace lidar
{

enum class MCStatus
{
  SKIPT = 0,
  WAITING = 1,
  SUCCESS = 2,
};

inline void log_error(const std::string& _info)
{
#ifdef RS_DRIVER
  RS_ERROR << _info << RS_REND;
#else
  std::cout << "\033[1m\033[31m" <<_info << "\033[0m" << std::endl;
#endif
  return;
}

inline void log_warning(const std::string& _info)
{
#ifdef RS_DRIVER
  RS_WARNING << _info << RS_REND;
#else
  std::cout << "\033[1m\033[33m" << _info << "\033[0m" << std::endl;
#endif
  return;
}

inline void log_info(const std::string& _info)
{
#ifdef RS_DRIVER
  RS_INFO << _info << RS_REND;
#else
  std::cout << "\033[1m\033[32m" << _info << "\033[0m" << std::endl;
#endif
  return;
}

inline void log_debug(const std::string& _info)
{
#ifdef RS_DRIVER
  RS_DEBUG << _info << RS_REND;
#else
  std::cout << "\033[1m\033[36m" << _info << "\033[0m" << std::endl;
#endif
  return;
}

/**
 * @brief calclute rotation from vector_a and vector_b.
 *
 * @param _va vector3 a.
 * @param _vb vector3 b.
 * @return R_b_a from _va coordinate to _vb coordinate.
 */
template <typename T> 
inline Eigen::Matrix<typename std::remove_reference<T>::type::Scalar, 3, 3> calRotationFromTwoVector(const T& _va, const T& _vb)
{
  typedef typename std::remove_reference<T>::type::Scalar Scalar;
  
  Eigen::Vector3d van = _va.normalized();
  Eigen::Vector3d vbn = _vb.normalized();
  
  Scalar theta_rad = std::acos(van.dot(vbn));
  Eigen::Matrix<Scalar, 3, 1> rotation_axis = van.cross(vbn).normalized();
  Eigen::AngleAxisd angle_axis(theta_rad, rotation_axis);
  Eigen::Matrix<Scalar, 3, 3> rotation_matrix = angle_axis.toRotationMatrix();
  
  return rotation_matrix;
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, 4, 4> eulerXYZToTransMatrix(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& roll, const Scalar& pitch, const Scalar& yaw) 
{
  Eigen::Transform<Scalar, 3, Eigen::Affine> transform = Eigen::Translation<Scalar, 3>(x, y, z) *
                                                         Eigen::AngleAxis<Scalar>(yaw, Eigen::Matrix<Scalar, 3, 1>::UnitZ()) *
                                                         Eigen::AngleAxis<Scalar>(pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()) *
                                                         Eigen::AngleAxis<Scalar>(roll, Eigen::Matrix<Scalar, 3, 1>::UnitX());
  return transform.matrix();
}

template <typename T>
inline Eigen::Matrix<typename std::remove_reference<T>::type::Scalar, 3, 1> eulerAnglesZYX(const T& q_in)
{
  typedef typename std::remove_reference<T>::type::Scalar Scalar;

  Eigen::Matrix<Scalar, 4, 1> q = q_in.normalized().coeffs();

  Scalar s = -2 * (q(0) * q(2) - q(3) * q(1));
  if (s > 1)
    s = 1;
  return (Eigen::Matrix<Scalar, 3, 1>()
              << atan2f(2 * (q(0) * q(1) + q(3) * q(2)), q(3) * q(3) + q(0) * q(0) - q(1) * q(1) - q(2) * q(2)),
          asin(s), atan2(2 * (q(1) * q(2) + q(3) * q(0)), q(3) * q(3) - q(0) * q(0) - q(1) * q(1) + q(2) * q(2)))
      .finished();
};

// struct OdomData
// {
//   double stamp;
//   double linear_velocity;
//   double steering_angle;
// };

struct IMUData {
    bool state;
    double timestamp;  // Time in nanoseconds
    float orientation_x;
    float orientation_y;
    float orientation_z;
    float orientation_w;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    float linear_acceleration_x;
    float linear_acceleration_y;
    float linear_acceleration_z;
    IMUData()
        : state{false},
          timestamp(0),
          orientation_x(0.0),
          orientation_y(0.0),
          orientation_z(0.0),
          orientation_w(1.0),
          angular_velocity_x(0.0),
          angular_velocity_y(0.0),
          angular_velocity_z(0.0),
          linear_acceleration_x(0.0),
          linear_acceleration_y(0.0),
          linear_acceleration_z(0.0) {}

    // Parameterized constructor to initialize all members
    IMUData(bool valid, double ts, float ori_x, float ori_y, float ori_z, float ori_w,
            float ang_vel_x, float ang_vel_y, float ang_vel_z,
            float lin_acc_x, float lin_acc_y, float lin_acc_z)
        : state{valid},
          timestamp(ts),
          orientation_x(ori_x),
          orientation_y(ori_y),
          orientation_z(ori_z),
          orientation_w(ori_w),
          angular_velocity_x(ang_vel_x),
          angular_velocity_y(ang_vel_y),
          angular_velocity_z(ang_vel_z),
          linear_acceleration_x(lin_acc_x),
          linear_acceleration_y(lin_acc_y),
          linear_acceleration_z(lin_acc_z) {}
    IMUData& operator=(const IMUData& other) {
        if (this != &other) {
            state = other.state;
            timestamp = other.timestamp;
            orientation_x = other.orientation_x;
            orientation_y = other.orientation_y;
            orientation_z = other.orientation_z;
            orientation_w = other.orientation_w;
            angular_velocity_x = other.angular_velocity_x;
            angular_velocity_y = other.angular_velocity_y;
            angular_velocity_z = other.angular_velocity_z;
            linear_acceleration_x = other.linear_acceleration_x;
            linear_acceleration_y = other.linear_acceleration_y;
            linear_acceleration_z = other.linear_acceleration_z;
        }
        return *this;
    }
    
    IMUData operator+(const IMUData& other) const {
        IMUData result;
        result.timestamp = std::max(timestamp, other.timestamp);
        result.angular_velocity_x = angular_velocity_x + other.angular_velocity_x;
        result.angular_velocity_y = angular_velocity_y + other.angular_velocity_y;
        result.angular_velocity_z = angular_velocity_z + other.angular_velocity_z;
        result.linear_acceleration_x = linear_acceleration_x + other.linear_acceleration_x;
        result.linear_acceleration_y = linear_acceleration_y + other.linear_acceleration_y;
        result.linear_acceleration_z = linear_acceleration_z + other.linear_acceleration_z;
        
        return result;
    }
    
    IMUData operator/(float scalar) const {
        IMUData result;
        result.angular_velocity_x = angular_velocity_x / scalar;
        result.angular_velocity_y = angular_velocity_y / scalar;
        result.angular_velocity_z = angular_velocity_z / scalar;
        result.linear_acceleration_x = linear_acceleration_x / scalar;
        result.linear_acceleration_y = linear_acceleration_y / scalar;
        result.linear_acceleration_z = linear_acceleration_z / scalar;
        return result;
    }
    
    void init()
    {
      state = false;
      timestamp = 0.0;
      orientation_x = 0.0;
      orientation_y = 0.0;
      orientation_z = 0.0;
      orientation_w = 1.0;
      angular_velocity_x = 0.0;
      angular_velocity_y = 0.0;
      angular_velocity_z = 0.0;
      linear_acceleration_x = 0.0;
      linear_acceleration_y = 0.0;
      linear_acceleration_z = 0.0;
    }
};

struct InteIMUData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMUData imu_data;
  Eigen::Vector3d p;  // position 
  Eigen::Vector3d v;  // v_local
  Eigen::Matrix3d R;  // R_global_local
  
  InteIMUData()
  {
    imu_data = IMUData();
    p = Eigen::Vector3d::Zero();
    v = Eigen::Vector3d::Zero();
    R = Eigen::Matrix3d::Identity();
  }
  
  InteIMUData(const IMUData& _data)
    : imu_data(_data)
  {
    p = Eigen::Vector3d::Zero();
    v = Eigen::Vector3d::Zero();
    R = Eigen::Matrix3d::Identity();
  }
  
  InteIMUData(const IMUData& _data, const Eigen::Vector3d& _v, const Eigen::Matrix3d& _R)
    : imu_data(_data),
      v(_v),
      R(_R)
  {
    p = Eigen::Vector3d::Zero();
  }
  
  InteIMUData(const IMUData& _data, const Eigen::Vector3d& _p, const Eigen::Vector3d& _v, const Eigen::Matrix3d& _R)
    : imu_data(_data),
      p(_p),
      v(_v),
      R(_R)
  {  }
  
  // copy
  InteIMUData(const InteIMUData& other)
    : imu_data(other.imu_data),
      p(other.p),
      v(other.v),
      R(other.R)
  {  }

  // =
  InteIMUData& operator=(const InteIMUData& other) {
    if (this != &other) {
      imu_data = other.imu_data;
      p = other.p;
      v = other.v;
      R = other.R;
    }
    return *this;
  }
};

class IMUModule
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMUModule(const YAML::Node& _cfg);  // 重载基类构造，且设置为public保证可以正常创建
  IMUModule(std::size_t _num);
  IMUModule(std::size_t _num, bool _use_linear_acceleration, double _gravity);
  IMUModule(std::size_t _num, bool _use_linear_acceleration, double _gravity, double _imu_frame_rate, double _lidar_frame_rate);
  
  bool addFrameData(const IMUData& _data);  // virtual 只是为了显式说明为虚函数派生，可以不加, 不加情况下再次派生还能多态吗？
  // bool addOdomData(const OdomData& _data);
  
  bool calAngleShift();
  bool calIMURot(double _st_stamp, double _end_stamp, Eigen::Matrix4d& _intergration_transform, std::size_t& _search_idx);
  bool calHeadIMURot(
    double _end_stamp, Eigen::Matrix4d& _intergration_transform, std::size_t& _search_idx, const bool _tail = false);
  bool findFrames(double _stamp, InteIMUData& _lhs, InteIMUData& _rhs, std::size_t& _search_idx);
  std::vector<std::size_t> getCurrentWindowCopy();

  bool setHeadStamp(double _stamp, MCStatus& _status, const double _tail = false);
  bool clearData(const std::size_t _left_boundry_idx);
  bool isDrifted();
  IMUData& back();
  std::size_t size();
  void pop_front();
  void pop_back();

  IMUData& getItem(std::size_t index);
  void print();
  std::vector<InteIMUData> imu_window_;
private:
  Eigen::Matrix4d getPose(const InteIMUData& _lhs, const InteIMUData& _rhs);
  Eigen::Matrix4d getInterpPose(double _stamp, const InteIMUData& _lhs, const InteIMUData& _rhs, bool reverse=false);
  
  std::deque<InteIMUData> data_frames_;
  
  Eigen::Vector3d gravity_vector_;  // global
  
  std::size_t num_window_ = 50;
  
  std::mutex mtx_;
  std::mutex drift_mtx_;
  IMUData drift_;
  bool is_drifted_ = false;
  bool use_linear_acceleration_ = false;
  
  double mointion_ratio_ = 0.5;
  double mointion_thres_ = 0.002;
  
  double static_ratio_ = 0.6;
  double static_thres_ = 0.001;
  
  double head_stamp_;
  std::size_t head_rhs_idx_ = 0.0;
  
  // for angle correction
  std::unordered_map<int, Eigen::Matrix4d> tmp_pose_;
  
  double imu_frame_rate_;
  double lidar_frame_rate_;
  
};

IMUModule::IMUModule(std::size_t _num)
  : num_window_(_num),
    use_linear_acceleration_(false),
    imu_frame_rate_(200),
    lidar_frame_rate_(10)
{
  drift_ = IMUData();
  gravity_vector_ = Eigen::Vector3d(0.0, 0.0, 9.81);
  this->print();
};

IMUModule::IMUModule(std::size_t _num, bool _use_linear_acceleration, double _gravity)
  : num_window_(_num),
    use_linear_acceleration_(_use_linear_acceleration),
    imu_frame_rate_(200),
    lidar_frame_rate_(10)
{
  drift_ = IMUData();
  gravity_vector_ = Eigen::Vector3d(0.0, 0.0, _gravity);
  this->print();
};

IMUModule::IMUModule(std::size_t _num, bool _use_linear_acceleration, double _gravity, double _imu_frame_rate, double _lidar_frame_rate)
  : num_window_(_num),
    use_linear_acceleration_(_use_linear_acceleration),
    imu_frame_rate_(_imu_frame_rate),
    lidar_frame_rate_(_lidar_frame_rate)
{
  drift_ = IMUData();
  gravity_vector_ = Eigen::Vector3d(0.0, 0.0, _gravity);
  this->print();
};

void IMUModule::print()
{
  log_info("------------------------------------------------------");
  log_info("IMU Module Config");
  log_info("Use Linear Acceleration: " + std::to_string(use_linear_acceleration_));
  log_info("Number Frame Drift: " + std::to_string(num_window_));
  log_info("Init Global Fravity: " + std::to_string(gravity_vector_[0]) + " " + std::to_string(gravity_vector_[1]) + " " + std::to_string(gravity_vector_[2]));
  log_info("------------------------------------------------------");
};

IMUData& IMUModule::back()
{
  std::lock_guard<std::mutex> lg(mtx_);
  return data_frames_.back().imu_data;
}

bool IMUModule::isDrifted()
{
  std::lock_guard<std::mutex> lg(drift_mtx_);
  return is_drifted_;
}

void IMUModule::pop_front()
{
  std::lock_guard<std::mutex> lg(mtx_);
  data_frames_.pop_front();
}

void IMUModule::pop_back()
{
  std::lock_guard<std::mutex> lg(mtx_);
  data_frames_.pop_back();
}

IMUData& IMUModule::getItem(std::size_t index)
{
  std::lock_guard<std::mutex> lg(mtx_);
  return data_frames_[index].imu_data;
}

std::size_t IMUModule::size()
{
  std::lock_guard<std::mutex> lg(mtx_);
  return data_frames_.size();
}

bool IMUModule::addFrameData(const IMUData& _data) 
{
  std::lock_guard<std::mutex> lg(mtx_);
  InteIMUData integration_imu_data(_data);
  
  double delta_stamp = data_frames_.empty() ? 0.005 : _data.timestamp - data_frames_.back().imu_data.timestamp;
  // if (delta_stamp < 0.004 || delta_stamp > 0.006)
  // {
  //   RS_WARNING << "Time step of IMU warning: " << std::to_string(delta_stamp) << std::endl;
  // }
  
  if (use_linear_acceleration_ && is_drifted_)
  {
    // 如果为空则lhs假设值和当前帧相同
    const InteIMUData lhs = data_frames_.empty() ? integration_imu_data : data_frames_.back();
    const Eigen::Matrix4d T_inter = getPose(lhs.imu_data, _data);
    const Eigen::Matrix3d R_current = lhs.R * T_inter.block<3, 3>(0, 0); // R_global_local
    integration_imu_data.R = R_current;
    const Eigen::Vector3d linear_acceleration_local = Eigen::Vector3d(_data.linear_acceleration_x, _data.linear_acceleration_y, _data.linear_acceleration_z);
    Eigen::Vector3d linear_velocity_local = lhs.v + delta_stamp * (linear_acceleration_local - R_current.transpose() * gravity_vector_);
    integration_imu_data.v = linear_velocity_local;
  }
  
  data_frames_.emplace_back(integration_imu_data);
  if (!is_drifted_ && data_frames_.size() > num_window_) 
  {
    std::lock_guard<std::mutex> lg_shift(drift_mtx_);
    
    for (auto it = data_frames_.begin(); it != data_frames_.begin() + num_window_; ++it) 
    {
      drift_ = drift_ + it->imu_data;
    }
    drift_ = drift_ / static_cast<double>(num_window_);
    is_drifted_ = true;
    
    std::ostringstream oss;
    oss << "Montion Correct IMU drifted finished!" 
              << " angle_vx: "<< std::to_string(drift_.angular_velocity_x) << " angle_vy: "<< std::to_string(drift_.angular_velocity_y) << " angle_vz: "<< std::to_string(drift_.angular_velocity_z)
              << " linear_ax: "<< std::to_string(drift_.linear_acceleration_x) << " linear_ay: "<< std::to_string(drift_.linear_acceleration_y) << " linear_az: "<< std::to_string(drift_.linear_acceleration_z);
    log_info(oss.str());
    
    if (use_linear_acceleration_)
    {
      Eigen::Vector3d gravity_vector_local(drift_.linear_acceleration_x, drift_.linear_acceleration_y, drift_.linear_acceleration_z);
      double gravity = gravity_vector_local.norm();
      gravity_vector_ = Eigen::Vector3d(0.0, 0.0, gravity);
      
      // calculate positon from gravity
      Eigen::Matrix3d R_global_local = calRotationFromTwoVector(gravity_vector_local, gravity_vector_);
      
      Eigen::Vector3d rpy = eulerAnglesZYX(Eigen::Quaterniond(R_global_local)) * (180.0 / M_PI);
      Eigen::Vector3d gravity_vector_local_fixed = R_global_local.transpose() * gravity_vector_;
      log_info("Local Gravity         : " + std::to_string(gravity_vector_local[0]) + " " + std::to_string(gravity_vector_local[1]) + " " + std::to_string(gravity_vector_local[2]));
      log_info("Global Gravity        : " + std::to_string(gravity_vector_[0]) + " " + std::to_string(gravity_vector_[1]) + " " + std::to_string(gravity_vector_[2]));
      log_info("Global Gravity Fixed  : " + std::to_string(gravity_vector_local_fixed[0]) + " " + std::to_string(gravity_vector_local_fixed[1]) + " " + std::to_string(gravity_vector_local_fixed[2]));
      log_info("Angle (rpy)           : " + std::to_string(rpy[0]) + " " + std::to_string(rpy[1]) + " " + std::to_string(rpy[2]));
      for (auto it = data_frames_.begin(); it != data_frames_.end(); ++it) 
      {
        it->R = R_global_local;
      }
    }
  }
  
  return true;
}

bool IMUModule::findFrames(double _stamp, InteIMUData& _lhs, InteIMUData& _rhs, std::size_t& _search_idx)
{
  if (_stamp < imu_window_.front().imu_data.timestamp)
  {
    // RS_WARNING << std::fixed << std::setprecision(9) << "Montion Correct Warning, findFrames stamp: " << _stamp << " less than front " << imu_window_.front().imu_data.timestamp << std::endl;
    return false;
  }
  
  if (_stamp > imu_window_.back().imu_data.timestamp)
  {
    // RS_WARNING << std::fixed << std::setprecision(9) << "Montion Correct Warning, findFrames stamp: " << _stamp << " more than back " << imu_window_.back().imu_data.timestamp << std::endl;
    return false;
  }
  
  if (_search_idx >= imu_window_.size())
  {
    _search_idx = imu_window_.size() - 1;
    log_warning("Montion Correct Warning, findFrames search_idx outof range");
    return false;
  }
  
  std::size_t search_idx = _search_idx;
  while (search_idx < imu_window_.size() && _stamp > imu_window_.at(search_idx).imu_data.timestamp)
  {
    search_idx++;
  }
  
  while (search_idx > 1 && _stamp <= imu_window_.at(search_idx-1).imu_data.timestamp)
  {
    search_idx--;
  }
  
  if(search_idx > 1 && _stamp > imu_window_.at(search_idx-1).imu_data.timestamp 
          && search_idx < imu_window_.size() && _stamp <= imu_window_.at(search_idx).imu_data.timestamp)
  {
    _lhs = imu_window_.at(search_idx-1);
    _rhs = imu_window_.at(search_idx);
    _search_idx = search_idx;
  }
  else
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9) << "Montion Correct Warning, findFrames error" 
      << " _search_idx: " << std::to_string(_search_idx)
      << " size: " << std::to_string(imu_window_.size())
      << " stamp: " << std::to_string(_stamp)
      << " low stamp" << std::to_string(imu_window_.at(search_idx-1).imu_data.timestamp)
      << " high stamp" << std::to_string(imu_window_.at(search_idx).imu_data.timestamp);
    log_warning(oss.str());
    return false;
  }
  
  return true;
}

bool IMUModule::calIMURot(double _st_stamp, double _end_stamp, Eigen::Matrix4d& _intergration_transform, std::size_t& _lhs_idx) 
{
  std::lock_guard<std::mutex> lg(mtx_); 
  if (!is_drifted_ || data_frames_.empty()) 
  {
    return false;
  }
  
  if (_st_stamp < data_frames_.front().imu_data.timestamp || _end_stamp > data_frames_.back().imu_data.timestamp) 
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9) << "Montion Correct Warning calIMURot Wrong Stamp! " 
      << data_frames_.front().imu_data.timestamp << " "
      << _st_stamp << " " 
      << data_frames_.back().imu_data.timestamp << " " 
      << _end_stamp << " ";
    log_warning(oss.str());
    return false;
  }
  
  // std::size_t lhs_idx = _lhs_idx;
  std::size_t search_rhs_thres = data_frames_.size() - 1;
  
  Eigen::Matrix4d transfrom = Eigen::Matrix4d::Identity();
  
  for (std::size_t i = 0; i < search_rhs_thres; i++) 
  {
    auto lhs = data_frames_[i];
    auto rhs = data_frames_[i+1];
    if (rhs.imu_data.timestamp < _st_stamp) 
    {
      continue;
    }
    
    if (lhs.imu_data.timestamp > _st_stamp && rhs.imu_data.timestamp < _end_stamp) 
    {
      transfrom *= getPose(lhs, rhs);
    }
    else if (lhs.imu_data.timestamp < _st_stamp && rhs.imu_data.timestamp >= _st_stamp) 
    {
      transfrom *= getInterpPose(_st_stamp, lhs, rhs, true);
    }
    else if (lhs.imu_data.timestamp < _end_stamp && rhs.imu_data.timestamp >= _end_stamp) 
    {
      transfrom *= getInterpPose(_end_stamp, lhs, rhs);
      _lhs_idx = i + 1;
      break;
    }
    else if (lhs.imu_data.timestamp > _end_stamp && rhs.imu_data.timestamp > _end_stamp) 
    {
      _lhs_idx = i;
      break;
    }
  }
  
  _intergration_transform = transfrom;
  
  return true;
}

Eigen::Matrix4d IMUModule::getPose(const InteIMUData& _lhs, const InteIMUData& _rhs)
{
  Eigen::Matrix4d delta_trans = Eigen::Matrix4d::Identity();
  IMUData avg_imu_data = (_lhs.imu_data + _rhs.imu_data) / 2;
  Eigen::Vector3d mid_rotvec;
  
  {
    std::lock_guard<std::mutex> lg_shift(drift_mtx_);
    mid_rotvec = Eigen::Vector3d (
      avg_imu_data.angular_velocity_x - drift_.angular_velocity_x,
      avg_imu_data.angular_velocity_y - drift_.angular_velocity_y,
      avg_imu_data.angular_velocity_z - drift_.angular_velocity_z
    );
  }
  
  double dt = _rhs.imu_data.timestamp - _lhs.imu_data.timestamp;
  // if (dt < 0.004 || dt > 0.006)
  // {
  //   RS_WARNING << "Time step of IMU warning: " << std::to_string(dt) << std::endl;
  // }
  mid_rotvec *= dt;
  Eigen::AngleAxisd rotation_vector(mid_rotvec.norm(), mid_rotvec.normalized());
  
  delta_trans.block<3, 3>(0, 0) = rotation_vector.toRotationMatrix();
  
  if (use_linear_acceleration_)
  {
    // translation
    const Eigen::Vector3d ag = Eigen::Vector3d(avg_imu_data.linear_acceleration_x, avg_imu_data.linear_acceleration_y, avg_imu_data.linear_acceleration_z);
    const Eigen::Vector3d a = ag - _lhs.R.transpose() * gravity_vector_;
    const Eigen::Vector3d v1 = _lhs.v + dt * a;
    Eigen::Vector3d translation = dt * (_lhs.v + v1) / 2.0;
   
#ifdef VIZ_DEBUG
    const Eigen::Vector3d gravity_local_vector = _lhs.R.transpose() * gravity_vector_;
    // std::cout << "R: " << _lhs.R << std::endl;
    std::cout << "acceleration: " << a[0] << ", " << a[1] << ", " << a[2] << std::endl;
    std::cout << "gravity_local_vector: " << gravity_local_vector[0] << ", " << gravity_local_vector[1] << ", " << gravity_local_vector[2] << std::endl;
    std::cout << "v0: " << _lhs.v[0] << ", " << _lhs.v[1] << ", " << _lhs.v[2] << std::endl;
    std::cout << "v1: " << v1[0] << ", " << v1[1] << ", " << v1[2] << std::endl;
    std::cout << "translation: " << translation[0] << ", " << translation[1] << ", " << translation[2] << std::endl;
#endif
    delta_trans.block<3, 1>(0, 3) = translation;  
  }
  return delta_trans;
}

Eigen::Matrix4d IMUModule::getInterpPose(double _stamp, const InteIMUData& _lhs, const InteIMUData& _rhs, bool reverse)
{
  Eigen::Matrix4d hole_trans = getPose(_lhs, _rhs);
  Eigen::Matrix3d hole_rotation_matrix = hole_trans.block<3,3>(0,0);
  
  Eigen::Quaterniond q1(Eigen::Matrix3d::Identity());
  Eigen::Quaterniond q2(hole_rotation_matrix);
  double scale = abs(_stamp - _lhs.imu_data.timestamp) / abs(_rhs.imu_data.timestamp - _lhs.imu_data.timestamp);
  Eigen::Quaterniond q_interp = q1.slerp(scale, q2);
  Eigen::Matrix4d trans_interp = Eigen::Matrix4d::Identity();
  trans_interp.block<3, 3>(0, 0) = q_interp.toRotationMatrix();
  
  if (use_linear_acceleration_)
  {
    Eigen::Vector3d interp_translation = hole_trans.block<3, 1>(0,3);
    trans_interp.block<3, 1>(0,3) = scale * interp_translation;
  }
  
  if (reverse)
  {
    return trans_interp.inverse() * hole_trans;
  }
  return trans_interp;
}

bool IMUModule::calHeadIMURot(double _cur_stamp, Eigen::Matrix4d& _intergration_transform, std::size_t& _search_idx, const bool _tail) 
{
  if (!is_drifted_)
  {
    return false;
  }
  
  InteIMUData lhs, rhs;
  if (!findFrames(_cur_stamp, lhs, rhs, _search_idx))
  {
    log_warning("Montion Correct Warning, calHeadIMURot find Frames Failed");;
    return false;
  }
  
  Eigen::Matrix4d head_trans, tail_trans;
  if (_tail)
  {
    // mid trans
    tail_trans = tmp_pose_[-1];
    if (tmp_pose_.find(static_cast<int>(_search_idx)) != tmp_pose_.end())
    {
      tail_trans = tmp_pose_[static_cast<int>(_search_idx)];
    }
    else
    {
      std::size_t st_loop = head_rhs_idx_-1, end_loop = _search_idx;
      for (std::size_t idx = st_loop; idx > end_loop && idx > 0; --idx)
      {
        tail_trans = getPose(imu_window_.at(idx-1), imu_window_.at(idx)) * tail_trans;
      }
      tmp_pose_[static_cast<int>(_search_idx)] = tail_trans;
    }
    head_trans = getInterpPose(_cur_stamp, lhs, rhs, true);
  }
  else
  {
    // mid trans
    head_trans = tmp_pose_[-1];
    if (tmp_pose_.find(static_cast<int>(_search_idx-1)) != tmp_pose_.end())
    {
      head_trans = tmp_pose_[static_cast<int>(_search_idx-1)];
    }
    else
    {
      std::size_t st_loop = head_rhs_idx_, end_loop = _search_idx > 1 ? _search_idx : 1;
      
      for (std::size_t idx = st_loop; idx < (end_loop-1) && idx < imu_window_.size(); idx++)
      {
        head_trans *= getPose(imu_window_.at(idx), imu_window_.at(idx+1));
      }
      tmp_pose_[static_cast<int>(_search_idx-1)] = head_trans;
    }
    
    /* quaternons spherical interpolation for the tail transform */
    tail_trans = getInterpPose(_cur_stamp, lhs, rhs, false);
  }
  
#ifdef VIZ_DEBUG
  Eigen::Vector3d ypr_head = eulerAnglesZYX(Eigen::Quaterniond(head_trans.block<3,3>(0,0)));
  Eigen::Vector3d ypr_tail = eulerAnglesZYX(Eigen::Quaterniond(tail_trans.block<3,3>(0,0)));
  
  std::cout << "ypr_head: " << ypr_head[2]  * (180.0 / M_PI) << ", " << ypr_head[1] * (180.0 / M_PI) << ", " << ypr_head[0]  * (180.0 / M_PI)<< std::endl;
  std::cout << "ypr_tail: " << ypr_tail[2]  * (180.0 / M_PI) << ", " << ypr_tail[1] * (180.0 / M_PI) << ", " << ypr_tail[0]  * (180.0 / M_PI)<< std::endl;
#endif

  _intergration_transform = head_trans * tail_trans;
  return true;
}

bool IMUModule::setHeadStamp(double _stamp, MCStatus& _status, const double _tail)
{
  if (!is_drifted_) 
  {
    _status = MCStatus::WAITING;
    return false;
  }
  
  head_stamp_ = _stamp;
  tmp_pose_.clear();
  imu_window_.clear();

  InteIMUData lhs, rhs;
  std::size_t rhs_idx = 1;
  
  double num_imu_per_lidar_fram = imu_frame_rate_ / lidar_frame_rate_;
  std::size_t min_num_window = static_cast<std::size_t>(std::ceil(1.2 * num_imu_per_lidar_fram));
  std::size_t left_thres = static_cast<std::size_t>(std::ceil(0.5 * num_imu_per_lidar_fram));
  std::size_t right_thres = static_cast<std::size_t>(std::ceil(1.5 * num_imu_per_lidar_fram));
  
  if (_tail)
  {
    double tmp = left_thres;
    left_thres = right_thres;
    right_thres = tmp;
  }
  // RS_DEBUG << "min_num_window: " << min_num_window
  //           << " left_thres: " << left_thres 
  //           << " right_thres: " << right_thres << std::endl;
  
  {
    std::lock_guard<std::mutex> lg(mtx_);
    bool find_hs = false;
    
    for (auto it = data_frames_.begin() + 1; it != data_frames_.end(); ++it)
    {
      lhs = *(it - 1);
      rhs = *it;
      // RS_DEBUG << std::to_string(lhs.timestamp) << " " << std::to_string(_stamp) << " " << std::to_string(rhs.timestamp) << " " << std::endl;
      if (_stamp > lhs.imu_data.timestamp && _stamp <= rhs.imu_data.timestamp)
      { 
        find_hs = true;
        // RS_DEBUG << "find hs: " << std::to_string(lhs.imu_data.timestamp) << " "  << std::to_string(_stamp) << " " << std::to_string(rhs.imu_data.timestamp) << std::endl;
        break;
      }
      rhs_idx++;
    }
  
    if (!find_hs)
    {
      // RS_WARNING << "Montion Correct Warning, setHeadStamp Find Frames Failed, Skip Frame!" << std::endl;
      _status = MCStatus::SKIPT;
      return false;
    }
  
    auto start_it = (rhs_idx >= left_thres) ? (data_frames_.begin() + rhs_idx - left_thres) : data_frames_.begin();
    std::size_t left_boundry = (rhs_idx >= left_thres) ? rhs_idx - left_thres : 0;
    
    auto end_it = (rhs_idx + right_thres < data_frames_.size()) ? (data_frames_.begin() + rhs_idx + right_thres) : data_frames_.end();
    std::size_t right_boundry = (rhs_idx + right_thres < data_frames_.size()) ? rhs_idx + right_thres : data_frames_.size();
    
    auto it = start_it;
    for (std::size_t idx = left_boundry; idx < right_boundry && it < end_it; ++it, ++idx)
    {
      imu_window_.emplace_back(*it);
      if (it->imu_data.timestamp == rhs.imu_data.timestamp)
      {
        head_rhs_idx_ = imu_window_.size() - 1;
      }
    }
    
    if (imu_window_.size() < min_num_window || imu_window_.front().imu_data.timestamp > _stamp || imu_window_.back().imu_data.timestamp < _stamp)
    {
      std::ostringstream oss;
      if (!imu_window_.empty())
      {
        oss << std::fixed << std::setprecision(9) << "Montion Correct Warning, SetHeadStamp Failed find Frames!"
            << " window size: " << std::to_string(imu_window_.size())
            << " rhs_idx: " << rhs_idx << " " << left_boundry << " " << right_boundry
            << " head stamp:  " << std::to_string(imu_window_.front().imu_data.timestamp)
            << " stamp: " << std::to_string(_stamp)
            << " tail stamp:  " << std::to_string(imu_window_.back().imu_data.timestamp)
            << " data_frames: " << std::to_string(data_frames_.size()) << " " << data_frames_.back().imu_data.timestamp;
      }
      else
      {
        oss << std::fixed << std::setprecision(9) << "Montion Correct Warning, SetHeadStamp Failed find Frames!"
            << " window size: " << std::to_string(imu_window_.size())
            << " rhs_idx: " << rhs_idx << " " << left_boundry << " " << right_boundry
            << " stamp: " << std::to_string(_stamp)
            << " data_frames: " << std::to_string(data_frames_.size()) << " " << std::to_string(data_frames_.front().imu_data.timestamp) << " " << std::to_string(data_frames_.back().imu_data.timestamp);
      }
      log_warning(oss.str());
      _status = MCStatus::WAITING;
      return false;
    }
  }

  // 移除数据， 避免数据积累
  std::size_t left_boundry_idx = std::max(static_cast<int>(rhs_idx) - static_cast<int>(left_thres) - 1, 0);
  clearData(left_boundry_idx);
  
  if (_tail)
  {
    Eigen::Matrix4d tail_trans = getInterpPose(_stamp, lhs, rhs, false);
    tmp_pose_[-1] = tail_trans;
    Eigen::Matrix4d mid_trans = tail_trans;
    for (std::size_t rhs_idx = head_rhs_idx_ - 1; rhs_idx > 1 && rhs_idx < imu_window_.size(); --rhs_idx)
    {
      Eigen::Matrix4d delta = getPose(imu_window_[rhs_idx-1], imu_window_[rhs_idx]);
      mid_trans = delta * mid_trans;
      tmp_pose_[rhs_idx-1] = mid_trans; // deep copy
    }
  }
  else
  {
    Eigen::Matrix4d head_trans = getInterpPose(_stamp, lhs, rhs, true);
    tmp_pose_[-1] = head_trans;
    Eigen::Matrix4d mid_trans = head_trans;
    for (std::size_t rhs_idx = head_rhs_idx_ + 1; rhs_idx > 1 && rhs_idx < imu_window_.size(); ++rhs_idx)
    {
      Eigen::Matrix4d delta = getPose(imu_window_[rhs_idx-1], imu_window_[rhs_idx]);
      mid_trans *= delta;
      tmp_pose_[rhs_idx] = mid_trans; // deep copy
    }
  }
  // RS_DEBUG << "setHeadStamp success, size:  " << imu_window_.size() << " "
  //          << std::to_string(imu_window_.front().imu_data.timestamp) << " "
  //          << std::to_string(imu_window_.back().imu_data.timestamp) << " " << std::endl;
  _status = MCStatus::SUCCESS;
  return true;
}

bool IMUModule::clearData(const std::size_t left_boundry_idx)
{
  std::lock_guard<std::mutex> lg(mtx_);
  std::size_t idx = 0;
  while (!data_frames_.empty() && idx++ < left_boundry_idx)
  {
    data_frames_.pop_front();
  }
  return true;
}

}
}