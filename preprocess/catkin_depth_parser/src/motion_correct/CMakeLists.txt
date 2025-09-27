cmake_minimum_required(VERSION 3.12)
project(motion_correction)

add_definitions(-std=c++17)
add_definitions(-Wall -O3)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)
# add_definitions(-Wall -O0 -g)
# set(CMAKE_BUILD_TYPE RelWithDebInfo)

# add_subdirectory(../rslidar_sdk/src/rs_driver ${CMAKE_BINARY_DIR}/rslidar_sdk_build)
# find_package(rs_driver REQUIRED)
# include_directories(${rs_driver_INCLUDE_DIRS})

# include_directories(../rslidar_sdk/src/)

include_directories(include)
include_directories(utility)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(yaml-cpp REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem system) # Add other components as needed

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

catkin_package(
  CATKIN_DEPENDS   roscpp std_msgs pcl_ros cv_bridge
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS
)

# Add executable
add_executable(demo_lidar2cam src/demo_lidar2cam.cpp)

# Link libraries
target_link_libraries(
  demo_lidar2cam
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
  ${PCL_LIBRARIES}
  # ${rs_driver_LIBRARIES}
  yaml-cpp
  pthread
  ${Boost_LIBRARIES}
)

target_include_directories(demo_lidar2cam PRIVATE ${catkin_INCLUDE_DIRS})
