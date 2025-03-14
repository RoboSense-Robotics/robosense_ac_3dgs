cmake_minimum_required(VERSION 3.0.2)
project(extract_images)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  cv_bridge
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(parser src/main.cpp)
target_link_libraries(parser ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
