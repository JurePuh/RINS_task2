cmake_minimum_required(VERSION 3.8)
project(barrel_leak_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters features segmentation search)
find_package(OpenCV REQUIRED)
find_package(msg_types REQUIRED)

add_executable(detect_barrel_cpp src/barrel_leak_detector.cpp)
target_compile_features(detect_barrel_cpp PUBLIC cxx_std_17)
target_include_directories(detect_barrel_cpp SYSTEM PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(detect_barrel_cpp
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(detect_barrel_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  message_filters
  pcl_conversions
  msg_types
)

install(TARGETS detect_barrel_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
