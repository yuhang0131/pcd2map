cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_costmap)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(crp_sdk_interfaces REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pointcloud_to_costmap_node src/pointcloud_to_costmap_node.cpp)

ament_target_dependencies(pointcloud_to_costmap_node
  rclcpp
  sensor_msgs
  pcl_conversions
  std_msgs
  crp_sdk_interfaces
)

target_link_libraries(pointcloud_to_costmap_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS
  pointcloud_to_costmap_node
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# 安装参数文件
install(DIRECTORY
  param/
  DESTINATION share/${PROJECT_NAME}/param)

ament_package()
