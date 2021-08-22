cmake_minimum_required(VERSION 3.0.2)
project(golfcart_push)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  image_transport
  image_geometry
  cv_bridge
  roscpp
  rospy
  pcl_ros
  std_msgs
  geometry_msgs
  avs_lecture_msgs
)
find_package(OpenCV REQUIRED)
# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_compile_options(-std=c++17)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/GolfcartPush.cfg
)

catkin_package(
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(hsv_example_node
  src/hsv_example_node.cpp
  src/HsvExample.cpp
)
add_dependencies(hsv_example_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(hsv_example_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(lidar_filter
  src/lidar_filter_node.cpp
  src/lidar_filter.cpp
)
add_dependencies(lidar_filter
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(lidar_filter
  ${catkin_LIBRARIES}
)

add_executable(camera_vision
  src/camera_vision_node.cpp
  src/camera_vision.cpp
)
add_dependencies(camera_vision
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(camera_vision
  ${catkin_LIBRARIES}
)

add_executable(r3d3_joint_pub
  src/main_r3d3_joint_pub.cpp
  src/r3d3JointPub.cpp
)
add_dependencies(r3d3_joint_pub
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(r3d3_joint_pub
  ${catkin_LIBRARIES}
)



