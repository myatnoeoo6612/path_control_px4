cmake_minimum_required(VERSION 3.8)
project(vision LANGUAGES CXX)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation features search)
find_package(pcl_conversions REQUIRED)
find_package(image_transport REQUIRED)
find_package(px4_msgs REQUIRED)


include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(camera_recorder src/camera_recorder.cpp)

ament_target_dependencies(camera_recorder 
rclcpp 
sensor_msgs 
cv_bridge
)

add_executable(depth_polygon_mask_node src/depth_polygon_mask_node.cpp)
add_executable(pointcloud_node src/pointcloud_node.cpp)
add_executable(aruco_detect  src/aruco_detect.cpp)
add_executable(lidar src/lidar.cpp)
add_executable(obstacle_node src/obstacle.cpp)

ament_target_dependencies(depth_polygon_mask_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  image_geometry
)

ament_target_dependencies(obstacle_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  image_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  px4_msgs 
)

ament_target_dependencies(pointcloud_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  image_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

ament_target_dependencies(lidar
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  image_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)


ament_target_dependencies(aruco_detect
  rclcpp
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
  tf2
  tf2_geometry_msgs
)

target_link_libraries(aruco_detect
  ${OpenCV_LIBS}
)

target_link_libraries(
  camera_recorder
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
target_link_libraries(
  pointcloud_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
target_link_libraries(
  obstacle_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
target_link_libraries(
  lidar
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

install(TARGETS 
  camera_recorder
  depth_polygon_mask_node
  pointcloud_node
  aruco_detect
  lidar
  obstacle_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
