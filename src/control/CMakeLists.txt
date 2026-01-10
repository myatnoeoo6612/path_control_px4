cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(trajectory_fly src/trajectory_fly.cpp)
ament_target_dependencies(trajectory_fly
  rclcpp 
  px4_msgs 
  std_msgs 
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  visualization_msgs
)

add_executable(auto_fly_test src/auto_fly_test.cpp)
ament_target_dependencies(auto_fly_test
  rclcpp 
  px4_msgs 
  std_msgs 
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  visualization_msgs
)


add_executable(px4_visualizer src/px4_visualizer.cpp)
ament_target_dependencies(px4_visualizer
  rclcpp
  px4_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
)

add_executable(tf_node src/tf_node.cpp)
ament_target_dependencies(tf_node
  rclcpp
  std_msgs
  px4_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(land src/land.cpp)
ament_target_dependencies(land
  rclcpp 
  px4_msgs 
  std_msgs 
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  visualization_msgs
)

add_executable(adaptive_land src/adaptive_land.cpp)
ament_target_dependencies(adaptive_land
  rclcpp 
  px4_msgs 
  std_msgs 
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  visualization_msgs
)

install(TARGETS
  tf_node
  trajectory_fly
  px4_visualizer
  auto_fly_test
  land
  adaptive_land
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${px4_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  "/opt/ros/humble/include/visualization_msgs"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
