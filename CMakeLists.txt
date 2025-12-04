cmake_minimum_required(VERSION 3.8)
project(robo_diferenciado)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


add_executable(odom src/odom.cpp)
ament_target_dependencies(odom rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(drive_robot_stamped src/example_cmd_publisher.cpp)
ament_target_dependencies(drive_robot_stamped rclcpp geometry_msgs)

add_executable(trajectory_generator src/trajectory_generator.cpp)
ament_target_dependencies(trajectory_generator rclcpp geometry_msgs nav_msgs rclcpp_action)

add_executable(path_follower src/path_follower.cpp)
ament_target_dependencies(path_follower rclcpp geometry_msgs nav_msgs rclcpp_action tf2_geometry_msgs)

# Interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowPath.action"
  DEPENDENCIES nav_msgs
)

rosidl_target_interfaces(trajectory_generator
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(path_follower
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install files
install(
  DIRECTORY action src launch config worlds models urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  odom
  drive_robot_stamped
  trajectory_generator
  path_follower
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")


ament_package()
