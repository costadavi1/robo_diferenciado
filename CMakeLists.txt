cmake_minimum_required(VERSION 3.8)
project(robo_diferenciado)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Eigen3 REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GeneratePath.srv"
  "action/FollowPath.action"
  DEPENDENCIES nav_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(odom src/odom.cpp)
ament_target_dependencies(odom rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(drive_robot_stamped src/example_cmd_publisher.cpp)
ament_target_dependencies(drive_robot_stamped rclcpp geometry_msgs)

add_executable(trajectory_generator src/trajectory_generator.cpp)
ament_target_dependencies(trajectory_generator rclcpp geometry_msgs nav_msgs rclcpp_action Eigen3 tf2_geometry_msgs)

add_executable(path_follower src/path_follower.cpp)
ament_target_dependencies(path_follower rclcpp geometry_msgs nav_msgs rclcpp_action tf2_geometry_msgs)

add_executable(astar_planner src/astar_planner.cpp)
ament_target_dependencies(astar_planner rclcpp nav_msgs geometry_msgs)
target_link_libraries(astar_planner
  yaml-cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(astar_planner
  ${cpp_typesupport_target})

target_link_libraries(trajectory_generator
  ${cpp_typesupport_target})

target_link_libraries(path_follower
  ${cpp_typesupport_target})



# Install directories
install(
  DIRECTORY action src launch config worlds models urdf rviz map
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  odom
  drive_robot_stamped
  trajectory_generator
  path_follower
  astar_planner
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
