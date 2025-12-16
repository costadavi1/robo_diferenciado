cmake_minimum_required(VERSION 3.8)
project(robo_diferenciado)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GeneratePath.srv"
  DEPENDENCIES nav_msgs geometry_msgs
)

ament_export_libraries(${PROJECT_NAME}__rosidl_typesupport_cpp)


add_executable(odom src/odom.cpp)
ament_target_dependencies(odom rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  odom
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(drive_robot_stamped src/example_cmd_publisher.cpp)
ament_target_dependencies(drive_robot_stamped rclcpp geometry_msgs)


install(TARGETS
  drive_robot_stamped
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(map_reader src/map_reader.cpp)
ament_target_dependencies(map_reader ament_cmake rclcpp nav_msgs)

target_link_libraries(map_reader
  yaml-cpp
)

install(TARGETS
  map_reader
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src launch config worlds models urdf rviz map
  DESTINATION share/${PROJECT_NAME}
)

add_executable(astar_planner src/astar_planner.cpp)
ament_target_dependencies(astar_planner rclcpp nav_msgs geometry_msgs)

target_link_libraries(astar_planner
  yaml-cpp
)

rosidl_target_interfaces(astar_planner
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  astar_planner
  DESTINATION lib/${PROJECT_NAME})

add_executable(Map src/Map.cpp)
ament_target_dependencies(Map rclcpp nav_msgs geometry_msgs)

target_link_libraries(Map
  yaml-cpp
)

install(TARGETS
  Map
  DESTINATION lib/${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(map_viewer src/map_viewer.cpp)

target_link_libraries(map_viewer
    sfml-system
    sfml-window
    sfml-graphics
)
install(TARGETS
  map_viewer
  DESTINATION lib/${PROJECT_NAME})


add_executable(map_viewer_astar src/map_viewer_astar.cpp)

target_link_libraries(map_viewer_astar
    sfml-system
    sfml-window
    sfml-graphics
)
install(TARGETS
  map_viewer_astar
  DESTINATION lib/${PROJECT_NAME})

  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")


ament_package()
