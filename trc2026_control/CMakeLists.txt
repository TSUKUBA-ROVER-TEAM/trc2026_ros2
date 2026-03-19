cmake_minimum_required(VERSION 3.8)
project(trc2026_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(four_wheel_steer_controller SHARED
  src/four_wheel_steer_controller.cpp
)

ament_target_dependencies(four_wheel_steer_controller
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclcpp_components
)

target_include_directories(four_wheel_steer_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

add_library(${PROJECT_NAME}::four_wheel_steer_controller ALIAS four_wheel_steer_controller)

rclcpp_components_register_node(four_wheel_steer_controller
  PLUGIN "trc2026_control::FourWheelSteerController"
  EXECUTABLE four_wheel_steer_controller_node
)

# Aruco Mission Node
add_library(aruco_mission_node_component SHARED
  src/aruco_mission_node.cpp
)
target_include_directories(aruco_mission_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(aruco_mission_node_component
  rclcpp
  rclcpp_components
  ament_index_cpp
  geometry_msgs
  aruco_opencv_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

rclcpp_components_register_node(aruco_mission_node_component
  PLUGIN "trc2026_control::ArucoMissionNode"
  EXECUTABLE aruco_mission_node
)

add_library(${PROJECT_NAME}::aruco_mission_node_component ALIAS aruco_mission_node_component)

add_library(control_history_csv_logger_node_component SHARED
  src/control_history_csv_logger_node.cpp
)
target_include_directories(control_history_csv_logger_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(control_history_csv_logger_node_component
  rclcpp
  rclcpp_components
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

rclcpp_components_register_node(control_history_csv_logger_node_component
  PLUGIN "trc2026_control::ControlHistoryCsvLoggerNode"
  EXECUTABLE control_history_csv_logger_node
)

add_library(${PROJECT_NAME}::control_history_csv_logger_node_component ALIAS control_history_csv_logger_node_component)

install(TARGETS 
  four_wheel_steer_controller
  aruco_mission_node_component
  control_history_csv_logger_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
