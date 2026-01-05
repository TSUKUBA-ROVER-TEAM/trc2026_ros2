cmake_minimum_required(VERSION 3.8)
project(trc2026_manual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(drive_manual_controller SHARED
  src/drive_manual_controller.cpp
)

ament_target_dependencies(drive_manual_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  rclcpp_components
)

add_library(joy_filter SHARED
  src/joy_filter.cpp
)
ament_target_dependencies(joy_filter
  rclcpp
  sensor_msgs
  rclcpp_components
)

target_include_directories(drive_manual_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_include_directories(joy_filter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

add_library(${PROJECT_NAME}::drive_manual_controller ALIAS drive_manual_controller)

rclcpp_components_register_node(drive_manual_controller
  PLUGIN "trc2026_manual::DriveManualController"
  EXECUTABLE drive_manual_controller_node
)

rclcpp_components_register_node(joy_filter
  PLUGIN "trc2026_manual::JoyFilter"
  EXECUTABLE joy_filter_node
)

install(TARGETS drive_manual_controller joy_filter
  DESTINATION lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# joy_filter_node is installed by rclcpp_components_register_node automatically to bin,
# but if it needs to be in lib/<pkg> we might need manual install or check where register_node puts it.
# Usually EXECUTABLE argument puts it in lib/<pkg>.


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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
