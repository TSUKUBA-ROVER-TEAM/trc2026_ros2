cmake_minimum_required(VERSION 3.8)
project(trc2026_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(four_wheel_steer_controller SHARED
  src/drive/four_wheel_steer_controller.cpp
)

ament_auto_add_library(led_controller SHARED
  src/led/led_controller.cpp
)

rclcpp_components_register_node(four_wheel_steer_controller
  PLUGIN "trc2026_control::FourWheelSteerController"
  EXECUTABLE four_wheel_steer_controller_node
)

rclcpp_components_register_node(led_controller
  PLUGIN "trc2026_control::LedController"
  EXECUTABLE led_controller_node
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

ament_auto_package()
