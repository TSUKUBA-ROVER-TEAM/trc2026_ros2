cmake_minimum_required(VERSION 3.8)
project(trc2026_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(robstride_bridge SHARED
  src/robstride_bridge.cpp
)

rclcpp_components_register_node(robstride_bridge
  PLUGIN "trc2026_driver::RobstrideBridge"
  EXECUTABLE robstride_bridge_node
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/can_tx_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME can_tx_node
)

install(PROGRAMS
  scripts/can_rx_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME can_rx_node
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
