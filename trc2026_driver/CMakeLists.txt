cmake_minimum_required(VERSION 3.8)
project(trc2026_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/robstride_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robstride_bridge_node
)

install(PROGRAMS
  scripts/robstride_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robstride_teleop_node
)

install(DIRECTORY
  scripts/robstride_dynamics
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/src
  DESTINATION lib/${PROJECT_NAME}
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
