cmake_minimum_required(VERSION 3.8)
project(tf2_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

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
add_executable(tf2_broadcaster src/tf2_broadcaster.cpp)
ament_target_dependencies(tf2_broadcaster rclcpp tf2_ros)

add_executable(tf2_listener src/tf2_listener.cpp)
ament_target_dependencies(tf2_listener rclcpp tf2_ros)

install(TARGETS
  tf2_broadcaster
  tf2_listener
  DESTINATION lib/${PROJECT_NAME})