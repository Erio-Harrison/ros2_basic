cmake_minimum_required(VERSION 3.8)
project(example_demo)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

# Create executable
add_executable(alpaca_trading_node src/alpaca_trading_node.cpp)
target_include_directories(alpaca_trading_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(alpaca_trading_node PUBLIC c_std_99 cxx_std_17)  # Require C++17
ament_target_dependencies(
  alpaca_trading_node
  "rclcpp"
  "std_msgs"
)

# Link libraries
target_link_libraries(alpaca_trading_node
  CURL::libcurl
  nlohmann_json::nlohmann_json
)

# Install
install(TARGETS alpaca_trading_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()