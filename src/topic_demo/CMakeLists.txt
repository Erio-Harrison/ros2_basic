cmake_minimum_required(VERSION 3.8)
project(topic_demo)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the Apache Arrow installation path
set(CMAKE_PREFIX_PATH "/usr/local")

# Find package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces_demo REQUIRED)
find_package(Arrow REQUIRED)  # Finds Apache Arrow
find_package(Parquet REQUIRED)  # Find Parquet separately if needed

# If linking Parquet separately is needed (comment out if not)
# find_package(Parquet REQUIRED)
# include_directories(${PARQUET_INCLUDE_DIR})

# Include directories for both Arrow and Parquet
include_directories(${ARROW_INCLUDE_DIR})
include_directories(${PARQUET_INCLUDE_DIR})
include(FetchContent)

FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.10.5
)

FetchContent_MakeAvailable(json)


# Linter settings
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Declare executables
add_executable(pub src/pub.cpp)
add_executable(sub src/sub.cpp)

# Declare dependencies of executables
ament_target_dependencies(pub "rclcpp" "std_msgs" "interfaces_demo")
ament_target_dependencies(sub "rclcpp" "std_msgs" "interfaces_demo")

# Link libraries with executables
# Link Arrow (and Parquet if separate) with your executables
# Link both Arrow and Parquet with your executables
target_link_libraries(pub ${ARROW_SHARED_LIB} ${PARQUET_SHARED_LIB})
target_link_libraries(sub ${ARROW_SHARED_LIB} ${PARQUET_SHARED_LIB})
target_link_libraries(pub ${ARROW_SHARED_LIB})  # Link Arrow to pub
target_link_libraries(sub ${ARROW_SHARED_LIB})  # Link Arrow to sub
target_include_directories(pub PRIVATE ${json_SOURCE_DIR}/single_include)


# If Parquet is separate, uncomment the following lines


# Install the executables
install(TARGETS pub sub DESTINATION lib/${PROJECT_NAME})

# Package configuration
ament_package()
