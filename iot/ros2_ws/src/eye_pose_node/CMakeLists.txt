cmake_minimum_required(VERSION 3.8)
project(eye_pose_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# JSON (header-only)
find_package(nlohmann_json REQUIRED)

# Asio (header-only) via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(ASIO REQUIRED IMPORTED_TARGET asio)

# libcurl
find_package(CURL REQUIRED)

# websocketpp is header-only; cloned under websocketpp/
include_directories(
  ${PROJECT_SOURCE_DIR}/websocketpp
)

add_executable(eye_pose_node src/eye_pose_node.cpp)
ament_target_dependencies(eye_pose_node
  rclcpp
  std_msgs
)

# Include directories for Asio and websocketpp
target_include_directories(eye_pose_node PRIVATE
  ${ASIO_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/websocketpp
)

# Link header-only libraries (no PRIVATE keyword to match ament_target_dependencies signature)
target_link_libraries(eye_pose_node
  PkgConfig::ASIO
  nlohmann_json::nlohmann_json
  CURL::libcurl
)

install(TARGETS eye_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
