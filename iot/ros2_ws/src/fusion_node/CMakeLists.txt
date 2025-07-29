cmake_minimum_required(VERSION 3.8)
project(fusion_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# JSON (header-only)
find_package(nlohmann_json REQUIRED)

add_executable(fusion_node src/fusion_node.cpp)

# ROS2 패키지 의존성
ament_target_dependencies(fusion_node
  rclcpp
  std_msgs
)

# Header-only 라이브러리 링크
target_link_libraries(fusion_node
  nlohmann_json::nlohmann_json
)

install(TARGETS fusion_node
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
