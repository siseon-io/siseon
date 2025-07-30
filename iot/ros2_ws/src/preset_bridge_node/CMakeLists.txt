cmake_minimum_required(VERSION 3.8)
project(preset_bridge_node)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 노드 빌드
add_executable(preset_bridge_node src/preset_bridge_node.cpp)

# 의존성 연결
ament_target_dependencies(preset_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
)

# 설치 설정
install(TARGETS
  preset_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
