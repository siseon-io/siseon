cmake_minimum_required(VERSION 3.8)
project(manual_bt_node)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# ament + ROS2 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(std_msgs     REQUIRED)

# ── D-Bus (sdbus-c++ v1.4.0-2) ────────────────────
find_path(SDBUS_CPP_INCLUDE_DIR
  NAMES sdbus-c++.h
  PATH_SUFFIXES sdbus-c++)
find_library(SDBUS_CPP_LIB
  NAMES sdbus-c++)
if(NOT SDBUS_CPP_INCLUDE_DIR OR NOT SDBUS_CPP_LIB)
  message(FATAL_ERROR
    "sdbus-c++ 개발 라이브러리를 찾을 수 없습니다.\n"
    "sudo apt install libsdbus-c++-dev 으로 설치해주세요.")
endif()
include_directories(${SDBUS_CPP_INCLUDE_DIR})

# 노드 빌드
add_executable(manual_bt_node src/manual_bt_node.cpp)

# ROS2 의존성 연결
ament_target_dependencies(manual_bt_node
  rclcpp
  std_msgs
)

# 외부 라이브러리 링크
target_link_libraries(manual_bt_node
  ${SDBUS_CPP_LIB}
)

# 설치 설정
install(TARGETS
  manual_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
