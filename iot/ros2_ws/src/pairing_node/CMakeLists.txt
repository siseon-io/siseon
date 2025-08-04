cmake_minimum_required(VERSION 3.8)
project(pairing_node)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# 필수 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── 외부 라이브러리 (Paho MQTT) ─────────────────
find_path(PAHO_MQTT_CPP_INCLUDE_DIR NAMES mqtt/async_client.h)
find_library(PAHO_MQTT_CPP_LIB         NAMES paho-mqttpp3)
find_library(PAHO_MQTT_C_LIB           NAMES paho-mqtt3as)

if(NOT PAHO_MQTT_CPP_INCLUDE_DIR OR NOT PAHO_MQTT_CPP_LIB OR NOT PAHO_MQTT_C_LIB)
  message(FATAL_ERROR
    "Paho MQTT C/C++ 라이브러리를 찾을 수 없습니다.\n"
    "libpaho-mqttpp3-dev, libpaho-mqtt3as-dev 설치를 확인하세요.")
endif()
include_directories(${PAHO_MQTT_CPP_INCLUDE_DIR})

# ── D-Bus (sdbus-c++ v1.4.0-2) ────────────────────
find_path(SDBUS_CPP_INCLUDE_DIR NAMES sdbus-c++.h PATH_SUFFIXES sdbus-c++)
find_library(SDBUS_CPP_LIB          NAMES sdbus-c++)
if(NOT SDBUS_CPP_INCLUDE_DIR OR NOT SDBUS_CPP_LIB)
  message(FATAL_ERROR
    "sdbus-c++ 개발 라이브러리를 찾을 수 없습니다.\n"
    "sudo apt install libsdbus-c++-dev 명령으로 설치하세요.")
endif()
include_directories(${SDBUS_CPP_INCLUDE_DIR})

# 노드 빌드
add_executable(pairing_node src/pairing_node.cpp)



# ROS2 의존성 연결
ament_target_dependencies(pairing_node
  rclcpp
  std_msgs
  geometry_msgs
)

# 외부 라이브러리 링크 순서 중요
target_link_libraries(pairing_node
  ${PAHO_MQTT_CPP_LIB}
  ${PAHO_MQTT_C_LIB}
  ${SDBUS_CPP_LIB}
)

# 설치 설정
install(TARGETS
  pairing_node
  DESTINATION lib/${PROJECT_NAME}
)

# ament 환경 훅(local_setup.sh) 설치
install(PROGRAMS
  hooks/local_setup.sh
  DESTINATION .
  RENAME local_setup.bash
)

ament_package()
