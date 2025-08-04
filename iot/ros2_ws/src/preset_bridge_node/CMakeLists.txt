cmake_minimum_required(VERSION 3.8)
project(preset_bridge_node)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# 필수 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 외부 라이브러리 (Paho MQTT)
find_path(PAHO_MQTT_CPP_INCLUDE_DIR NAMES mqtt/async_client.h)
find_library(PAHO_MQTT_CPP_LIB NAMES paho-mqttpp3)
find_library(PAHO_MQTT_C_LIB NAMES paho-mqtt3as)

if(NOT PAHO_MQTT_CPP_INCLUDE_DIR OR NOT PAHO_MQTT_CPP_LIB OR NOT PAHO_MQTT_C_LIB)
  message(FATAL_ERROR "Paho MQTT C++ library not found. Please install libpaho-mqttpp3-dev and libpaho-mqtt3as-dev.")
endif()

include_directories(${PAHO_MQTT_CPP_INCLUDE_DIR})

# 노드 빌드
add_executable(preset_bridge_node src/preset_bridge_node.cpp)

# 의존성 연결
ament_target_dependencies(preset_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Paho 라이브러리 연결
target_link_libraries(preset_bridge_node
  ${PAHO_MQTT_CPP_LIB}
  ${PAHO_MQTT_C_LIB}
)

# 설치 설정
install(TARGETS
  preset_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
