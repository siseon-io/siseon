# CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(pairing_bridge_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(geometry_msgs REQUIRED)

# Paho MQTT
find_path(PAHO_MQTT_CPP_INCLUDE_DIR NAMES mqtt/async_client.h)
find_library(PAHO_MQTT_CPP_LIB         NAMES paho-mqttpp3)
find_library(PAHO_MQTT_C_LIB           NAMES paho-mqtt3as)
if(NOT PAHO_MQTT_CPP_INCLUDE_DIR OR NOT PAHO_MQTT_CPP_LIB OR NOT PAHO_MQTT_C_LIB)
  message(FATAL_ERROR
    "Paho MQTT C/C++ 라이브러리를 찾을 수 없습니다.")
endif()
include_directories(${PAHO_MQTT_CPP_INCLUDE_DIR})

# sdbus-c++
find_path(SDBUS_CPP_INCLUDE_DIR NAMES sdbus-c++.h PATH_SUFFIXES sdbus-c++)
find_library(SDBUS_CPP_LIB           NAMES sdbus-c++)
if(NOT SDBUS_CPP_INCLUDE_DIR OR NOT SDBUS_CPP_LIB)
  message(FATAL_ERROR
    "libsdbus-c++-dev 설치를 확인하세요.")
endif()
include_directories(${SDBUS_CPP_INCLUDE_DIR})

# nlohmann/json
find_package(nlohmann_json 3.2.0 REQUIRED)

# xml2cpp로 생성된 GATT 서비스·특성 어댑터
set(DBUS_INTERFACES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/dbus_interfaces/gatt_service.xml"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/dbus_interfaces/gatt_characteristic.xml"
)
set(GENERATED_HEADERS
    "${CMAKE_CURRENT_BINARY_DIR}/gatt_service_adaptor.h"
    "${CMAKE_CURRENT_BINARY_DIR}/gatt_characteristic_adaptor.h"
)
list(LENGTH DBUS_INTERFACES NUM_INTFS)
math(EXPR LAST_IDX "${NUM_INTFS} - 1")
foreach(i RANGE ${LAST_IDX})
  list(GET DBUS_INTERFACES ${i} IFACE_XML)
  list(GET GENERATED_HEADERS ${i} GEN_H)
  add_custom_command(
    OUTPUT ${GEN_H}
    COMMAND sdbus-c++-xml2cpp ${IFACE_XML} --adaptor=${GEN_H}
    DEPENDS ${IFACE_XML}
  )
endforeach()
add_custom_target(pairing_bridge_node_dbus_codegen DEPENDS ${GENERATED_HEADERS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(pairing_bridge_node src/pairing_bridge_node.cpp)
add_dependencies(pairing_bridge_node pairing_bridge_node_dbus_codegen)

ament_target_dependencies(pairing_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
)

target_link_libraries(pairing_bridge_node
  ${PAHO_MQTT_CPP_LIB}
  ${PAHO_MQTT_C_LIB}
  ${SDBUS_CPP_LIB}
  nlohmann_json::nlohmann_json
)

install(TARGETS pairing_bridge_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS hooks/local_setup.sh DESTINATION . RENAME local_setup.bash)

ament_package()
