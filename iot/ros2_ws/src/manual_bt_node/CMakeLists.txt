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
find_package(geometry_msgs REQUIRED)

# sdbus-c++
find_path(SDBUS_CPP_INCLUDE_DIR NAMES sdbus-c++.h PATH_SUFFIXES sdbus-c++)
find_library(SDBUS_CPP_LIB           NAMES sdbus-c++)
if(NOT SDBUS_CPP_INCLUDE_DIR OR NOT SDBUS_CPP_LIB)
  message(FATAL_ERROR
    "libsdbus-c++-dev 설치를 확인하세요.")
endif()
include_directories(${SDBUS_CPP_INCLUDE_DIR})

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
add_custom_target(manual_bt_node_dbus_codegen DEPENDS ${GENERATED_HEADERS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


# 노드 빌드
add_executable(manual_bt_node src/manual_bt_node.cpp)
add_dependencies(manual_bt_node manual_bt_node_dbus_codegen)


# ROS2 의존성 연결
ament_target_dependencies(manual_bt_node
  rclcpp
  std_msgs
  geometry_msgs
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
