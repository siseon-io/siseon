cmake_minimum_required(VERSION 3.8)
project(arm_control_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# This creates message-related targets.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CmdPose.msg"
)

# We must use a different name for our executable to avoid conflict.
add_executable(arm_control_node_exec src/arm_control_node.cpp)

# Link dependencies from other packages.
ament_target_dependencies(arm_control_node_exec
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  dynamixel_sdk
)

# Link the message library created in this package.
# This also sets up the build dependency correctly.
target_link_libraries(arm_control_node_exec
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Install the executable.
install(TARGETS
  arm_control_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install config files if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# We remove the install rules for urdf and meshes as they don't exist
# # Install URDF files
# install(DIRECTORY
#   urdf
#   DESTINATION share/${PROJECT_NAME}/
# )
# 
# # Install mesh files
# install(DIRECTORY
#   meshes
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
