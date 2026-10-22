cmake_minimum_required(VERSION 3.8)
project(livox_ros_driver2)

# Only the messages of Livox-SDK/livox_ros_driver2 (21445540, MIT), which fast_lio builds
# against. The real driver needs Livox-SDK2 and a generated package.xml.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CustomPoint.msg
  msg/CustomMsg.msg
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
