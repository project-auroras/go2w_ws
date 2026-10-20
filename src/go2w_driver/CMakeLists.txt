cmake_minimum_required(VERSION 3.8)
project(go2w_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(go2_interfaces REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  go2_interfaces
  unitree_api
  unitree_go
)

add_library(${PROJECT_NAME} SHARED
  src/go2w_driver.cpp
  src/sim_go2w.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "go2w_driver::Go2WDriver"
  EXECUTABLE go2w_driver_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "go2w_driver::SimGo2W"
  EXECUTABLE sim_go2w_node
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
