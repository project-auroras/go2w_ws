cmake_minimum_required(VERSION 3.8)
project(go2w_bringup)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# C++ port of scripts/lio_scan.py (go2w.launch.py lio_scan_cpp:=true)
add_executable(lio_scan_cpp src/lio_scan.cpp)
target_compile_features(lio_scan_cpp PUBLIC cxx_std_17)
target_include_directories(lio_scan_cpp PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(lio_scan_cpp rclcpp nav_msgs sensor_msgs tf2 tf2_ros)
install(TARGETS lio_scan_cpp DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/lio_scan.py DESTINATION lib/${PROJECT_NAME})

ament_package()
