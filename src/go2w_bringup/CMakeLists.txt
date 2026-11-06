cmake_minimum_required(VERSION 3.8)
project(go2w_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/lio_scan.py DESTINATION lib/${PROJECT_NAME})

ament_package()
