cmake_minimum_required(VERSION 3.8)
project(go2w_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch urdf config rviz models maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
