#!/usr/bin/env bash
# Shared by docker/Dockerfile and the NX so both environments stay the same.
#   install_deps.sh system   apt packages
#   install_deps.sh ws       import external repos + rosdep for src/
#   install_deps.sh all      both (default)
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUDO=$([ "$(id -u)" -eq 0 ] && echo "" || echo "sudo")

install_system() {
  $SUDO apt-get update
  $SUDO apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    nlohmann-json3-dev \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rclcpp-components \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-pcl-ros \
    ros-humble-rviz2 \
    ros-humble-plotjuggler-ros \
    ros-humble-slam-toolbox
  [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || $SUDO rosdep init
}

install_ws() {
  mkdir -p "$WS/src/external"
  vcs import --recursive "$WS/src/external" < "$WS/deps.repos"
  rosdep update --rosdistro humble
  $SUDO apt-get update
  # ros_gz keys: Fortress on Humble, conflicts with the Harmonic ros_gz of the PC image
  rosdep install --from-paths "$WS/src" --ignore-src --rosdistro humble -y -r \
    --skip-keys "ros_gz ros_gz_sim ros_gz_bridge gz_sim_vendor gz_plugin_vendor"
}

case "${1:-all}" in
  system) install_system ;;
  ws) install_ws ;;
  all) install_system; install_ws ;;
  *) echo "usage: $0 [system|ws|all]" >&2; exit 1 ;;
esac
