# Source this file (PC container and NX): ROS 2 Humble + CycloneDDS + workspace.
#   GO2_IFACE=enP8p1s0 source scripts/env.sh
GO2W_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
[ -f "$GO2W_WS/install/setup.bash" ] && source "$GO2W_WS/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -n "$GO2_IFACE" ]; then
  export CYCLONEDDS_URI="file://$GO2W_WS/config/cyclonedds.xml"
else
  unset CYCLONEDDS_URI
  echo "[go2w] GO2_IFACE not set: CycloneDDS will pick the interface itself" >&2
fi
