# Source this file (PC container and NX): ROS 2 Humble + CycloneDDS + workspace.
#   GO2_IFACE=enP8p1s0 source scripts/env.sh
GO2W_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
[ -f "$GO2W_WS/install/setup.bash" ] && source "$GO2W_WS/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   GO2_PEERS="ip1 ip2"   optional: also discover these hosts by unicast (needed over WiFi,
#                         where multicast often does not get through). List every machine,
#                         including this one. GO2_MULTICAST=false: unicast only (laptop on WiFi;
#                         never on the NX, the robot itself is only found by multicast).
if [ -n "$GO2_IFACE" ]; then
  # The interface name is written into the config now: with
  # "GO2_IFACE=x source env.sh", bash drops GO2_IFACE as soon as the source ends,
  # so a ${GO2_IFACE} read later by CycloneDDS (config/cyclonedds.xml) would be empty.
  export GO2_IFACE GO2_PEERS GO2_MULTICAST
  _go2w_general="<Interfaces><NetworkInterface name=\"$GO2_IFACE\" priority=\"default\" multicast=\"default\"/></Interfaces>"
  _go2w_discovery=""
  if [ "$GO2_MULTICAST" = "false" ]; then
    _go2w_general="$_go2w_general<AllowMulticast>false</AllowMulticast>"
  fi
  if [ -n "$GO2_PEERS" ]; then
    _go2w_discovery="<Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>40</MaxAutoParticipantIndex><Peers>"
    for _go2w_peer in $GO2_PEERS; do
      _go2w_discovery="$_go2w_discovery<Peer address=\"$_go2w_peer\"/>"
    done
    _go2w_discovery="$_go2w_discovery</Peers></Discovery>"
  fi
  export CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General>$_go2w_general</General>$_go2w_discovery</Domain></CycloneDDS>"
  unset _go2w_general _go2w_discovery _go2w_peer
  echo "[go2w] CycloneDDS: $GO2_IFACE${GO2_PEERS:+, peers: $GO2_PEERS}${GO2_MULTICAST:+, multicast: $GO2_MULTICAST}" >&2
else
  unset CYCLONEDDS_URI
  echo "[go2w] GO2_IFACE not set: CycloneDDS will pick the interface itself" >&2
fi
