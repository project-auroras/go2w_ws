#!/usr/bin/env bash
# Run on any Linux PC/laptop: share its internet (WiFi or other) with the robot
# network through an Ethernet cable, so the NX can reach the internet (apt, git...).
#   pc_share_internet.sh up [eth_iface] [wan_iface]
#   pc_share_internet.sh down [eth_iface] [wan_iface]
# Without arguments the interfaces are detected:
#   wan_iface: the one with the default route
#   eth_iface: the wired interface (not WiFi, not virtual) with the cable plugged in
# Not persistent: undo with 'down' or reboot.
set -euo pipefail

PC_IP="${PC_IP:-192.168.123.99/24}"   # PC address on the robot network, if it has none
ROBOT_NET="192.168.123.0/24"
NX_IP="${NX_IP:-192.168.123.18}"

ACTION="${1:-}"
ETH="${2:-}"
WAN="${3:-}"
SUDO=$([ "$(id -u)" -eq 0 ] && echo "" || echo "sudo")

die() { echo "$*" >&2; exit 1; }

[[ "$ACTION" =~ ^(up|down)$ ]] || die "usage: $0 up|down [eth_iface] [wan_iface]"
command -v iptables >/dev/null || die "iptables not found: sudo apt install iptables"

# Address of an interface on the robot network, if any (e.g. 192.168.123.99/24)
robot_addr() {
  ip -4 -o addr show dev "$1" 2>/dev/null | awk '{print $4}' | grep '^192\.168\.123\.' | head -1 || true
}

# Physical, non-wireless interfaces: Ethernet ports and USB adapters
wired_ifaces() {
  local path name
  for path in /sys/class/net/*; do
    name=$(basename "$path")
    [ -e "$path/device" ] && [ ! -e "$path/wireless" ] && [ ! -e "$path/phy80211" ] \
      && [ "$name" != "$WAN" ] && echo "$name"
  done
}

has_carrier() {
  [ "$(cat "/sys/class/net/$1/carrier" 2>/dev/null)" = "1" ]
}

detect_eth() {
  local candidates=() iface
  # An interface already on the robot network wins
  for iface in $(wired_ifaces); do
    [ -n "$(robot_addr "$iface")" ] && { echo "$iface"; return; }
  done
  for iface in $(wired_ifaces); do
    # carrier is only readable when the link is up
    $SUDO ip link set "$iface" up >/dev/null 2>&1 || true
    has_carrier "$iface" && candidates+=("$iface")
  done
  case ${#candidates[@]} in
    1) echo "${candidates[0]}" ;;
    0) die "no wired interface with a cable plugged in (ip -br link). Pass it: $0 $ACTION <eth_iface>" ;;
    *) die "several wired interfaces with cable: ${candidates[*]}. Pass it: $0 $ACTION <eth_iface>" ;;
  esac
}

if [ -z "$WAN" ]; then
  WAN=$(ip route show default | awk '{print $5; exit}')
  [ -n "$WAN" ] || die "no default route: this PC has no internet"
fi
[ -n "$ETH" ] || ETH=$(detect_eth)

for iface in "$ETH" "$WAN"; do
  ip link show "$iface" >/dev/null 2>&1 || die "interface '$iface' not found (ip -br link)"
done
[ "$ETH" != "$WAN" ] || die "the robot cable ($ETH) is also the internet interface: use WiFi or another port for internet"

# FORWARD rules go first: Docker, if installed, sets the FORWARD policy to DROP
RULES=(
  "-t nat POSTROUTING -s $ROBOT_NET -o $WAN -j MASQUERADE"
  "FORWARD -i $ETH -o $WAN -s $ROBOT_NET -j ACCEPT"
  "FORWARD -i $WAN -o $ETH -d $ROBOT_NET -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT"
)

rule() {  # rule <-I|-D> "<chain and match>"
  local op=$1 table=() spec
  read -ra spec <<< "$2"
  if [ "${spec[0]}" = "-t" ]; then
    table=("${spec[@]:0:2}")
    spec=("${spec[@]:2}")
  fi
  local exists=0
  $SUDO iptables "${table[@]}" -C "${spec[@]}" 2>/dev/null && exists=1
  if [ "$op" = "-I" ] && [ $exists -eq 0 ]; then
    $SUDO iptables "${table[@]}" -I "${spec[@]}"
  elif [ "$op" = "-D" ] && [ $exists -eq 1 ]; then
    $SUDO iptables "${table[@]}" -D "${spec[@]}"
  fi
}

if [ "$ACTION" = "up" ]; then
  $SUDO ip link set "$ETH" up
  ADDR=$(robot_addr "$ETH")
  if [ -z "$ADDR" ]; then
    $SUDO ip addr add "$PC_IP" dev "$ETH"
    ADDR=$PC_IP
  fi
  $SUDO sysctl -q -w net.ipv4.ip_forward=1
  for r in "${RULES[@]}"; do rule -I "$r"; done

  echo "[go2w] sharing $WAN -> $ETH ($ADDR)"
  if ping -c 1 -W 2 "$NX_IP" >/dev/null 2>&1; then
    echo "[go2w] NX reachable at $NX_IP"
  else
    echo "[go2w] NX not answering at $NX_IP (other IP? set NX_IP=...)"
  fi
  echo "[go2w] on the NX: ./nx_internet.sh up ${ADDR%/*}"
else
  for r in "${RULES[@]}"; do rule -D "$r"; done
  $SUDO ip addr del "$PC_IP" dev "$ETH" 2>/dev/null || true
  echo "[go2w] stopped sharing $WAN -> $ETH (ip_forward left as is)"
fi
