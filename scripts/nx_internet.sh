#!/usr/bin/env bash
# Run on the NX: reach the internet through the PC (see pc_share_internet.sh).
#   nx_internet.sh up [pc_ip] [iface]     defaults: 192.168.123.99 eth0
#   nx_internet.sh down [pc_ip] [iface]
# Only adds a default route and DNS: the robot network (DDS) is not touched.
# Not persistent: undo with 'down' or reboot.
set -euo pipefail

ACTION="${1:-}"
PC_IP="${2:-192.168.123.99}"
IFACE="${3:-eth0}"
DNS=(1.1.1.1 8.8.8.8)
SUDO=$([ "$(id -u)" -eq 0 ] && echo "" || echo "sudo")

if [[ ! "$ACTION" =~ ^(up|down)$ ]]; then
  echo "usage: $0 up|down [pc_ip] [iface]" >&2
  exit 1
fi
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "interface '$IFACE' not found (ip -br link)" >&2
  exit 1
fi

if [ "$ACTION" = "down" ]; then
  $SUDO ip route del default via "$PC_IP" dev "$IFACE" 2>/dev/null || true
  command -v resolvectl >/dev/null && $SUDO resolvectl revert "$IFACE" || true
  echo "[go2w] internet through $PC_IP removed"
  exit 0
fi

if ! ping -c 1 -W 2 "$PC_IP" >/dev/null; then
  echo "PC $PC_IP not reachable: run pc_share_internet.sh up on the PC first" >&2
  exit 1
fi

$SUDO ip route replace default via "$PC_IP" dev "$IFACE" metric 50
if command -v resolvectl >/dev/null; then
  $SUDO resolvectl dns "$IFACE" "${DNS[@]}"
  $SUDO resolvectl domain "$IFACE" "~."
else
  echo "resolvectl not found: set DNS by hand in /etc/resolv.conf" >&2
fi

if ping -c 1 -W 3 1.1.1.1 >/dev/null; then
  echo "[go2w] internet OK through $PC_IP"
else
  echo "no internet: check pc_share_internet.sh on the PC" >&2
  exit 1
fi
if ! getent hosts github.com >/dev/null; then
  echo "DNS not working" >&2
  exit 1
fi
# Without RTC the clock may be wrong and apt/git (TLS) fail; timesyncd fixes it once online
$SUDO timedatectl set-ntp true 2>/dev/null || true
echo "[go2w] DNS OK, date: $(date)"
