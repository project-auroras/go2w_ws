#!/usr/bin/env bash
# Records one validation test on the NX (bringup already up: go2w.launch.py lio:=true).
#   scripts/validation/record.sh <test> [--no-raw] [--no-metrics]
# <test>: static | line | spin | loop | nav | free   (protocol: docs/validation/README.md)
# 1. preflight (topic rates, odom TF)  2. go2w_metrics on the NX (/vitals/status, /jetson/stats)
# 3. rosbag in bags/<date>_<time>_<test>/ until Ctrl+C  4. asks for the measured ground truth
#    (tape) and writes meta.yaml next to the bag. Analyse: ros2 run go2w_metrics validate <bag>
# --no-raw: without /utlidar/cloud and the IMUs (smaller bag, but it cannot be replayed).
set -uo pipefail
set -m   # background jobs in their own process group: SIGINT reaches the whole launch

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TEST="${1:-}"
case "$TEST" in
  static|line|spin|loop|nav|free) shift ;;
  -h|--help|"") sed -n 2,9p "$0"; exit 0 ;;
  *) echo "unknown test '$TEST' (static|line|spin|loop|nav|free)" >&2; exit 1 ;;
esac
RAW=true
METRICS=true
for a in "$@"; do
  case "$a" in
    --no-raw) RAW=false ;;
    --no-metrics) METRICS=false ;;
    *) echo "unknown option: $a" >&2; exit 1 ;;
  esac
done

export GO2_IFACE="${GO2_IFACE:-enP8p1s0}"
set +u
# shellcheck disable=SC1091
source "$WS/scripts/env.sh"
set -u

TOPICS=(/odom /Odometry /scan /cmd_vel /tf /tf_static /rosout
        /vitals/status /jetson/stats
        /goal_pose /easynav_control /easynav_manager_info /easynav_navstate
        /planner_node/astar/path)
$RAW && TOPICS+=(/utlidar/cloud /utlidar/imu /lidar_imu)

NAME="$(date +%F_%H-%M-%S)_$TEST"
OUT="$WS/bags/$NAME"
mkdir -p "$WS/bags"
METRICS_PID=""

stop_group() {
  [ -n "$1" ] || return
  kill -INT -- "-$1" 2>/dev/null || return
  for _ in $(seq 50); do kill -0 "$1" 2>/dev/null || return; sleep 0.1; done
  kill -KILL -- "-$1" 2>/dev/null
}
trap 'stop_group "$METRICS_PID"' EXIT

echo "== preflight"
if ! python3 "$WS/scripts/validation/preflight.py"; then
  read -rp "Preflight failed. Record anyway? [y/N] " ans
  [ "$ans" = y ] || exit 1
fi

if $METRICS; then
  if ros2 pkg prefix go2w_metrics >/dev/null 2>&1 && ros2 pkg prefix ros2_vitals >/dev/null 2>&1; then
    ros2 launch go2w_metrics metrics.launch.py jetson:=true < /dev/null > "$WS/bags/$NAME.metrics.log" 2>&1 &
    METRICS_PID=$!
    sleep 3
  else
    echo "!! go2w_metrics or ros2_vitals not built: no hardware data (scripts/nx_update.sh)" >&2
    read -rp "Go on without metrics? [y/N] " ans
    [ "$ans" = y ] || exit 1
  fi
fi

echo "== recording $OUT  (Ctrl+C to stop)"
case "$TEST" in
  static) echo "   Robot still, standing, nobody moving around it. At least 5 minutes." ;;
  line)   echo "   Mark start on the floor, drive straight ~10 m at constant speed, stop. Tape the distance." ;;
  spin)   echo "   Stand on a mark, 3 full turns in place (count them), stop facing the start direction." ;;
  loop)   echo "   Start on a floor mark, drive a closed loop, stop as close as possible to the mark." ;;
  nav)    echo "   Send goals with scripts/validation/send_goal.py (another terminal)." ;;
esac
T0=$(date +%s)
# own process group (set -m): Ctrl+C in this terminal stops ros2 bag cleanly via the trap
trap 'kill -INT -- "-$REC_PID" 2>/dev/null' INT
# stdin from /dev/null: with the terminal, its keyboard handler gets the background job stopped (SIGTTOU)
ros2 bag record -o "$OUT" "${TOPICS[@]}" < /dev/null > "$WS/bags/$NAME.record.log" 2>&1 &
REC_PID=$!
wait "$REC_PID"
wait "$REC_PID" 2>/dev/null
trap - INT
DUR=$(( $(date +%s) - T0 ))
stop_group "$METRICS_PID"; METRICS_PID=""
[ -f "$OUT/metadata.yaml" ] || { echo "!! no bag written, see $WS/bags/$NAME.record.log" >&2; exit 1; }

echo "== ground truth (empty = unknown)"
GT=""
case "$TEST" in
  line) read -rp "Distance start mark -> robot measured with tape [m]: " v; GT="distance_m: ${v:-null}" ;;
  spin) read -rp "Full turns done: " v; GT="turns: ${v:-null}" ;;
  loop) read -rp "Distance robot -> start mark at the end, tape [m]: " v; GT="return_error_m: ${v:-null}" ;;
esac
read -rp "Place / surface / notes: " NOTES
cat > "$OUT/meta.yaml" <<EOF
test: $TEST
date: $(date -Iseconds)
duration_s: $DUR
host: $(hostname)
git: $(git -C "$WS" rev-parse --short HEAD) ($(git -C "$WS" rev-parse --abbrev-ref HEAD))$(git -C "$WS" diff --quiet || echo ' +uncommitted')
raw_sensors: $RAW
ground_truth:
  ${GT:-none: null}
notes: "${NOTES//\"/\'}"
EOF
mv "$WS/bags/$NAME".*.log "$OUT/" 2>/dev/null
echo "== done: $OUT ($(du -sh "$OUT" | cut -f1))"
echo "   PC: scp -r unitree@192.168.123.18:$OUT ~/go2w_ws/bags/"
