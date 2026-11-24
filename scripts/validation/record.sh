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
  line)   echo "   Other terminal: scripts/validation/drive_line.py 10 [--speed 1.0]. Hands off the remote." ;;
  spin)   echo "   Other terminal: scripts/validation/drive_line.py --spin 3 (or --spin -3). Hands off the remote." ;;
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
  line) while true; do
          echo "   Point under the LiDAR (front of the head) on the floor. Tape from it to the TWO"
          echo "   marks of the floor line closest to it (e.g. the 8 m and the 10 m marks)."
          read -rp "   1) first mark, position on the line [m, e.g. 8]: " xa
          read -rp "      tape robot point -> that mark [m]: " a
          read -rp "   2) second mark, position on the line [m, e.g. 10]: " xb
          read -rp "      tape robot point -> that mark [m]: " b
          read -rp "   3) robot LEFT or RIGHT of the line, looking forward [l/r]: " sd
          case "$sd" in [Rr]*|[Dd]*) sd=right ;; *) sd=left ;; esac
          RES=$(python3 - "$xa" "$a" "$xb" "$b" "$sd" <<'PY'
import math, sys
try:
    xa, a, xb, b = map(float, sys.argv[1:5])
    x = (a * a - b * b + xb * xb - xa * xa) / (2 * (xb - xa))
    y2 = a * a - (x - xa) ** 2
    if y2 < -0.02 ** 2:
        print('!! these distances do not fit (check the marks and the numbers)')
    else:
        y = math.sqrt(max(y2, 0)) * (-1 if sys.argv[5] == 'right' else 1)
        print(f'-> along the line {x:.3f} m, sideways {y:+.3f} m, straight distance {math.hypot(x, y):.3f} m')
except (ValueError, ZeroDivisionError):
    print('!! need four numbers and two different marks')
PY
)
          echo "   $RES"
          read -rp "   OK? [Y/n] " ok
          [[ "$ok" =~ ^[Nn] || "$RES" == '!!'* ]] || break
        done
        GT="mark_a_m: ${xa:-null}"$'\n'"  dist_a_m: ${a:-null}"$'\n'"  mark_b_m: ${xb:-null}"$'\n'"  dist_b_m: ${b:-null}"$'\n'"  side: $sd" ;;
  spin) read -rp "Full turns done (count them): " v
        echo "   Before the test you marked on the floor the point under the LiDAR (front) and under"
        echo "   the back of the body (rear). Now, how far is each point from its mark, sideways?"
        read -rp "   front point -> front mark, LEFT + / RIGHT - looking forward [m]: " f
        read -rp "   rear point  -> rear mark,  LEFT + / RIGHT - [m]: " r
        read -rp "   distance front mark -> rear mark [m]: " L
        GT="turns: ${v:-null}"$'\n'"  front_left_m: ${f:-null}"$'\n'"  rear_left_m: ${r:-null}"$'\n'"  front_rear_m: ${L:-null}" ;;
  loop) read -rp "Distance robot -> start mark at the end, tape [m]: " v; GT="return_error_m: ${v:-null}" ;;
esac
read -rp "Place / surface / notes: " NOTES
# which /scan node ran (CPU of the LIO stack depends on it)
PROCS=$(pgrep -af "lib/go2w_bringup/lio_scan")   # stored first: a grep in the pipe would match itself
case "$PROCS" in *lio_scan_cpp*) LIO_SCAN=lio_scan_cpp ;; *lio_scan.py*) LIO_SCAN=lio_scan.py ;; *) LIO_SCAN=none ;; esac
cat > "$OUT/meta.yaml" <<EOF
test: $TEST
date: $(date -Iseconds)
duration_s: $DUR
host: $(hostname)
git: $(git -C "$WS" rev-parse --short HEAD) ($(git -C "$WS" rev-parse --abbrev-ref HEAD))$(git -C "$WS" diff --quiet || echo ' +uncommitted')
raw_sensors: $RAW
lio_scan: $LIO_SCAN
numpy: $(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || echo none)
ground_truth:
  ${GT:-none: null}
notes: "${NOTES//\"/\'}"
EOF
mv "$WS/bags/$NAME".*.log "$OUT/" 2>/dev/null
echo "== done: $OUT ($(du -sh "$OUT" | cut -f1))"
echo "   PC: scp -r unitree@192.168.123.18:$OUT ~/go2w_ws/bags/"
