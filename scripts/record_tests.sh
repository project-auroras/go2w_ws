#!/usr/bin/env bash
# Laptop side of the test recordings (run inside the container, robot + NX already up):
# external RealSense + YOLO (people_live.launch.py), checks the topics and records one
# rosbag per test in /bags (~/rosbags on the host), with the name you type.
#   scripts/record_tests.sh                        # YOLO only, the robot does not move
#   scripts/record_tests.sh --follower             # also person_follower (the robot MOVES)
#   scripts/record_tests.sh --follower --max-linear 0.4 --max-angular 0.8
# Between tests: 'v 0.4' (or 'v 0.4 0.8') restarts the follower with those speeds.
# Also launches the laptop hardware metrics (go2w_metrics). On the NX, besides bringup and
# camera: ros2 launch go2w_metrics metrics.launch.py jetson:=true
# CSVs of a bag: ros2 run go2w_metrics bag_to_csv /bags/<name> [--clock-offset S]
# Logs of the launches: /bags/logs/<session>/. Ctrl+C or 'q' stops everything.
set -uo pipefail
set -m   # every background job in its own process group: SIGINT reaches it (clean stop)

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FOLLOWER=false
MAX_LINEAR=0.6     # m/s, default of people_live.launch.py
MAX_ANGULAR=1.2    # rad/s
while [ $# -gt 0 ]; do
  case "$1" in
    --follower) FOLLOWER=true ;;
    --max-linear) MAX_LINEAR="$2"; shift ;;
    --max-angular) MAX_ANGULAR="$2"; shift ;;
    -h|--help) sed -n 2,13p "$0"; exit 0 ;;
    *) echo "unknown option: $1 (--help)" >&2; exit 1 ;;
  esac
  shift
done

export GO2_IFACE="${GO2_IFACE:-wlo1}"
# ROS setup files use unset variables: no -u while sourcing them
set +u
# shellcheck disable=SC1091
source "$WS/scripts/env.sh"
set -u

EXT=/external/cam/color/image_raw/compressed
NX=/camera/camera/color/image_raw/compressed
TOPICS=("$EXT" "$NX" /people /yolo/cam0/detections_3d /cmd_vel /odom /scan /tf /tf_static /rosout
        /vitals/status /jetson/stats)   # hardware of laptop + NX (go2w_metrics), CSV with bag_to_csv

SESSION=$(date +%F_%H-%M-%S)
LOGS=/bags/logs/$SESSION
mkdir -p "$LOGS"
CAM_PID=""
METRICS_PID=""
YOLO_PID=""
REC_PID=""

stop_group() {   # SIGINT to the process group, SIGKILL if it is still there after 5 s
  [ -n "$1" ] || return
  kill -INT -- "-$1" 2>/dev/null || return
  for _ in $(seq 50); do kill -0 "$1" 2>/dev/null || return; sleep 0.1; done
  kill -KILL -- "-$1" 2>/dev/null
}

cleanup() {
  trap - INT TERM EXIT
  echo
  [ -n "$REC_PID" ] && { echo "[rec] stopping the recording..."; stop_group "$REC_PID"; }
  echo "[rec] stopping camera and YOLO..."
  stop_group "$YOLO_PID"
  stop_group "$CAM_PID"
  stop_group "$METRICS_PID"
  echo "[rec] done. Bags in /bags (~/rosbags on the host), logs in $LOGS"
  exit 0
}
trap cleanup INT TERM EXIT

start_yolo() {
  echo "[rec] YOLO, follower:=$FOLLOWER max_linear:=$MAX_LINEAR max_angular:=$MAX_ANGULAR" \
       "($LOGS/yolo.log)"
  if [ "$FOLLOWER" = true ]; then
    echo "[rec] !!! the robot WILL MOVE: clear area and someone with the remote !!!"
  fi
  ros2 launch go2w_perception people_live.launch.py follower:=$FOLLOWER \
    max_linear:=$MAX_LINEAR max_angular:=$MAX_ANGULAR \
    < /dev/null >> "$LOGS/yolo.log" 2>&1 &
  YOLO_PID=$!
}

echo "[rec] external camera ($LOGS/camera.log)"
ros2 launch realsense2_camera rs_launch.py camera_namespace:=external camera_name:=cam \
  enable_depth:=false rgb_camera.color_profile:=640x480x30 initial_reset:=true \
  < /dev/null > "$LOGS/camera.log" 2>&1 &
CAM_PID=$!

echo "[rec] hardware metrics of the laptop -> /vitals/status ($LOGS/metrics.log)"
ros2 launch go2w_metrics metrics.launch.py < /dev/null > "$LOGS/metrics.log" 2>&1 &
METRICS_PID=$!
start_yolo

echo "[rec] waiting 15 s for everything to start..."
sleep 15
ros2 daemon stop >/dev/null 2>&1

check() {
  local missing=0
  for t in "$EXT" "$NX" /people /odom /scan /vitals/status /jetson/stats; do
    if timeout 5 ros2 topic echo --once --no-arr "$t" >/dev/null 2>&1; then
      echo "  OK  $t"
    else
      echo "  --  $t  (nothing in 5 s)"; missing=1
    fi
  done
  return $missing
}

echo "[rec] checking topics:"
if ! check; then
  echo "[rec] some topics are missing. Check: NX launches running (/jetson/stats: metrics.launch.py"
  echo "      jetson:=true on the NX), WiFi on the robot network, '[go2w] CycloneDDS: wlo1' above,"
  echo "      and the logs in $LOGS"
fi

N=0
while true; do
  echo
  echo "[rec] follower:=$FOLLOWER, max_linear $MAX_LINEAR m/s, max_angular $MAX_ANGULAR rad/s"
  read -r -p "[rec] bag name (Enter = automatic, v LIN [ANG] = speed, c = check, q = quit): " NAME
  case "$NAME" in
    q) break ;;
    c) check; continue ;;
    v\ *)
      read -r _ lin ang <<< "$NAME"
      if ! [[ "$lin" =~ ^[0-9]*\.?[0-9]+$ ]] || { [ -n "${ang:-}" ] && ! [[ "$ang" =~ ^[0-9]*\.?[0-9]+$ ]]; }; then
        echo "[rec] usage: v 0.4   or   v 0.4 0.8   (m/s, rad/s)"; continue
      fi
      MAX_LINEAR=$lin
      MAX_ANGULAR=${ang:-$MAX_ANGULAR}
      echo "[rec] restarting YOLO + follower with the new speeds..."
      stop_group "$YOLO_PID"
      start_yolo
      sleep 15
      check
      continue ;;
  esac
  N=$((N + 1))
  NAME=${NAME// /_}
  NAME=${NAME:-$(date +%F_%H-%M-%S)_test_$N}
  BAG=/bags/$NAME
  if [ -e "$BAG" ]; then
    echo "[rec] $BAG already exists: choose another name"; N=$((N - 1)); continue
  fi
  ros2 bag record -o "$BAG" "${TOPICS[@]}" < /dev/null > "$LOGS/rec_$N.log" 2>&1 &
  REC_PID=$!
  sleep 2
  echo "[rec] RECORDING $BAG"
  echo "      1) claps in front of the robot (both cameras)  2) test  3) claps again"
  read -r -p "[rec] Enter to stop the recording... " _
  stop_group "$REC_PID"
  REC_PID=""
  if [ -f "$BAG/metadata.yaml" ]; then
    echo "[rec] saved $BAG:"
    ros2 bag info "$BAG" 2>/dev/null | grep -E "Duration|Topic: ($EXT|$NX|/people) " \
      | sed -E 's/ +\| Serialization.*//; s/^ */  /'
  else
    echo "[rec] $BAG was not saved properly: see $LOGS/rec_$N.log"
  fi
done
