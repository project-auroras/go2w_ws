#!/usr/bin/env bash
# LiDAR range check against a wall (docs/validation/README.md, "Escala del LiDAR"). On the NX:
#   scripts/validation/wall_record.sh <tape_m>
# Robot still, facing a flat wall. <tape_m>: tape from the floor point under the LiDAR (plumb line
# from the front of the head) to the wall, perpendicular to it. Records /utlidar/cloud for 10 s in
# bags/<date>_<time>_wall/ with meta.yaml. Do it at two or more distances (e.g. 1.5 and 4.5 m):
# the difference cancels the unknown offset of the optical centre. PC: wall_range.py bags/*_wall
set -uo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TAPE="${1:-}"
[[ "$TAPE" =~ ^[0-9]+([.][0-9]+)?$ ]] || { sed -n 2,7p "$0"; exit 1; }
export GO2_IFACE="${GO2_IFACE:-enP8p1s0}"
set +u
# shellcheck disable=SC1091
source "$WS/scripts/env.sh"
set -u
OUT="$WS/bags/$(date +%F_%H-%M-%S)_wall"
echo "== recording 10 s, robot still, nobody between it and the wall"
timeout -s INT 12 ros2 bag record -o "$OUT" /utlidar/cloud /tf_static > /dev/null 2>&1
[ -f "$OUT/metadata.yaml" ] || { echo "!! no bag written (is /utlidar/cloud published?)" >&2; exit 1; }
cat > "$OUT/meta.yaml" <<EOM
test: wall
date: $(date -Iseconds)
host: $(hostname)
git: $(git -C "$WS" rev-parse --short HEAD) ($(git -C "$WS" rev-parse --abbrev-ref HEAD))
ground_truth:
  wall_tape_m: $TAPE
EOM
echo "== done: $OUT ($(du -sh "$OUT" | cut -f1))"
echo "   PC: scp -r unitree@192.168.123.18:$OUT ~/go2w_ws/bags/"
