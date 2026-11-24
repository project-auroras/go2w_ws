#!/usr/bin/env bash
# L1 range drift while it warms up (docs/validation/README.md, "Escala del LiDAR"). On the NX,
# right after switching the robot on from COLD (off >= 1 h):
#   scripts/validation/wall_warmup.sh <tape_m> [interval_min=5] [total_min=40]
# Robot still in front of a flat wall the whole time, same posture, nobody touching it. Calls
# wall_record.sh every <interval_min> and writes the minutes since the first record and the NX
# uptime in each meta.yaml. PC: python3 scripts/validation/wall_range.py bags/*_wall
set -uo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TAPE="${1:-}"; EVERY="${2:-5}"; TOTAL="${3:-40}"
[[ "$TAPE" =~ ^[0-9]+([.][0-9]+)?$ ]] || { sed -n 2,8p "$0"; exit 1; }
UP=$(cut -d' ' -f1 /proc/uptime)
echo "== NX up for $(( ${UP%.*} / 60 )) min. Cold start expected (robot off >= 1 h)."
T0=$(date +%s)
SERIES=$(date +%F_%H-%M)
i=0
while :; do
  MIN=$(( ($(date +%s) - T0) / 60 ))
  echo "== record $((i + 1)) at minute $MIN"
  WALL_EXTRA_META="warmup_series: $SERIES"$'\n'"warmup_min: $MIN" "$WS/scripts/validation/wall_record.sh" "$TAPE" \
    || echo "!! record failed, going on"
  i=$((i + 1))
  NEXT=$(( T0 + i * EVERY * 60 ))
  [ $(( NEXT - T0 )) -gt $(( TOTAL * 60 )) ] && break
  WAIT=$(( NEXT - $(date +%s) ))
  [ "$WAIT" -gt 0 ] && { echo "   next in $((WAIT / 60)) min $((WAIT % 60)) s (Ctrl+C to stop)"; sleep "$WAIT"; }
done
echo "== done, $i records. PC: scp -r unitree@192.168.123.18:$WS/bags/*_wall ~/go2w_ws/bags/"
