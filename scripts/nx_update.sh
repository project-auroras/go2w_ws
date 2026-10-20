#!/usr/bin/env bash
# On the NX: pull the latest code and build natively.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

git pull --ff-only
"$WS/scripts/install_deps.sh" ws

set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
