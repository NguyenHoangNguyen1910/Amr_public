#!/usr/bin/env bash
set -Eeuo pipefail

readonly ROS_CONTAINER="${ROS_CONTAINER:-yahboom-ros-main}"
readonly MAP_NAME="${1:-}"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! "$MAP_NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "Usage: $0 MAP_NAME" >&2
  echo "MAP_NAME may contain letters, digits, dot, underscore and dash." >&2
  exit 2
fi

if [[ "${FORCE:-0}" != "1" ]]; then
  for extension in yaml pgm posegraph data; do
    if [[ -e "$SCRIPT_DIR/maps/$MAP_NAME.$extension" ]]; then
      echo "Error: maps/$MAP_NAME.$extension already exists." >&2
      echo "Choose another name, or explicitly overwrite with FORCE=1." >&2
      exit 1
    fi
  done
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$ROS_CONTAINER"; then
  echo "Error: ROS container is not running: $ROS_CONTAINER" >&2
  exit 1
fi

docker exec \
  --env="MAP_BASE=/workspace/maps/$MAP_NAME" \
  "$ROS_CONTAINER" bash -lc '
    set -Eeo pipefail
    source /opt/ros/humble/setup.bash
    set -u
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
      "{name: {data: \"${MAP_BASE}\"}}"
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
      "{filename: \"${MAP_BASE}\"}"
    test -s "${MAP_BASE}.yaml"
    test -s "${MAP_BASE}.pgm"
    test -s "${MAP_BASE}.posegraph"
    test -s "${MAP_BASE}.data"
  '

echo "Saved occupancy map and pose graph under maps/$MAP_NAME.*"
