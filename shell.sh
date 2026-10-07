#!/usr/bin/env bash
set -Eeuo pipefail

readonly ROS_CONTAINER="${ROS_CONTAINER:-yahboom-ros-main}"
readonly ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
readonly ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/platform.sh"
readonly MODE="${1:-}"

if ! command -v docker >/dev/null 2>&1; then
  echo "Error: docker is not installed or is not in PATH." >&2
  exit 1
fi

ensure_ros_image

if [[ -n "$MODE" && "$MODE" != "--background" ]]; then
  echo "Usage: $0 [--background]" >&2
  exit 2
fi

if docker ps --format '{{.Names}}' | grep -Fxq "$ROS_CONTAINER"; then
  running_image="$(docker inspect --format '{{.Config.Image}}' "$ROS_CONTAINER")"
  running_arch="$(docker image inspect --format '{{.Architecture}}' "$running_image" 2>/dev/null || true)"
  expected_arch="${ROS_PLATFORM#linux/}"
  if [[ -n "$running_arch" && "$running_arch" != "$expected_arch" ]]; then
    echo "Error: $ROS_CONTAINER is $running_arch, but this host is configured for $expected_arch." >&2
    echo "Stop it before switching image: docker stop $ROS_CONTAINER" >&2
    exit 1
  fi
  if [[ "$MODE" == "--background" ]]; then
    echo "Yahboom ROS container is already running: $ROS_CONTAINER"
    exit 0
  fi

  echo "Entering the running Yahboom ROS container: $ROS_CONTAINER"
  exec docker exec -it \
    --env="ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
    --env="ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY" \
    "$ROS_CONTAINER" bash --rcfile /workspace/src/bringup/config/interactive.bash -i
fi

if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

docker_args=(
  --name "$ROS_CONTAINER"
  --privileged=true
  --net=host
  --env="DISPLAY=${DISPLAY:-:0}"
  --env="QT_X11_NO_MITSHM=1"
  --env="ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  --env="ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
  -v /tmp/.X11-unix:/tmp/.X11-unix
  --security-opt apparmor:unconfined
  -v "$SCRIPT_DIR/ros/src:/workspace/src"
  -v "$SCRIPT_DIR/maps:/workspace/maps"
)

docker_args+=(
  -v /dev/input:/dev/input
  -v /dev/video0:/dev/video0
)

docker_args+=(-v "$SCRIPT_DIR/ros/install/bringup:/workspace/install/bringup")
docker_args+=(-v "$SCRIPT_DIR/ros/install/robot_localization:/workspace/install/robot_localization:ro")

if [[ "$MODE" == "--background" ]]; then
  ros_id="$(docker run -d --rm --platform "$ROS_PLATFORM" "${docker_args[@]}" "$ROS_IMAGE" bash -lc 'exec tail -f /dev/null')"
  echo "Yahboom ROS container is running in the background: $ros_id"
  exit 0
fi

exec docker run -it --rm --platform "$ROS_PLATFORM" "${docker_args[@]}" "$ROS_IMAGE" \
  bash --rcfile /workspace/src/bringup/config/interactive.bash -i
