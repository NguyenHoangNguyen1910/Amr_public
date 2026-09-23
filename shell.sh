#!/usr/bin/env bash
set -Eeuo pipefail

readonly ROS_IMAGE="${ROS_IMAGE:-yahboomtechnology/ros-humble:4.0.3}"
readonly ROS_CONTAINER="${ROS_CONTAINER:-yahboom-ros-main}"
readonly ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
readonly ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
readonly REQUESTED_BACKEND="${ROBOT_BACKEND:-}"
readonly ROBOT_BACKEND="${ROBOT_BACKEND:-hardware}"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly MODE="${1:-}"

if ! command -v docker >/dev/null 2>&1; then
  echo "Error: docker is not installed or is not in PATH." >&2
  exit 1
fi

if [[ -n "$MODE" && "$MODE" != "--background" ]]; then
  echo "Usage: $0 [--background]" >&2
  exit 2
fi

if docker ps --format '{{.Names}}' | grep -Fxq "$ROS_CONTAINER"; then
  running_backend="$(docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' \
    "$ROS_CONTAINER" | sed -n 's/^ROBOT_BACKEND=//p' | tail -n 1)"
  running_backend="${running_backend:-hardware}"
  if [[ -n "$REQUESTED_BACKEND" && "$running_backend" != "$ROBOT_BACKEND" ]]; then
    echo "Error: $ROS_CONTAINER uses ROBOT_BACKEND=$running_backend, requested $ROBOT_BACKEND." >&2
    echo "Stop it before switching backend: docker stop $ROS_CONTAINER" >&2
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
    "$ROS_CONTAINER" bash -lc \
    'source /opt/ros/humble/setup.bash &&
     source /root/imu_ws/install/imu_complementary_filter/share/imu_complementary_filter/package.bash &&
     source /workspace/install/robot_localization/share/robot_localization/package.bash &&
     source /workspace/install/bringup/share/bringup/package.bash &&
     exec bash -i'
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
  --env="ROBOT_BACKEND=$ROBOT_BACKEND"
  -v /tmp/.X11-unix:/tmp/.X11-unix
  --security-opt apparmor:unconfined
  -v "$SCRIPT_DIR/ros/src:/workspace/src"
  -v "$SCRIPT_DIR/maps:/workspace/maps"
)

if [[ "$ROBOT_BACKEND" == "hardware" ]]; then
  docker_args+=(
    -v /dev/input:/dev/input
    -v /dev/video0:/dev/video0
  )
fi

docker_args+=(-v "$SCRIPT_DIR/ros/install/bringup:/workspace/install/bringup")
docker_args+=(-v "$SCRIPT_DIR/ros/install/robot_localization:/workspace/install/robot_localization:ro")

if [[ "$MODE" == "--background" ]]; then
  ros_id="$(docker run -d --rm "${docker_args[@]}" "$ROS_IMAGE" bash -lc 'exec tail -f /dev/null')"
  echo "Yahboom ROS container is running in the background: $ros_id"
  exit 0
fi

exec docker run -it --rm "${docker_args[@]}" "$ROS_IMAGE" bash -lc \
  'source /opt/ros/humble/setup.bash &&
   source /root/imu_ws/install/imu_complementary_filter/share/imu_complementary_filter/package.bash &&
   source /workspace/install/robot_localization/share/robot_localization/package.bash &&
   source /workspace/install/bringup/share/bringup/package.bash &&
   exec bash -i'
