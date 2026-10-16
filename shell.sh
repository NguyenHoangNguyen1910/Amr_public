#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$ROOT/ros"
ROS_IMAGE="${ROS_IMAGE:-robot_nav/ros-humble:arm64}"
ROS_CONTAINER="${ROS_CONTAINER:-yahboom-ros-main}"
DOMAIN="${ROS_DOMAIN_ID:-20}"
LOCALHOST="${ROS_LOCALHOST_ONLY:-0}"
MAP_TOPIC="${MAP_TOPIC:-/rtabmap/map}"
USE_SLAM_TOOLBOX="${USE_SLAM_TOOLBOX:-false}"
ASTRA_IMAGE="${ASTRA_IMAGE:-husarion/astra:humble-nightly}"
ASTRA_CONTAINER="${ASTRA_CONTAINER:-astra-camera-slam}"
ASTRA_PARAMS_FILE="${ASTRA_PARAMS_FILE:-$ROOT/config/astra_params.yaml}"
RTABMAP_IMAGE="${RTABMAP_IMAGE:-introlab3it/rtabmap_ros:humble-latest}"
RTABMAP_CONTAINER="${RTABMAP_CONTAINER:-rtabmap-slam}"
RTABMAP_DATA_DIR="${RTABMAP_DATA_DIR:-$ROOT/rtabmap_data}"
RTABMAP_LAUNCH_FILE="${RTABMAP_LAUNCH_FILE:-$ROOT/config/rtabmap_rgbd.launch.py}"
RTABMAP_DATABASE_NAME="${RTABMAP_DATABASE_NAME:-recovery/warehouse.db.(test)-depth-$(date +%Y%m%d-%H%M%S).db}"
RTABMAP_ARGS="${RTABMAP_ARGS:---Rtabmap/DetectionRate 1.0 --Mem/IncrementalMemory true --Grid/Sensor true --Grid/RangeMax 5.0}"
RTABMAP_TOPIC_QUEUE_SIZE="${RTABMAP_TOPIC_QUEUE_SIZE:-10}"
RTABMAP_SYNC_QUEUE_SIZE="${RTABMAP_SYNC_QUEUE_SIZE:-10}"
USE_NAVIGATION="${USE_NAVIGATION:-false}"
AGENT_IMAGE="${AGENT_IMAGE:-microros/micro-ros-agent:humble}"
AGENT_CONTAINER="${AGENT_CONTAINER:-yahboom-micro-ros-agent}"
SERIAL_DEVICE="${SERIAL_DEVICE:-/dev/ttyUSB0}"
SERIAL_BAUDRATE="${SERIAL_BAUDRATE:-921600}"
AUTO_RESET="${AUTO_RESET:-1}"

usage() { cat <<EOF
Usage: $0 {build|bootstrap|rtabmap|run|serve_web|save_map NAME|stop|--background}
Run without a command to enter the ROS container.
EOF
}
docker_ok() { command -v docker >/dev/null || { echo "Error: docker is required." >&2; exit 1; }; }
image_ok() { docker image inspect "$ROS_IMAGE" >/dev/null 2>&1 || { echo "Error: ARM64 image is missing: $ROS_IMAGE" >&2; exit 1; }; }
ensure_astra_camera() {
  [[ -f "$ASTRA_PARAMS_FILE" ]] || { echo "Error: Astra params file not found: $ASTRA_PARAMS_FILE" >&2; return 1; }
  if ! docker image inspect "$ASTRA_IMAGE" >/dev/null 2>&1; then
    echo "Pulling Astra image: $ASTRA_IMAGE"
    docker pull "$ASTRA_IMAGE"
  fi

  if docker inspect "$ASTRA_CONTAINER" >/dev/null 2>&1; then
    local current_image
    current_image="$(docker inspect --format '{{.Config.Image}}' "$ASTRA_CONTAINER")"
    if [[ "$current_image" != "$ASTRA_IMAGE" ]]; then
      echo "Error: $ASTRA_CONTAINER uses image $current_image, expected $ASTRA_IMAGE." >&2
      echo "Remove it once if it was created with an old camera driver." >&2
      return 1
    fi
    local current_params_mount
    current_params_mount="$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/husarion_utils/astra_params.yaml"}}{{.Source}}{{end}}{{end}}' "$ASTRA_CONTAINER")"
    if [[ "$current_params_mount" != "$ASTRA_PARAMS_FILE" ]]; then
      echo "Recreating Astra camera with the optimized params file"
      docker rm -f "$ASTRA_CONTAINER" >/dev/null
    else
      if docker ps --format '{{.Names}}' | grep -Fxq "$ASTRA_CONTAINER"; then
        echo "Astra camera is already running: $ASTRA_CONTAINER"
      else
        docker start "$ASTRA_CONTAINER" >/dev/null
        echo "Started Astra camera: $ASTRA_CONTAINER"
      fi
      return 0
    fi
  fi

  echo "Creating Astra camera container: $ASTRA_CONTAINER"
  docker run -d \
    --name "$ASTRA_CONTAINER" \
    --restart unless-stopped \
    --network host \
    --ipc host \
    --privileged \
    --env="ROS_DOMAIN_ID=$DOMAIN" \
    --env="ROS_LOCALHOST_ONLY=$LOCALHOST" \
    --env=RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v /dev/bus/usb:/dev/bus/usb \
    -v "$ASTRA_PARAMS_FILE:/husarion_utils/astra_params.yaml:ro" \
    "$ASTRA_IMAGE" \
    bash -lc 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; exec ros2 launch astra_camera astra_mini.launch.py' \
    >/dev/null
  echo "Started Astra camera: $ASTRA_CONTAINER"
}
ensure_image() {
  if ! docker image inspect "$ROS_IMAGE" >/dev/null 2>&1; then
    echo "Building ARM64 image: $ROS_IMAGE"
    docker build -t "$ROS_IMAGE" "$ROOT"
  fi
}

ensure_micro_ros_agent() {
  docker_ok
  [[ -e "$SERIAL_DEVICE" ]] || {
    echo "Error: serial device not found: $SERIAL_DEVICE" >&2
    echo "Check the robot power, USB cable, and: ls /dev/ttyUSB*" >&2
    return 1
  }

  local running_agent
  running_agent="$(docker ps -q --filter "ancestor=$AGENT_IMAGE" | head -n 1)"
  if [[ -n "$running_agent" ]]; then
    echo "micro-ROS agent is already running: $running_agent"
    return 0
  fi

  echo "Starting micro-ROS agent on $SERIAL_DEVICE"
  docker run -d --rm \
    --name "$AGENT_CONTAINER" \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    --privileged \
    --net=host \
    --env="ROS_DOMAIN_ID=$DOMAIN" \
    --env="ROS_LOCALHOST_ONLY=$LOCALHOST" \
    "$AGENT_IMAGE" \
    serial --dev "$SERIAL_DEVICE" -b "$SERIAL_BAUDRATE" -v4 \
    >/dev/null

  if [[ "$AUTO_RESET" == 1 ]]; then
    sleep 1
    if ! python3 -c 'import serial,sys,time; p=serial.Serial(sys.argv[1],baudrate=int(sys.argv[2]),timeout=.2); p.dtr=False; p.rts=False; time.sleep(.1); p.dtr=True; p.rts=True; time.sleep(.2); p.close()' "$SERIAL_DEVICE" "$SERIAL_BAUDRATE"; then
      echo "Warning: automatic reset failed; press RESET on the micro-ROS control board." >&2
    fi
  fi
}

build() {
  docker_ok; ensure_image; test -f "$WS/src/robot_localization/package.xml"
  echo "Building ROS workspace: $WS"
  docker run --rm --cpus=1 --memory=2304m --memory-swap=2304m --pids-limit=256 -v "$WS:/workspace" "$ROS_IMAGE" bash -lc '
    set -Eeo pipefail; source /opt/ros/humble/setup.bash; set -u; cd /workspace
    export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1; ulimit -v 2400000
    nice -n 15 ionice -c 3 colcon --log-base log build --base-paths src --build-base build --install-base install --executor sequential --parallel-workers 1 --packages-select robot_localization bringup --event-handlers console_cohesion+ --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release "-DCMAKE_CXX_FLAGS_RELEASE=-O0 -DNDEBUG"'
  docker run --rm -v "$WS:/workspace" "$ROS_IMAGE" rm -f /workspace/install/bringup/share/bringup/config/navigation.yaml
  for f in \
    "$WS/install/robot_localization/lib/robot_localization/ekf_node" \
    "$WS/install/bringup/lib/bringup/depth_websocket.py" \
    "$WS/install/bringup/lib/bringup/cmd_vel_arbiter.py" \
    "$WS/install/bringup/lib/bringup/web_bridge.py" \
    "$WS/install/bringup/lib/bringup/navigation_node.py" \
    "$WS/install/bringup/lib/bringup/imu_covariance_adapter.py"; do test -x "$f"; done
  for f in "$WS/install/bringup/share/bringup/msg/ScanPoints.idl" "$WS/install/bringup/share/bringup/action/NavigateToPose.idl" "$WS/install/bringup/share/bringup/launch/web.launch.py"; do test -f "$f"; done
  test -f "$WS/install/bringup/local/lib/python3.10/dist-packages/navigation/ros_adapter.py" || test -f "$WS/install/bringup/local/lib/python3.12/dist-packages/navigation/ros_adapter.py"
  echo "Build completed."
}

ros() {
  local mode="${1:-interactive}"; docker_ok; image_ok
  if [[ "$mode" == run ]]; then
    ensure_micro_ros_agent
    ensure_astra_camera
  fi
  if docker ps --format '{{.Names}}' | grep -Fxq "$ROS_CONTAINER"; then
    [[ "$mode" == background ]] && { echo "ROS container is already running: $ROS_CONTAINER"; return; }
    [[ "$mode" == run ]] && exec docker exec -it \
      --env="ROS_DOMAIN_ID=$DOMAIN" --env="ROS_LOCALHOST_ONLY=$LOCALHOST" \
      --env="MAP_TOPIC=$MAP_TOPIC" --env="USE_SLAM_TOOLBOX=$USE_SLAM_TOOLBOX" \
      --env="USE_NAVIGATION=$USE_NAVIGATION" \
      "$ROS_CONTAINER" bash -lc \
      'source /workspace/src/bringup/config/interactive.bash; exec ros2 launch bringup web.launch.py map_topic:=${MAP_TOPIC} use_slam_toolbox:=${USE_SLAM_TOOLBOX} use_navigation:=${USE_NAVIGATION}'
    exec docker exec -it --env="ROS_DOMAIN_ID=$DOMAIN" --env="ROS_LOCALHOST_ONLY=$LOCALHOST" "$ROS_CONTAINER" bash --rcfile /workspace/src/bringup/config/interactive.bash -i
  fi
  [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1 && xhost +local:root >/dev/null 2>&1 || true
  local -a args=(--name "$ROS_CONTAINER" --privileged --net=host --ipc=host --env="DISPLAY=${DISPLAY:-:0}" --env=QT_X11_NO_MITSHM=1 --env="ROS_DOMAIN_ID=$DOMAIN" --env="ROS_LOCALHOST_ONLY=$LOCALHOST" --env="MAP_TOPIC=$MAP_TOPIC" --env="USE_SLAM_TOOLBOX=$USE_SLAM_TOOLBOX" --env="USE_NAVIGATION=$USE_NAVIGATION" -v /tmp/.X11-unix:/tmp/.X11-unix --security-opt apparmor:unconfined -v "$ROOT/ros/src:/workspace/src" -v "$ROOT/maps:/workspace/maps" -v /dev/input:/dev/input -v /dev/bus/usb:/dev/bus/usb -v "$ROOT/ros/install/bringup:/workspace/install/bringup" -v "$ROOT/ros/install/robot_localization:/workspace/install/robot_localization:ro")
  if [[ "$mode" == background ]]; then
    docker run -d --rm "${args[@]}" "$ROS_IMAGE" tail -f /dev/null
  elif [[ "$mode" == run ]]; then
    exec docker run -it --rm "${args[@]}" "$ROS_IMAGE" bash -lc 'source /workspace/src/bringup/config/interactive.bash; exec ros2 launch bringup web.launch.py map_topic:=${MAP_TOPIC} use_slam_toolbox:=${USE_SLAM_TOOLBOX} use_navigation:=${USE_NAVIGATION}'
  else
    exec docker run -it --rm "${args[@]}" "$ROS_IMAGE" bash --rcfile /workspace/src/bringup/config/interactive.bash -i
  fi
}

bootstrap() {
  docker_ok
  ensure_micro_ros_agent
  ensure_astra_camera
  ros background
  echo "View logs: docker logs -f $AGENT_CONTAINER"
}

rtabmap() {
  docker_ok
  ensure_micro_ros_agent
  ensure_astra_camera
  [[ -f "$RTABMAP_LAUNCH_FILE" ]] || { echo "Error: RTAB-Map launch file not found: $RTABMAP_LAUNCH_FILE" >&2; exit 1; }
  docker ps --format '{{.Names}}' | grep -Fxq "$ROS_CONTAINER" || {
    echo "Error: ROS container is not running: $ROS_CONTAINER" >&2
    exit 1
  }
  docker image inspect "$RTABMAP_IMAGE" >/dev/null 2>&1 || docker pull "$RTABMAP_IMAGE"
  mkdir -p "$RTABMAP_DATA_DIR"
  if docker inspect "$RTABMAP_CONTAINER" >/dev/null 2>&1; then
    if [[ "${FORCE_RECREATE:-0}" != 1 ]]; then
      if docker ps --format '{{.Names}}' | grep -Fxq "$RTABMAP_CONTAINER"; then
        echo "RTAB-Map is already running: $RTABMAP_CONTAINER"
      else
        echo "RTAB-Map container already exists: $RTABMAP_CONTAINER"
        echo "Use FORCE_RECREATE=1 only when intentionally replacing it."
      fi
      return
    fi
    if docker ps --format '{{.Names}}' | grep -Fxq "$RTABMAP_CONTAINER"; then
      docker stop "$RTABMAP_CONTAINER" >/dev/null
    fi
    docker rm "$RTABMAP_CONTAINER" >/dev/null
  fi
  docker run -d --name "$RTABMAP_CONTAINER" --restart unless-stopped \
    --network host --ipc host \
    --env="ROS_DOMAIN_ID=$DOMAIN" --env=ROS_LOCALHOST_ONLY=0 \
    --env=RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --env="RTABMAP_DATABASE_NAME=$RTABMAP_DATABASE_NAME" \
    --env="RTABMAP_ARGS=$RTABMAP_ARGS" \
    --env="RTABMAP_TOPIC_QUEUE_SIZE=$RTABMAP_TOPIC_QUEUE_SIZE" \
    --env="RTABMAP_SYNC_QUEUE_SIZE=$RTABMAP_SYNC_QUEUE_SIZE" \
    -v "$RTABMAP_DATA_DIR:/root/.ros" \
    -v "$RTABMAP_LAUNCH_FILE:/opt/amr/rtabmap_rgbd.launch.py:ro" \
    "$RTABMAP_IMAGE" \
    bash -lc '
      source /opt/ros/humble/setup.bash
      exec ros2 launch /opt/amr/rtabmap_rgbd.launch.py \
      "rtabmap_args:=${RTABMAP_ARGS}" \
      "database_path:=/root/.ros/${RTABMAP_DATABASE_NAME}" \
      topic_queue_size:=${RTABMAP_TOPIC_QUEUE_SIZE} \
      sync_queue_size:=${RTABMAP_SYNC_QUEUE_SIZE}
    ' >/dev/null
  echo "RTAB-Map is running: $RTABMAP_CONTAINER"
}

serve_web() {
  command -v python3 >/dev/null || { echo "Error: python3 is required." >&2; exit 1; }
  test -f "$ROOT/web/index.html" || { echo "Error: web/index.html was not found." >&2; exit 1; }
  echo "RosRobot Web UI: http://$(hostname -I | awk '{print $1}'):${WEB_PORT:-8080}/index.html"
  exec python3 -m http.server "${WEB_PORT:-8080}" --bind 0.0.0.0 --directory "$ROOT/web"
}

save_map() {
  local name="${1:-}"
  [[ $# -eq 1 && "$name" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || { echo "Usage: $0 save_map MAP_NAME" >&2; exit 2; }
  if [[ "${FORCE:-0}" != 1 ]]; then for ext in yaml pgm posegraph data; do [[ ! -e "$ROOT/maps/$name.$ext" ]] || { echo "Error: maps/$name.$ext already exists." >&2; exit 1; }; done; fi
  docker_ok; docker ps --format '{{.Names}}' | grep -Fxq "$ROS_CONTAINER" || { echo "Error: ROS container is not running: $ROS_CONTAINER" >&2; exit 1; }
  docker exec --env="MAP_BASE=/workspace/maps/$name" "$ROS_CONTAINER" bash -lc 'set -Eeuo pipefail; source /opt/ros/humble/setup.bash; ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: \"${MAP_BASE}\"}}"; ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: \"${MAP_BASE}\"}"; for e in yaml pgm posegraph data; do test -s "${MAP_BASE}.${e}"; done'
  echo "Saved map under maps/$name.*"
}

stop() {
  docker_ok
  if docker ps --format '{{.Names}}' | grep -Fxq "$ASTRA_CONTAINER"; then
    docker stop "$ASTRA_CONTAINER" >/dev/null && echo "Stopped $ASTRA_CONTAINER"
  fi
  for c in "$RTABMAP_CONTAINER" "$ROS_CONTAINER" "$AGENT_CONTAINER"; do
    if docker inspect "$c" >/dev/null 2>&1; then
      docker rm -f "$c" >/dev/null && echo "Stopped $c"
    fi
  done
}

cmd="${1:-shell}"; shift || true
case "$cmd" in
  build) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; build ;;
  bootstrap) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; bootstrap ;;
  rtabmap) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; rtabmap ;;
  run) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; ros run ;;
  serve_web) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; serve_web ;;
  save_map) save_map "$@" ;;
  stop) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; stop ;;
  --background) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; ros background ;;
  shell|interactive) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; ros interactive ;;
  -h|--help) usage ;;
  *) usage >&2; exit 2 ;;
esac
