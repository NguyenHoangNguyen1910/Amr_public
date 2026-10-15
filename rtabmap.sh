#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly RTABMAP_CONTAINER="${RTABMAP_CONTAINER:-rtabmap-slam}"
readonly RTABMAP_IMAGE="${RTABMAP_IMAGE:-introlab3it/rtabmap_ros:humble-latest}"
readonly ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-20}"
readonly RTABMAP_DATA_DIR="${RTABMAP_DATA_DIR:-$SCRIPT_DIR/rtabmap_data}"
readonly RTABMAP_DATABASE_NAME="${RTABMAP_DATABASE_NAME:-astra_wheel_mapping.db}"
readonly RTABMAP_DATABASE="$RTABMAP_DATA_DIR/$RTABMAP_DATABASE_NAME"
readonly RTABMAP_DEPTH_TOPIC="${RTABMAP_DEPTH_TOPIC:-/camera/depth/image_raw}"
readonly RTABMAP_FRAME_ID="${RTABMAP_FRAME_ID:-base_link}"
readonly RTABMAP_VISUAL_ODOMETRY="${RTABMAP_VISUAL_ODOMETRY:-false}"
readonly RTABMAP_ODOM_TOPIC="${RTABMAP_ODOM_TOPIC:-/odom}"

if ! command -v docker >/dev/null 2>&1; then
  echo "Error: docker is not installed or is not in PATH." >&2
  exit 1
fi

mkdir -p "$RTABMAP_DATA_DIR"

if docker inspect "$RTABMAP_CONTAINER" >/dev/null 2>&1; then
  current_cmd="$(docker inspect --format '{{json .Config.Cmd}}' "$RTABMAP_CONTAINER")"
  desired_config=true
  for required_arg in \
    "depth_topic:=$RTABMAP_DEPTH_TOPIC" \
    "frame_id:=$RTABMAP_FRAME_ID" \
    "visual_odometry:=$RTABMAP_VISUAL_ODOMETRY" \
    "odom_topic:=$RTABMAP_ODOM_TOPIC" \
    "database_path:=/root/.ros/$RTABMAP_DATABASE_NAME"; do
    if [[ "$current_cmd" != *"$required_arg"* ]]; then
      desired_config=false
      break
    fi
  done

  if [[ "$desired_config" == true ]]; then
    if [[ -e "$RTABMAP_DATABASE" && ! -s "$RTABMAP_DATABASE" ]]; then
      echo "Removing the empty RTAB-Map database so RTAB-Map can initialize it."
      if docker ps --format '{{.Names}}' | grep -Fxq "$RTABMAP_CONTAINER"; then
        docker stop "$RTABMAP_CONTAINER" >/dev/null
      fi
      docker rm "$RTABMAP_CONTAINER" >/dev/null 2>&1 || true
      rm -f "$RTABMAP_DATABASE"
    else
      if ! docker ps --format '{{.Names}}' | grep -Fxq "$RTABMAP_CONTAINER"; then
        docker start "$RTABMAP_CONTAINER" >/dev/null
      fi
      echo "RTAB-Map is already configured for depth $RTABMAP_DEPTH_TOPIC and odom $RTABMAP_ODOM_TOPIC."
      exit 0
    fi
  fi

  # Copy while the old container still exists. docker cp also works while it
  # is running, and removing the container first would remove its writable
  # layer, including the SQLite database.
  current_database_name="$(sed -n 's#.*database_path:=/root/.ros/\([^ ]*\).*#\1#p' <<<"$current_cmd")"
  if [[ "$current_database_name" == "$RTABMAP_DATABASE_NAME" && ! -s "$RTABMAP_DATABASE" ]]; then
    rm -f "$RTABMAP_DATABASE"
    if docker cp "$RTABMAP_CONTAINER:/root/.ros/$RTABMAP_DATABASE_NAME" \
       "$RTABMAP_DATABASE" >/dev/null 2>&1; then
      echo "Backed up the existing RTAB-Map database to $RTABMAP_DATABASE"
    fi
  fi

  if docker ps --format '{{.Names}}' | grep -Fxq "$RTABMAP_CONTAINER"; then
    docker stop "$RTABMAP_CONTAINER" >/dev/null
  fi

  docker rm "$RTABMAP_CONTAINER" >/dev/null 2>&1 || true
fi

docker run -d \
  --name "$RTABMAP_CONTAINER" \
  --restart unless-stopped \
  --network host \
  --ipc host \
  -e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VALUE" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$RTABMAP_DATA_DIR:/root/.ros" \
  "$RTABMAP_IMAGE" \
  bash -lc "source /opt/ros/humble/setup.bash && ros2 launch rtabmap_launch rtabmap.launch.py \
    frame_id:=$RTABMAP_FRAME_ID \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=$RTABMAP_DEPTH_TOPIC \
    camera_info_topic:=/camera/color/camera_info \
    visual_odometry:=$RTABMAP_VISUAL_ODOMETRY \
    odom_topic:=$RTABMAP_ODOM_TOPIC \
    approx_sync:=true \
    approx_sync_max_interval:=0.1 \
    qos:=2 \
    wait_for_transform:=1.0 \
    odom_always_process_most_recent_frame:=false \
    odom_args:='--Vis/MaxFeatures 400' \
    args:='--Rtabmap/DetectionRate 1.0' \
    database_path:=/root/.ros/$RTABMAP_DATABASE_NAME \
    rtabmap_viz:=false rviz:=false" >/dev/null

echo "Started $RTABMAP_CONTAINER with depth input $RTABMAP_DEPTH_TOPIC."
echo "Database: $RTABMAP_DATABASE"
