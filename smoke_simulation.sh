#!/usr/bin/env bash
set -Eeuo pipefail

readonly ROS_CONTAINER="${ROS_CONTAINER:-yahboom-ros-main}"

if [[ $# -ne 0 ]]; then
  echo "Usage: $0" >&2
  exit 2
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$ROS_CONTAINER"; then
  echo "Error: $ROS_CONTAINER is not running." >&2
  echo "Start simulation first:" >&2
  echo "  ROBOT_BACKEND=simulation ./bootstrap.sh" >&2
  exit 1
fi

docker_exec_args=(-i)
if [[ -n "${SMOKE_MAP:-}" ]]; then
  docker_exec_args+=(--env "SMOKE_MAP=$SMOKE_MAP")
fi
if [[ -n "${SIM_INITIAL_X:-}" ]]; then
  docker_exec_args+=(--env "SIM_INITIAL_X=$SIM_INITIAL_X")
fi
if [[ -n "${SIM_INITIAL_Y:-}" ]]; then
  docker_exec_args+=(--env "SIM_INITIAL_Y=$SIM_INITIAL_Y")
fi
if [[ -n "${SIM_INITIAL_YAW:-}" ]]; then
  docker_exec_args+=(--env "SIM_INITIAL_YAW=$SIM_INITIAL_YAW")
fi
exec docker exec "${docker_exec_args[@]}" "$ROS_CONTAINER" bash -lc \
  'source /opt/ros/humble/setup.bash &&
   if [[ -f /root/imu_ws/install/imu_complementary_filter/share/imu_complementary_filter/package.bash ]]; then
     source /root/imu_ws/install/imu_complementary_filter/share/imu_complementary_filter/package.bash
   fi &&
   source /workspace/install/robot_localization/share/robot_localization/package.bash &&
   source /workspace/install/bringup/share/bringup/package.bash &&
   exec python3 -u /workspace/src/bringup/test/test_simulation_smoke.py'
