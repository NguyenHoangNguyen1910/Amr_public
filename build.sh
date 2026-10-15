#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/platform.sh"
readonly WORKSPACE="$SCRIPT_DIR/ros"
readonly PLATFORM_MARKER="$WORKSPACE/.robot_nav_platform"

if ! command -v docker >/dev/null 2>&1; then
  echo "Error: docker is not installed or is not in PATH." >&2
  exit 1
fi

ensure_ros_image

if [[ -d "$WORKSPACE/build" || -d "$WORKSPACE/install" || -d "$WORKSPACE/log" ]]; then
  if [[ ! -f "$PLATFORM_MARKER" || "$(<"$PLATFORM_MARKER")" != "$ROS_PLATFORM" ]]; then
    echo "Generated workspace files were built for another platform; cleaning them."
    docker run --rm \
      --platform "$ROS_PLATFORM" \
      --volume "$WORKSPACE:/workspace" \
      "$ROS_IMAGE" \
      rm -rf /workspace/build /workspace/install /workspace/log
  fi
fi

test -f "$WORKSPACE/src/robot_localization/package.xml"

echo "Building the current source workspace with the upstream robot_localization code."
echo "No EKF-only CMake option and no source patch are used."
echo "Build output: $WORKSPACE/build"
echo "Install output: $WORKSPACE/install"

docker run --rm \
  --platform "$ROS_PLATFORM" \
  --cpus=1.0 \
  --memory=2304m \
  --memory-swap=2304m \
  --pids-limit=256 \
  --volume "$WORKSPACE:/workspace" \
  "$ROS_IMAGE" \
  bash -lc '
    set -Eeo pipefail
    source /opt/ros/humble/setup.bash
    set -u
    cd /workspace
    export CMAKE_BUILD_PARALLEL_LEVEL=1
    export MAKEFLAGS=-j1
    ulimit -v 2400000

    exec nice -n 15 ionice -c 3 \
      colcon --log-base log build \
        --base-paths src \
        --build-base build \
        --install-base install \
        --executor sequential \
        --parallel-workers 1 \
        --packages-select \
          robot_localization \
          bringup \
          --event-handlers console_cohesion+ \
        --cmake-args \
          -DBUILD_TESTING=OFF \
          -DCMAKE_BUILD_TYPE=Release \
          "-DCMAKE_CXX_FLAGS_RELEASE=-O0 -DNDEBUG"
  '

docker run --rm \
  --platform "$ROS_PLATFORM" \
  --volume "$WORKSPACE:/workspace" \
  "$ROS_IMAGE" \
  rm -f /workspace/install/bringup/share/bringup/config/navigation.yaml

test -x "$WORKSPACE/install/robot_localization/lib/robot_localization/ekf_node"
test -f "$WORKSPACE/install/bringup/share/bringup/msg/ScanPoints.idl"
test -f "$WORKSPACE/install/bringup/share/bringup/action/NavigateToPose.idl"
test -f "$WORKSPACE/install/bringup/share/bringup/launch/web.launch.py"
test -x "$WORKSPACE/install/bringup/lib/bringup/camera_mjpeg.py"
test -f "$WORKSPACE/install/bringup/lib/bringup/camera_stream.py"
test -x "$WORKSPACE/install/bringup/lib/bringup/cmd_vel_arbiter.py"
test -x "$WORKSPACE/install/bringup/lib/bringup/web_bridge.py"
test -x "$WORKSPACE/install/bringup/lib/bringup/navigation_node.py"
test -x "$WORKSPACE/install/bringup/lib/bringup/simulation_adapter.py"
test -f "$WORKSPACE/install/bringup/share/bringup/maps/amr_world.json"
test -f "$WORKSPACE/install/bringup/local/lib/python3.10/dist-packages/navigation/ros_adapter.py" || \
  test -f "$WORKSPACE/install/bringup/local/lib/python3.12/dist-packages/navigation/ros_adapter.py"
test -f "$WORKSPACE/install/bringup/local/lib/python3.10/dist-packages/simulation/core.py" || \
test -f "$WORKSPACE/install/bringup/local/lib/python3.12/dist-packages/simulation/core.py"

printf '%s\n' "$ROS_PLATFORM" > "$PLATFORM_MARKER"

echo "Build completed and required runtime files were found."
