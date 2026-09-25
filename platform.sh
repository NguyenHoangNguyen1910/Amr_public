#!/usr/bin/env bash

# Shared platform/image selection for the workspace scripts.
# This file is sourced by build.sh, shell.sh, and bootstrap.sh.

readonly ROBOT_NAV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly ROBOT_NAV_MACHINE="$(uname -m)"

case "$ROBOT_NAV_MACHINE" in
  x86_64|amd64)
    readonly ROBOT_NAV_NATIVE_PLATFORM="linux/amd64"
    ;;
  aarch64|arm64)
    readonly ROBOT_NAV_NATIVE_PLATFORM="linux/arm64"
    ;;
  *)
    echo "Error: unsupported host architecture: $ROBOT_NAV_MACHINE" >&2
    exit 1
    ;;
esac

case "${ROS_PLATFORM:-$ROBOT_NAV_NATIVE_PLATFORM}" in
  amd64|x86_64|linux/amd64)
    readonly ROS_PLATFORM="linux/amd64"
    ;;
  arm64|aarch64|linux/arm64)
    readonly ROS_PLATFORM="linux/arm64"
    ;;
  *)
    echo "Error: ROS_PLATFORM must be amd64 or arm64." >&2
    exit 2
    ;;
esac

if [[ -n "${ROS_IMAGE:-}" ]]; then
  readonly ROS_IMAGE_IS_AUTOMATIC=0
else
  readonly ROS_IMAGE_IS_AUTOMATIC=1
  if [[ "$ROS_PLATFORM" == "linux/amd64" ]]; then
    readonly ROS_IMAGE="robot_nav/ros-humble:amd64"
  else
    readonly ROS_IMAGE="yahboomtechnology/ros-humble:4.0.3"
  fi
fi

ensure_ros_image() {
  if docker image inspect "$ROS_IMAGE" >/dev/null 2>&1; then
    return 0
  fi

  if [[ "$ROS_IMAGE_IS_AUTOMATIC" == "1" && "$ROS_PLATFORM" == "linux/amd64" ]]; then
    echo "Native AMD64 ROS image is missing; building $ROS_IMAGE."
    docker build \
      --platform "$ROS_PLATFORM" \
      --tag "$ROS_IMAGE" \
      --file "$ROBOT_NAV_ROOT/Dockerfile.amd64" \
      "$ROBOT_NAV_ROOT"
    return 0
  fi

  echo "ROS image $ROS_IMAGE is not available locally; Docker will pull it." >&2
}
