#!/usr/bin/env bash
# Build/test in containers with no network or robot device mounts.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
VERIFY_DIR="${VERIFY_DIR:-$(mktemp -d /tmp/amr-verify-XXXXXXXX)}"
ROS_IMAGE="${ROS_IMAGE:-robot_nav/ros-humble:arm64}"
RTABMAP_IMAGE="${RTABMAP_IMAGE:-introlab3it/rtabmap_ros:humble-latest}"
mkdir -p "$VERIFY_DIR"
test -x "$ROOT/ros/install/robot_localization/lib/robot_localization/ekf_node"
bash -n "$ROOT/shell.sh"
git -C "$ROOT" diff --check
echo "Verification artifacts: $VERIFY_DIR"
docker run --rm --network none --cpus=2 \
  -e ROS_DOMAIN_ID=83 -e ROS_LOCALHOST_ONLY=1 \
  -v "$ROOT/ros/src:/src:ro" -v "$ROOT/tools:/tools:ro" \
  -v "$ROOT/ros/install/robot_localization:/existing_ekf:ro" \
  -v "$VERIFY_DIR:/verify" "$ROS_IMAGE" bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /root/imu_ws/install/local_setup.bash
    mkdir -p /verify/robot_localization
    cp -a /existing_ekf/. /verify/robot_localization/
    cp /src/robot_localization/params/ekf.yaml /verify/robot_localization/share/robot_localization/params/ekf.yaml
    COLCON_CURRENT_PREFIX=/verify/robot_localization source /verify/robot_localization/share/robot_localization/package.bash
    cd /verify
    colcon --log-base /verify/log build --base-paths /src/bringup \
      --build-base /verify/build --install-base /verify/install --executor sequential \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release > /verify/build.log 2>&1
    source /verify/install/local_setup.bash
    export PYTHONPATH=/src/bringup/scripts:$PYTHONPATH
    export LD_LIBRARY_PATH=/verify/robot_localization/lib:$LD_LIBRARY_PATH
    export AMR_EKF_EXECUTABLE=/verify/robot_localization/lib/robot_localization/ekf_node
    python3 -m unittest discover -s /src/bringup/test -v > /verify/unit-tests.log 2>&1 || {
      tail -100 /verify/unit-tests.log; exit 1;
    }
    tail -5 /verify/unit-tests.log
    ros2 launch bringup web.launch.py --show-args > /verify/bringup-launch-args.log
    python3 /tools/verify_bringup.py
  '
for mode in false true; do
  docker run --rm --network none --cpus=2 \
    -e ROS_DOMAIN_ID=84 -e ROS_LOCALHOST_ONLY=1 -e VERIFY_VO="$mode" \
    -v "$ROOT/config/rtabmap_rgbd.launch.py:/opt/amr/rtabmap_rgbd.launch.py:ro" \
    -v "$ROOT/ros/src/bringup/scripts/vo_gate.py:/opt/amr/vo_gate.py:ro" \
    -v "$ROOT/ros/src/bringup/scripts/map_lifecycle.py:/opt/amr/map_lifecycle.py:ro" \
    -v "$ROOT/ros/src/bringup/scripts/amr_perception:/opt/amr/amr_perception:ro" \
    -v "$ROOT/ros/src/bringup/config/perception.yaml:/opt/amr/perception.yaml:ro" \
    -v "$ROOT/tools:/tools:ro" -v "$VERIFY_DIR:/verify" \
    "$RTABMAP_IMAGE" bash -lc '
      set -eo pipefail
      source /opt/ros/humble/setup.bash
      export PYTHONPATH=/opt/amr:$PYTHONPATH
      python3 /tools/verify_rtabmap.py
    '
done
echo "All architecture checks passed: $VERIFY_DIR"
