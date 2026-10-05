# Interactive container setup: the image's .bashrc may source an older underlay.
# Source the workspace LAST so ros2 run/launch resolves the code just built.
if [[ -f /root/.bashrc ]]; then
  source /root/.bashrc
fi
source /opt/ros/humble/setup.bash
if [[ -f /root/imu_ws/install/imu_complementary_filter/share/imu_complementary_filter/package.bash ]]; then
  source /root/imu_ws/install/imu_complementary_filter/share/imu_complementary_filter/package.bash
fi
source /workspace/install/robot_localization/share/robot_localization/package.bash
source /workspace/install/bringup/share/bringup/package.bash
