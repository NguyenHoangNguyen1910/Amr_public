cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScanPoints.msg"
  DEPENDENCIES std_msgs
)

install(
  PROGRAMS
    scripts/camera_mjpeg.py
    scripts/cmd_vel_arbiter.py
    scripts/web_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
