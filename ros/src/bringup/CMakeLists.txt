cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToPose.action"
  "msg/ScanPoints.msg"
  "srv/ResetSimulation.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

install(
  PROGRAMS
    scripts/camera_mjpeg.py
    scripts/depth_rtabmap_adapter.py
    scripts/cmd_vel_arbiter.py
    scripts/web_bridge.py
    scripts/navigation_node.py
    scripts/simulation_adapter.py
    scripts/imu_covariance_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/camera_stream.py DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(navigation
  PACKAGE_DIR scripts/navigation
)

ament_python_install_package(simulation
  PACKAGE_DIR scripts/simulation
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

install(
  DIRECTORY config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
