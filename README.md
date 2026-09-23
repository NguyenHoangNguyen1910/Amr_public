# AMR

Mobile robot platform with ROS 2, navigation, and a web UI.

The same stack can run against either the real robot or a headless 2D virtual
robot. See [`docs/simulation.md`](docs/simulation.md) for backend switching and
the saved-map workflow.

![AMR Web UI demo](docs/demo.jpg)

## Topology

```text
Browser / tablet
        │ HTTP · WebSocket · MJPEG
        ▼
ROS 2 stack on Raspberry Pi 5 (Docker)
        ├── SLAM · navigation backend · EKF · web_bridge
        ├── rosbridge · camera_mjpeg · cmd_vel_arbiter
        └── Micro XRCE-DDS Agent
        │
        │ USB serial / Micro XRCE-DDS
        ▼
ESP32 micro-ROS client
        └── Motor · Encoder · IMU · Servo · Buzzer

Hardware:
Raspberry Pi 5 ── LiDAR USB · Camera USB
ESP32 ─────────── Robot hardware control
```

Main data flows:

```text
Browser / tablet
├── Goal:   /web/goal_pose ───────────────> web_bridge ─> navigation ─> /cmd_vel_nav ─┐
├── Manual: /web/cmd_vel_manual ────────────────────────────────────────────────┼─> cmd_vel_arbiter ─> /cmd_vel ─> Agent ─> ESP32 ─> Motor
├── Mode:   /web/control_mode ──────────────────────────────────────────────────┘
├── Map:    /web/map <──────────────────── web_bridge <─ /map <─ slam_toolbox <─ /scan (LiDAR)
├── Scan:   /web/scan_points <──────────── web_bridge <─ /scan (LiDAR)
├── Pose:   /robot_pose <───────────────── web_bridge <─ /odom + TF <─ EKF <─ /imu + /odom_raw (ESP32)
└── Camera: MJPEG :8081 <───────────────── camera_mjpeg <─ /dev/video0 (USB camera)
```

`web.launch.py` starts the stack in the ROS container, but does not start the
LiDAR driver or the micro-ROS Agent. See [`docs/web_topics.md`](docs/web_topics.md)
for the web topic details.

## Run the robot

Start the containers:

```bash
./bootstrap.sh
```

Open the container shell:

```bash
./shell.sh
```

Inside the container, start ROS 2 with navigation:

```bash
ros2 launch bringup web.launch.py
```

The navigation node owns `/navigate_to_pose` and publishes navigation commands to
`/cmd_vel_nav`; `cmd_vel_arbiter.py` remains the final safety gate.

## Run without robot hardware

```bash
./build.sh
ROBOT_BACKEND=simulation ./bootstrap.sh
./shell.sh
```

Inside the container:

```bash
ros2 launch bringup web.launch.py robot_backend:=simulation use_camera:=false
```

The virtual adapter publishes the same `/map`, `/scan`, `/odom` and TF contract
used by the rest of the stack, so the existing web UI and navigation remain
unchanged. It can also simulate directly on a map saved from the real robot.

### Web UI

```bash
./serve_web.sh
```

Open [http://localhost:8080](http://localhost:8080).

### Stop

Press `Ctrl+C`, then run:

```bash
docker rm -f yahboom-ros-main yahboom-micro-ros-agent
```
