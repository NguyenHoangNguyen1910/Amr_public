#!/usr/bin/env bash
set -Eeuo pipefail

readonly AGENT_IMAGE="${AGENT_IMAGE:-microros/micro-ros-agent:humble}"
readonly AGENT_CONTAINER="${AGENT_CONTAINER:-yahboom-micro-ros-agent}"
readonly ROS_CONTAINER="${ROS_CONTAINER:-yahboom-ros-main}"
readonly SERIAL_DEVICE="${SERIAL_DEVICE:-/dev/ttyUSB0}"
readonly SERIAL_BAUDRATE="${SERIAL_BAUDRATE:-921600}"
readonly ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
readonly ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
readonly AUTO_RESET="${AUTO_RESET:-1}"
readonly ROBOT_BACKEND="${ROBOT_BACKEND:-hardware}"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v docker >/dev/null 2>&1; then
  echo "Error: docker is not installed or is not in PATH." >&2
  exit 1
fi

if [[ "$ROBOT_BACKEND" != "hardware" && "$ROBOT_BACKEND" != "simulation" ]]; then
  echo "Error: ROBOT_BACKEND must be hardware or simulation." >&2
  exit 2
fi

if docker ps --format '{{.Names}}' | grep -Fxq "$ROS_CONTAINER"; then
  running_backend="$(docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' \
    "$ROS_CONTAINER" | sed -n 's/^ROBOT_BACKEND=//p' | tail -n 1)"
  running_backend="${running_backend:-hardware}"
  if [[ "$running_backend" != "$ROBOT_BACKEND" ]]; then
    echo "Error: $ROS_CONTAINER is already running with ROBOT_BACKEND=$running_backend." >&2
    echo "Stop it before switching backend: docker stop $ROS_CONTAINER" >&2
    exit 1
  fi
fi

if [[ "$ROBOT_BACKEND" == "simulation" ]]; then
  if docker ps --format '{{.Names}}' | grep -Fxq "$AGENT_CONTAINER"; then
    echo "Error: $AGENT_CONTAINER is still connected to the real robot." >&2
    echo "Stop it before simulation: docker stop $AGENT_CONTAINER" >&2
    exit 1
  fi
  echo "Starting without robot hardware; the ROS simulation adapter will provide sensors and motion."
  "$SCRIPT_DIR/shell.sh" --background
  echo "Next: ./shell.sh"
  echo "Then: ros2 launch bringup web.launch.py robot_backend:=simulation use_camera:=false"
  exit 0
fi

if [[ ! -e "$SERIAL_DEVICE" ]]; then
  echo "Error: serial device $SERIAL_DEVICE was not found." >&2
  echo "Check the robot power, USB cable, and: ls /dev/ttyUSB*" >&2
  exit 1
fi

running_agent="$(docker ps -q --filter "ancestor=$AGENT_IMAGE" | head -n 1)"
if [[ -n "$running_agent" ]]; then
  echo "micro-ROS agent is already running: $running_agent"
else
  echo "Starting micro-ROS agent on $SERIAL_DEVICE at $SERIAL_BAUDRATE baud (domain $ROS_DOMAIN_ID)."

  agent_id="$(docker run -d --rm \
    --name "$AGENT_CONTAINER" \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    --privileged \
    --net=host \
    --env="ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
    --env="ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY" \
    "$AGENT_IMAGE" \
    serial --dev "$SERIAL_DEVICE" -b "$SERIAL_BAUDRATE" -v4)"

  echo "micro-ROS agent is running in the background: $agent_id"

  if [[ "$AUTO_RESET" == "1" ]]; then
    echo "Attempting automatic reset of the microROS control board..."
    # Give the container time to open the serial port before resetting the board.
    sleep 1
    if ! python3 - "$SERIAL_DEVICE" "$SERIAL_BAUDRATE" <<'PY'
import sys
import time

try:
    import serial
except ImportError:
    print("python3-serial is not installed", file=sys.stderr)
    raise SystemExit(2)

device = sys.argv[1]
baudrate = int(sys.argv[2])

try:
    # Yahboom/ESP32 USB-UART boards commonly connect DTR/RTS to EN/BOOT.
    # Toggling both lines is the software equivalent of pressing RESET.
    with serial.Serial(device, baudrate=baudrate, timeout=0.2) as port:
        port.dtr = False
        port.rts = False
        time.sleep(0.1)
        port.dtr = True
        port.rts = True
        time.sleep(0.2)
except Exception as exc:
    print(f"automatic reset failed: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY
    then
      echo "Warning: automatic reset failed; press RESET on the microROS control board." >&2
    else
      echo "Automatic reset signal sent."
    fi
  else
    echo "Automatic reset disabled (AUTO_RESET=$AUTO_RESET)."
  fi
fi

"$SCRIPT_DIR/shell.sh" --background

echo "View logs: docker logs -f $AGENT_CONTAINER"
echo "Stop agent: docker stop $AGENT_CONTAINER"
