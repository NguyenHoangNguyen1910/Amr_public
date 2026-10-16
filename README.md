# AMR

ROS 2 AMR stack for Raspberry Pi 5, using the ARM64 Yahboom image and an
Astra depth camera.

## Chạy mapping

Build image chỉ cần chạy lần đầu hoặc sau khi sửa ROS code:

```bash
./shell.sh build
```

Terminal 1:

```bash
./shell.sh bootstrap
./shell.sh rtabmap
./shell.sh run
```

RTAB-Map nhận trực tiếp RGB, depth, CameraInfo, odometry và TF qua ROS 2
DDS. Lệnh `run` mặc định dùng map `/rtabmap/map` và tắt SLAM Toolbox.
RTAB-Map chạy 5 Hz, ưu tiên frame mới nhất; Web UI cập nhật map tối đa 5 Hz.

Nếu cần dùng lại SLAM Toolbox:

```bash
MAP_TOPIC=/map USE_SLAM_TOOLBOX=true ./shell.sh run
```

Terminal 2, chạy Web UI:

```bash
./shell.sh serve_web
```

Mở trình duyệt tại `http://<ROBOT_IP>:8080/index.html`.
Lấy IP robot bằng:

```bash
hostname -I
```

RTAB-Map nhận RGB, depth, CameraInfo, odometry và TF trực tiếp qua ROS 2 DDS.
Map 2D được publish ở `/rtabmap/map`, xử lý 5 Hz và Web UI cập nhật tối đa 5 Hz.

Database được lưu tại:

```text
rtabmap_data/astra_wheel_mapping.db
```

Xem log RTAB-Map:

```bash
docker logs -f rtabmap-slam
```


## Stop

```bash
./shell.sh stop
```

Lệnh `stop` dừng ROS, micro-ROS agent và RTAB-Map.
