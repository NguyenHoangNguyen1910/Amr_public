# AMR

ROS 2 AMR stack for Raspberry Pi 5, using the ARM64 Yahboom image and an
Astra depth camera.

## Chạy mapping RGB-D

Mapping mặc định dùng driver Astra chuẩn trong container
`astra-camera-slam`, image `husarion/astra:humble-nightly`. `shell.sh` tự
pull, tạo và khởi động container này; Web stack chạy trong ROS container
chính và nhận topic qua ROS 2 DDS.

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

RTAB-Map nhận trực tiếp RGB, depth và CameraInfo từ Astra; RGB-D odometry
tích hợp của RTAB-Map publish `visual_odom` vì hệ thống không có LiDAR. Lệnh
`run` mặc định dùng map `/rtabmap/map`, tắt SLAM Toolbox, navigation/scan và
chạy depth WebSocket trong ROS container. Astra giữ độ phân giải 640×480 nhưng
chạy RGB/depth 15 FPS, tắt IR và point cloud; RTAB-Map xử lý map 1 Hz, RGB-D
odometry tối đa 10 Hz, queue 15 frame và ưu tiên frame mới nhất để giữ độ trễ
trong khoảng 2–3 giây.

Nếu robot được gắn LiDAR và cần navigation, chạy lại web stack bằng:

```bash
USE_NAVIGATION=true ./shell.sh run
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

Map 2D được publish ở `/rtabmap/map`. Độ trễ map có thể khoảng 2–3 giây khi
RTAB-Map đang chờ TF hoặc xử lý backlog; depth preview vẫn cập nhật riêng.

Database được lưu trong thư mục `rtabmap_data/`, theo tên
`RTABMAP_DATABASE_NAME` (mặc định là tên database có timestamp):

```text
rtabmap_data/<RTABMAP_DATABASE_NAME>
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
