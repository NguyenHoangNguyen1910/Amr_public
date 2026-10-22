# AMR

ROS 2 AMR stack for Raspberry Pi 5, using the ARM64 Yahboom image and an
Astra depth camera.

Kiến trúc hiện tại lấy encoder + IMU làm odometry chính. RGB-D VO là tùy chọn,
tắt mặc định và chỉ bổ sung vận tốc tịnh tiến sau gate. Xem
[kiến trúc và cấu hình](docs/architecture.md).

Kiểm thử không kết nối phần cứng robot:

```bash
./tools/verify_architecture.sh
```

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

`rgbd_sync` đồng bộ ảnh đã đăng ký depth sang RGB và xuất `/camera/rgbd_image`.
RTAB-Map nhận RGB-D này cùng `/odom` từ EKF, không chờ VO hoặc `odom_info`.
Lệnh `run` mặc định dùng map `/rtabmap/map`, tắt SLAM Toolbox và cho navigation
đọc `/depth/scan` từ depth obstacle adapter. Astra chạy 640×480 ở 15 FPS,
tắt IR và point cloud; mapper xử lý khoảng 1 Hz, queue 3. Depth preview giảm
xuống 5 Hz. Những mức này là cấu hình khởi đầu, chưa phải cam kết độ trễ thực tế.

VO tùy chọn chạy tối đa 5 Hz, không phát TF và không gửi pose/yaw vào EKF:

```bash
ENABLE_VO=true ./shell.sh rtabmap
```

Các số đo camera/thân robot trong `ros/src/bringup/config/geometry.yaml` được
để bằng 0 theo yêu cầu. `motion_monitor` chỉ xuất cảnh báo ở `/motion/status`,
không chặn lệnh và không phát `/cmd_vel`. Arbiter vẫn là node phát lệnh cuối.

Nếu không cần navigation và chỉ muốn chạy mapping/teleop, chạy web stack bằng:

```bash
USE_NAVIGATION=false ./shell.sh run
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

Map 2D được publish ở `/rtabmap/map`. Cần đo độ trễ và sai số thực tế sau khi
hiệu chỉnh camera và kiểm chứng odometry; depth preview cập nhật riêng.

Mặc định RTAB-Map chạy theo phiên: database nằm trên `/dev/shm` và bị xóa
khi phiên mới bắt đầu, nên không ghi database xuống thẻ nhớ. Chế độ này vẫn
giữ pose graph/map trong suốt thời gian robot đang chạy.

Nếu cần lưu database để phân tích hoặc tiếp tục phiên sau, bật:

```bash
RTABMAP_PERSIST=true ./shell.sh rtabmap
```

Chế độ persistent lưu trong thư mục `rtabmap_data/`, theo tên
`RTABMAP_DATABASE_NAME` (mặc định là tên database có timestamp):

```text
rtabmap_data/<RTABMAP_DATABASE_NAME>
```

Xem log RTAB-Map:

```bash
docker logs -f rtabmap-slam
```

Database phiên trên `/dev/shm` dùng RAM thay vì storage; cần theo dõi dung
lượng RAM nếu mapping trong thời gian dài.


## Stop

```bash
./shell.sh stop
```

Lệnh `stop` dừng ROS, micro-ROS agent và RTAB-Map.
