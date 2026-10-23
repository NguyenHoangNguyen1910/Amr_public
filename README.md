# AMR

ROS 2 AMR stack for Raspberry Pi 5, using the ARM64 Yahboom image and an
Astra depth camera.

Kiến trúc hiện tại lấy encoder + IMU làm odometry chính. RGB-D VO bật mặc định,
chỉ bổ sung vận tốc tịnh tiến sau gate. Xem
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

VO chạy tối đa 5 Hz, không phát TF và không gửi pose/yaw vào EKF. Có thể tắt bằng:

```bash
ENABLE_VO=false ./shell.sh rtabmap
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

## Chốt map để tiếp tục định vị và điều hướng

Khi map đã quét đủ và robot đứng yên, bấm **Dừng quét map** trên Web UI.
Hệ thống lưu database RTAB-Map vào `rtabmap_data/map-*.db`, chọn nó làm
`active_map`, rồi chuyển RTAB-Map sang localization mode. RTAB-Map vẫn chạy
để hiệu chỉnh vị trí `map -> odom`; camera depth và `/depth/scan` vẫn cập nhật
vật cản mới cho navigation. Nút chỉ báo hoàn tất khi database đã lưu và
chuyển chế độ thành công.

Ở lần chạy `./shell.sh rtabmap` tiếp theo, `active_map` được sao vào RAM và
RTAB-Map tự khởi động ở localization mode. Bản trên ổ lưu trữ không bị ghi
trong lúc định vị. Muốn quét một map mới, chạy:

```bash
RTABMAP_NEW_MAP=true ./shell.sh rtabmap
```

Có thể chọn một bản map đã lưu khác bằng:

```bash
RTABMAP_MAP_NAME=<tên-map> ./shell.sh rtabmap
```

Chuyển sang phiên RTAB-Map mới sẽ ngắt map/TF trong lúc
khởi động; hãy dừng Nav trước. Map trong RAM mất khi tắt nguồn, còn bản
`rtabmap_data/map-*.db` vẫn được giữ để nạp lại.

Map 2D được publish ở `/rtabmap/map`. Cần đo độ trễ và sai số thực tế sau khi
hiệu chỉnh camera và kiểm chứng odometry; depth preview cập nhật riêng.

Trước khi chốt map, RTAB-Map tạo database phiên trong `/dev/shm`; một phiên
mapping mới sẽ xóa database phiên cũ. Sau khi chốt, bản lưu trên ổ được nạp
lại vào `/dev/shm` để localization ở các phiên sau.

Nếu cần lưu database để phân tích hoặc tiếp tục phiên sau, bật:

```bash
RTABMAP_NEW_MAP=true RTABMAP_PERSIST=true ./shell.sh rtabmap
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
