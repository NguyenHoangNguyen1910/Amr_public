
# robot_nav

Luồng mapping hiện tại là:

```text
Astra RGB + depth + /odom -> RTAB-Map -> /rtabmap/map -> web_bridge -> Web UI
```

RTAB-Map dùng depth gốc `/camera/depth/image_raw` và odometry bánh xe `/odom`.
LiDAR không cần cho luồng này.

## Chạy robot thật

Chạy các bước sau theo thứ tự. `build.sh` phải chạy trước khi khởi động
backend để container dùng đúng launch file mới.

### Terminal 1: build, camera và micro-ROS

```bash
cd ~/quangnv/robot_nav
./build.sh
docker start astra-camera-slam 2>/dev/null || true
ROS_LOCALHOST_ONLY=0 AUTO_RESET=1 ./bootstrap.sh
```

Kiểm tra camera:

```bash
docker exec astra-camera-slam bash -lc '
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=0
  ros2 topic hz /camera/color/image_raw
'
```

Depth phải tồn tại:

```bash
docker exec astra-camera-slam bash -lc '
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=0
  ros2 topic list | grep /camera/depth/image_raw
'
```

### Terminal 2: RTAB-Map

```bash
cd ~/quangnv/robot_nav
./rtabmap.sh
```

Script tự cấu hình:

- `visual_odometry:=false`
- `odom_topic:=/odom`
- `frame_id:=base_link`
- `depth_topic:=/camera/depth/image_raw`
- database: `rtabmap_data/astra_wheel_mapping.db`

Log đúng phải có dòng tương tự:

```text
rtabmap subscribed to (approx sync):
   /odom
   /camera/color/image_raw
   /camera/depth/image_raw
   /camera/color/camera_info
```

### Terminal 3: backend ROS và rosbridge

```bash
cd ~/quangnv/robot_nav
./shell.sh
```

Trong container:

```bash
ros2 launch bringup web.launch.py use_rtabmap_depth_adapter:=false
```

Launch này đồng thời phát TF `base_link -> camera_link`, cần thiết để
RTAB-Map ghép depth với odometry. Mặc định offset camera bằng 0 để bảo đảm
chuỗi TF nối được; khi biết chính xác vị trí camera, truyền offset thật, ví dụ:

```bash
ros2 launch bringup web.launch.py \
  use_rtabmap_depth_adapter:=false \
  camera_mount_x:=0.10 camera_mount_z:=0.20
```

Không mở launch backend lần thứ hai. Nếu gặp `Address already in use` ở port
9090, nghĩa là rosbridge cũ vẫn đang chạy; dùng lại terminal/container đang
chạy thay vì start thêm một launch.

### Terminal 4: Web UI

```bash
cd ~/quangnv/robot_nav
./serve_web.sh
```

Mở địa chỉ mà script in ra, thường là `http://<IP-Raspberry-Pi>:8080/`.

## Kiểm tra trước khi mapping

Trong container backend, các topic sau phải có dữ liệu:

```bash
ros2 topic hz /odom
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

Nếu RTAB-Map báo `Did not receive data since 5 seconds`, hãy kiểm tra cả ba
topic và timestamp của chúng. Có publisher trong `ros2 topic info` nhưng không
có message vẫn chưa đủ để mapping.

Khi robot đứng yên, map chỉ có thể tạo vùng quan sát ban đầu. Hãy cho robot
di chuyển chậm qua khu vực cần quét; `/odom` phải thay đổi theo chuyển động.
Nếu robot đi ra ngoài OccupancyGrid rồi biến mất trên UI, đó là dấu hiệu
RTAB-Map chưa nhận được frame đồng bộ hoặc odometry, không phải do UI tự đặt
một kích thước map cố định.

## Lưu database/map

Database mapping được lưu tại:

```text
rtabmap_data/astra_wheel_mapping.db
```

Để bắt đầu một session sạch mà vẫn giữ session cũ:

```bash
RTABMAP_DATABASE_NAME=astra_wheel_mapping_new.db ./rtabmap.sh
```

Các file `.pgm/.yaml` cũ trong `maps/` là map tĩnh của ROS và không tự động
được cập nhật từ database RTAB-Map. Luồng web hiện hiển thị trực tiếp
`/rtabmap/map` thông qua `/web/map`.

## Camera depth qua WebSocket

Giao diện nhận JPEG nhị phân từ `ws://<IP-robot>:8081/camera.ws`, có
`permessage-deflate`. Camera không dùng chung rosbridge cổng 9090.
Ảnh depth, độ phân giải và JPEG quality được giữ nguyên.

Sau khi cập nhật code, chạy `./build.sh`. Script tự tạo image mặc định mới
có `python3-aiohttp` cho AMD64 hoặc ARM64. Với container ROS đang chạy từ
image cũ, dừng container đó trước khi chạy lại `./shell.sh`, rồi khởi động
`ros2 launch bringup web.launch.py` và tải lại trang web.
Nếu tự đặt `ROS_IMAGE`, image đó phải có `python3-aiohttp`.

Xem [cách kiểm tra WebSocket compression](docs/web_topics.md#camera-depth-websocket).
