# AMR

ROS 2 AMR stack for Raspberry Pi 5, using the ARM64 Yahboom image and an
Astra depth camera.

## Chạy mapping RGB-D

Mapping mặc định dùng camera Astra bên ngoài, RTAB-Map là backend duy nhất.
`shell.sh` sẽ tự start container `astra-camera-slam`, container phải publish
RGB, depth và CameraInfo trên cùng ROS domain. Có thể start riêng bằng:

```bash
./shell.sh camera
```

Nếu container `astra-camera-slam` chưa tồn tại, cần cài/provision camera
driver trước; không chạy đồng thời driver này và `astra_bridge.py`.

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
DDS. Lệnh `run` mặc định dùng map `/rtabmap/map`, tắt SLAM Toolbox và không
khởi động bridge Astra nội bộ. RTAB-Map xử lý 1 Hz, chờ TF tối đa 3 giây,
và xử lý frame theo thứ tự timestamp.

Depth preview trên Web UI subscribe `/camera/depth/image_raw` qua WebSocket
8082, không sở hữu camera và không làm chậm RTAB-Map.

Nếu tạm thời chưa có camera container, có thể dùng bridge Astra cũ để kiểm
tra phần còn lại của hệ thống:

```bash
USE_ASTRA=true ./shell.sh run
```

Bridge này chỉ là fallback; nó vẫn dùng CameraInfo tự tạo nên không phải cấu
hình mapping chính thức.

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
