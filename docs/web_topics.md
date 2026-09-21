# Web ROS topics

Tài liệu này mô tả luồng dữ liệu giữa `web/index.html` và các node trong
`ros/src/bringup/scripts/`. Toàn bộ stack được khởi chạy bằng:

```bash
ros2 launch bringup web.launch.py
```

## Kết nối rosbridge

Frontend tạo hai kết nối WebSocket tới cùng một rosbridge:

```text
ws://<hostname>:9090
```

Nếu trang được phục vụ qua HTTPS, frontend dùng `wss://` thay cho `ws://`.

| Connection | Vai trò |
|---|---|
| `realtimeRos` | Pose robot, LiDAR packed và trạng thái navigation |
| `bulkRos` | Map và path |

Camera được phục vụ riêng qua HTTP MJPEG tại:

```text
http://<hostname>:8081/camera.mjpg
```

Frontend kiểm tra trạng thái camera mỗi 2 giây qua
`http://<hostname>:8081/healthz`.

## Các topic subscribe

| Topic | Type | Throttle | Chức năng |
|---|---|---:|---|
| `/web/map` | `nav_msgs/msg/OccupancyGrid` | 500 ms | Nhận occupancy grid chỉ khi map mới/thay đổi và vẽ nền map |
| `/robot_pose` | `geometry_msgs/msg/PoseStamped` | 50 ms | Nhận vị trí/hướng robot, vẽ robot màu xanh và xác nhận robot đã được định vị |
| `/web/scan_points` | `bringup/msg/ScanPoints` | 200 ms | Nhận LiDAR packed trên realtime socket và vẽ màu cam |
| `/plan` | `nav_msgs/msg/Path` | 250 ms | Nhận đường đi navigation và vẽ màu xanh tím |
| `/web/nav_status` | `std_msgs/msg/String` | 0 | Nhận trạng thái điều hướng tới goal |

### `/web/map`

Frontend sử dụng các trường:

```json
{
  "info": {
    "width": 640,
    "height": 480,
    "resolution": 0.05,
    "origin": {
      "position": {"x": 0, "y": 0, "z": 0},
      "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
    }
  },
  "data": [-1, 0, 100]
}
```

Backend chỉ publish topic này lần đầu hoặc khi metadata/`data[]` thực sự thay
đổi; các lần publish `/map` giữ nguyên nội dung sẽ bị bỏ qua. Publisher dùng
QoS `TRANSIENT_LOCAL` để giữ map gần nhất cho subscriber dùng durability này.

`data[]` được dùng để tô màu từng ô:

- `-1`: ô chưa biết;
- `0`: ô trống;
- `100`: vật cản.

`info.origin` và `info.resolution` được dùng để chuyển tọa độ world sang tọa độ canvas.

### `/robot_pose`

```json
{
  "header": {"frame_id": "map"},
  "pose": {
    "position": {"x": 1.2, "y": 0.8, "z": 0},
    "orientation": {"x": 0, "y": 0, "z": 0.38, "w": 0.92}
  }
}
```

Khi nhận message, frontend cập nhật `state.pose`, đặt `state.localized = true`, sau đó vẽ vị trí và hướng robot.

### `/web/scan_points`

Topic này dùng message `bringup/msg/ScanPoints`:

```json
{
  "header": {
    "stamp": {"sec": 1789717639, "nanosec": 243000000},
    "frame_id": "map"
  },
  "seq": 1024,
  "points": [1.0, 0.2, 1.1, 0.25]
}
```

`points[]` có dạng `[x0, y0, x1, y1, ...]`. Backend đã transform điểm
sang frame `map` bằng TF tại timestamp của scan. Mặc định bridge publish tối
đa `5 Hz`, lấy cách điểm tối thiểu là `2` và giới hạn mỗi message ở `180` điểm.
Nếu chưa có TF đúng timestamp, scan đó bị bỏ thay vì dùng transform sai thời
điểm.

Luồng xử lý:

```text
/web/scan_points
    -> state.scan
    -> dirty.scan = true
    -> scheduleRender() (requestAnimationFrame)
    -> drawScanLayer()
    -> điểm LiDAR màu cam trên map
```

Frontend dùng trực tiếp từng cặp `x, y`; scan không còn phụ thuộc vào pose
frontend mới nhất để biến đổi tọa độ. `seq` được dùng để bỏ message cũ hoặc
trùng lặp.

### `/plan`

Topic này cũng là `nav_msgs/msg/Path`:

```json
{
  "poses": [
    {
      "pose": {
        "position": {"x": 1.0, "y": 0.5, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
      }
    }
  ]
}
```

Frontend lưu message vào `state.path` rồi vẽ các `poses[].pose.position` thành đường màu xanh tím.

### `/web/nav_status`

`msg.data` là JSON string:

```json
{"state":"MOVING","message":"robot is navigating"}
```

Các trạng thái được frontend xử lý:

| `state` | Xử lý |
|---|---|
| `PLANNING`, `MOVING` | Đặt `navigationActive = true`, bật Cancel |
| `SUCCEEDED`, `CANCELED`, `FAILED`, `ERROR`, `IDLE` | Đặt `navigationActive = false`, tắt Cancel và chuyển control mode về `manual` |
| `LOCALIZATION_REQUIRED` | Frontend không đổi trạng thái navigation; nút GO vẫn chờ robot pose |

Frontend không tự tính khoảng cách robot-goal. Việc robot đã tới goal được xác định qua `state = "SUCCEEDED"` từ topic này.

### Camera MJPEG

Node `ros/src/bringup/scripts/camera_mjpeg.py` subscribe
`/usb_cam/image_raw/compressed`, giữ lại frame JPEG mới nhất và phục vụ
multipart HTTP tại `/camera.mjpg`. Frontend dùng trực tiếp URL này trong thẻ
`<img>`, nên ảnh camera không đi qua rosbridge hoặc hai WebSocket của UI.

## Các topic publish

| Topic | Type | Khi publish | Chức năng |
|---|---|---|---|
| `/web/control_mode` | `std_msgs/msg/String` | Bấm GO, Cancel, navigation kết thúc | Chuyển giữa `nav` và `manual` |
| `/web/goal_pose` | `geometry_msgs/msg/PoseStamped` | Bấm GO | Gửi goal tới backend navigation |
| `/web/cancel_navigation` | `std_msgs/msg/Empty` | Bấm Cancel hoặc đóng trang | Hủy navigation |
| `/web/cmd_vel_manual` | `geometry_msgs/msg/Twist` | Nhấn/thả phím mũi tên, lặp mỗi 100 ms khi đang giữ lệnh | Điều khiển robot bằng tay |

### `/web/control_mode`

Khi bấm GO:

```json
{"data":"nav"}
```

Khi bấm Cancel hoặc navigation kết thúc:

```json
{"data":"manual"}
```

### `/web/goal_pose`

```json
{
  "header": {
    "stamp": {"sec": 0, "nanosec": 0},
    "frame_id": "map"
  },
  "pose": {
    "position": {"x": 1.2, "y": 0.8, "z": 0},
    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
  }
}
```

Đặc điểm:

- `x, y` được tính từ vị trí click trên canvas qua `canvasToWorld()`;
- `z = 0`;
- yaw luôn bằng `0`;
- timestamp luôn bằng `0`;
- frame là `map`.

### `/web/cancel_navigation`

Publish message rỗng:

```json
{}
```

Được gửi khi bấm Cancel và trong `beforeunload` khi đóng/rời trang.

### `/web/cmd_vel_manual`

```json
{
  "linear": {"x": 0.22, "y": 0, "z": 0},
  "angular": {"x": 0, "y": 0, "z": 0.55}
}
```

Mapping phím:

| Phím | Lệnh |
|---|---|
| `ArrowUp` | `linear.x` dương |
| `ArrowDown` | `linear.x` âm |
| `ArrowLeft` | `angular.z` dương |
| `ArrowRight` | `angular.z` âm |

Khi nhả phím, mất focus cửa sổ hoặc chuyển sang navigation, frontend gửi lệnh dừng:

```json
{
  "linear": {"x": 0, "y": 0, "z": 0},
  "angular": {"x": 0, "y": 0, "z": 0}
}
```

Node `ros/src/bringup/scripts/cmd_vel_arbiter.py` chọn duy nhất nguồn manual
hoặc custom navigation, giới hạn `|linear.x| <= 0.22 m/s` và
`|angular.z| <= 0.55 rad/s`. Lệnh manual hết hạn sau `350 ms`; vì vậy xe tự
dừng nếu trình duyệt ngừng gửi lệnh. Arbiter xuất lệnh an toàn cuối cùng lên
`/cmd_vel` ở `20 Hz`.

## Luồng goal tổng quát

```text
Bấm Set Goal
    -> click lên map
    -> canvasToWorld()
    -> state.goal = {position: {x, y}, yaw: 0}
    -> kiểm tra map/pose/navigation
    -> bấm GO
    -> publish /web/control_mode = "nav"
    -> publish /web/goal_pose
    -> nhận /web/nav_status = "PLANNING" hoặc "MOVING"
    -> nhận /web/nav_status = "SUCCEEDED"
    -> publish /web/control_mode = "manual"
```

## Luồng backend chính

```text
/map  -> web_bridge -> /web/map         -> bulkRos     -> map canvas
/scan -> web_bridge -> /web/scan_points -> realtimeRos -> scan canvas
TF map -> base_footprint
      -> web_bridge -> /robot_pose       -> realtimeRos -> overlay canvas

Web keyboard -> /web/cmd_vel_manual -------------------------------+
Custom navigation -> /cmd_vel_nav -------------------------------+-> cmd_vel_arbiter -> /cmd_vel
Web UI       -> /web/control_mode --------------------------------+
```

`web_bridge.py`, `camera_mjpeg.py` và `cmd_vel_arbiter.py` là ba process độc
lập. `msg/ScanPoints.msg` chỉ định nghĩa schema ROS và không tạo thêm process
runtime.
