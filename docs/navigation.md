> Current hardware defaults: RTAB-Map `/rtabmap/map`, projected `/depth/scan`, encoder + IMU EKF; VO optional and off. See [architecture.md](architecture.md) for current startup, geometry and advisory-only monitoring. The sections below include the original planner design.

# Navigation Integration

## 1. Mục tiêu

Tích hợp backend navigation sử dụng Dijkstra cost-to-go và DWA-lite từ repository tham
khảo:

```text
/home/pi/quangnv/bk/amr/src/navigation
```

Repository tham khảo chỉ được đọc. Mọi file cần thiết phải được copy sang và
chỉnh sửa trong:

```text
/home/pi/quangnv/amr
```

Kết quả mong muốn:

- Không thay đổi Web UI hoặc ROS topic mà Web UI đang dùng.
- Không thay đổi luồng điều khiển manual.
- Giữ `web_bridge.py` làm client của action `/navigate_to_pose`.
- Navigation publish vận tốc vào `/cmd_vel_nav`; không publish trực tiếp
  vào `/cmd_vel`.
- `cmd_vel_arbiter.py` vẫn là nơi duy nhất chọn manual/navigation, giới hạn vận
  tốc và publish `/cmd_vel` tới ESP32.
- Dùng `/map` của `slam_toolbox`, pose từ TF và dữ liệu `/depth/scan`, `/odom` hiện có.
- Chỉ chạy một navigation node.

## 2. Ngoài phạm vi

Giai đoạn này không thực hiện:

- Sửa code trong `/home/pi/quangnv/bk/amr`.
- Dùng Gazebo Transport hoặc copy `src/transport.py` từ repository tham khảo.
- Thay đổi firmware ESP32, micro-ROS Agent hoặc topic `/cmd_vel`.
- Thay đổi giao thức giữa browser và rosbridge.
- Thay `slam_toolbox`, EKF hoặc cây TF hiện tại.
- Không giữ thêm cấu hình hoặc runtime navigation dư thừa.

## 3. Kiến trúc mục tiêu

```text
Browser
   │
   ├── /web/goal_pose
   ├── /web/cancel_navigation
   └── /web/control_mode
   │
   ▼
web_bridge.py
   │  bringup/action/NavigateToPose
   ▼
/navigate_to_pose
   │
   ▼
navigation_node.py
   ├── Global planner: Dijkstra cost-to-go
   ├── Local planner: DWA-lite
   ├── subscribe /map, /depth/scan, /odom
   ├── lookup TF map -> base_footprint
   ├── publish /plan
   └── publish /cmd_vel_nav
                    │
                    ▼
             cmd_vel_arbiter.py
                    │
                    ▼
                 /cmd_vel
                    │
                    ▼
             micro-ROS / ESP32
```

Navigation node và `web_bridge.py` dùng action type nội bộ
`bringup/action/NavigateToPose`; đây là contract ROS duy nhất của navigation.

## 4. File dự kiến thêm và sửa

### 4.1. File mới

```text
ros/src/bringup/scripts/navigation_node.py
ros/src/bringup/scripts/navigation/__init__.py
ros/src/bringup/scripts/navigation/config.py
ros/src/bringup/scripts/navigation/types.py
ros/src/bringup/scripts/navigation/global_planner.py
ros/src/bringup/scripts/navigation/local_planner.py
ros/src/bringup/scripts/navigation/navigator.py
ros/src/bringup/scripts/navigation/ros_adapter.py
ros/src/bringup/config/navigation.yaml
ros/src/bringup/test/test_navigation_map.py
ros/src/bringup/test/test_navigation_action.py
ros/src/bringup/test/test_navigation_safety.py
```

Vai trò:

- `navigation_node.py`: entrypoint ROS 2, tạo node và executor.
- `config.py`: schema, validation và fallback cho cấu hình navigation.
- `types.py`: data contracts, phép biến đổi hình học, chuẩn hóa góc và xử lý
  path.
- `global_planner.py`: occupancy grid, inflation, costmap và Dijkstra.
- `local_planner.py`: DWA-lite và kiểm tra trajectory.
- `navigator.py`: synchronous navigation session; mỗi `step()` nhận sensor
  snapshot và trả command/path/outcome.
- `ros_adapter.py`: chuyển ROS message/TF thành input planner, triển khai action
  server và publish ROS output.
- `navigation.yaml`: toàn bộ tham số có thể tune mà không sửa code.
- Các test: conversion map, lifecycle của action và các điều kiện dừng an toàn.

### 4.2. File hiện tại cần sửa

| File | Thay đổi |
|---|---|
| `ros/src/bringup/launch/web.launch.py` | Khởi chạy navigation node duy nhất |
| `ros/src/bringup/CMakeLists.txt` | Cài executable, Python package và config SLAM |
| `ros/src/bringup/package.xml` | Khai báo Python/NumPy và các ROS dependency trực tiếp của navigation node |
| `shell.sh build` | Kiểm tra executable và module đã được cài đúng |
| `README.md` | Cập nhật topology và lệnh chạy navigation |
| `docs/web_topics.md` | Ghi rõ navigation node publish `/plan` và nhận goal qua local action |

### 4.3. File giữ nguyên hành vi

- `ros/src/bringup/scripts/web_bridge.py` tiếp tục gửi local
  `NavigateToPose.Goal`, nhận feedback và xử lý action result.
- `ros/src/bringup/scripts/cmd_vel_arbiter.py` tiếp tục nhận
  `/cmd_vel_nav`, áp giới hạn và timeout rồi mới publish `/cmd_vel`.
- `ros/src/bringup/config/slam.yaml` tiếp tục cung cấp map và TF.
- `web/index.html` không đổi topic hoặc schema message.

## 5. Phạm vi code được copy

Copy logic từ repository tham khảo theo ánh xạ sau:

| Nguồn chỉ đọc | Đích trong repository hiện tại |
|---|---|
| `src/navigation/global_planner.py` | `scripts/navigation/global_planner.py` |
| `src/navigation/local_planner.py` | `scripts/navigation/local_planner.py` |
| `src/navigation/navigator.py` | `scripts/navigation/navigator.py` |
| `src/navigation/types.py` | `scripts/navigation/types.py` |
| Phần `NavigationConfig` trong `src/config.py` | `scripts/navigation/config.py` |

Không copy:

- `src/transport.py`, vì file này dùng Gazebo Transport.
- `src/server.py`, FastAPI adapter và HTTP gateway.
- `src/navigation/factory.py` và các adapter navigation không dùng trong ROS runtime.
- Static map của simulator trong `src/sim/maps`.

Sau khi copy, import phải đổi sang relative import trong package
`navigation`. Không thêm đường dẫn của repository `bk` vào `PYTHONPATH`
và không tạo symlink tới repository đó.

## 6. ROS input adapter

### 6.1. Map

Navigation node subscribe:

```text
/map  nav_msgs/msg/OccupancyGrid
```

QoS phải tương thích publisher của `slam_toolbox`: reliable, transient local,
depth 1.

Chuyển đổi map:

- `data >= occupied_threshold`: occupied.
- `data == -1`: unknown và được coi là blocked.
- Các cell còn lại: free.
- Dùng `info.resolution` và `info.origin` để chuyển world/grid.
- Không hard-code kích thước hoặc resolution của map.
- Inflation dùng `robot_radius_m + safety_margin_m`.
- Validate width, height, data length, resolution và origin trước khi nhận map.

Planner chỉ được nhận goal nằm trong vùng đã biết và không bị inflation. Nếu
chưa có map, action goal bị reject với log rõ ràng.

Mỗi goal dùng một snapshot của map mới nhất. Map nhận trong lúc robot đang chạy
được áp dụng cho goal kế tiếp; LiDAR và costmap động chịu trách nhiệm tránh vật
cản tức thời trong mission hiện tại.

### 6.2. Pose và velocity

Pose điều khiển phải nằm trong frame `map`:

```text
map -> base_footprint
```

Navigation node lookup TF ở timestamp phù hợp, không lấy pose từ tọa độ odom rồi
coi trực tiếp là map pose. `/odom` cung cấp measured linear/angular velocity.

Một control cycle chỉ hợp lệ khi:

- Có TF `map -> base_footprint`.
- `/odom` và `/depth/scan` đã được nhận.
- Tuổi dữ liệu nhỏ hơn timeout cấu hình.
- Tất cả giá trị dùng cho planner là finite.

Nếu dữ liệu hết hạn trong lúc chạy, node publish zero ngay và kết thúc action ở
trạng thái aborted.

### 6.3. LiDAR

Navigation node subscribe:

```text
/depth/scan  sensor_msgs/msg/LaserScan
```

Scan được chuyển thành obstacle points trong frame `base_footprint` bằng TF tại
timestamp của scan. Không hard-code số ray bằng `360`, vì LiDAR thật có thể có
số sample khác simulator. Quy tắc dữ liệu:

- Bỏ NaN và range ngoài `[range_min, range_max]`.
- `+inf` nghĩa là không có obstacle trên ray đó.
- Không clamp invalid range thành obstacle giả ở `range_min`.
- Drop scan nếu không có TF đúng timestamp.
- Tăng sequence chỉ khi một scan hợp lệ được nhận.

## 7. ROS output và action contract

### 7.1. Velocity

Navigation node chỉ publish:

```text
/cmd_vel_nav  geometry_msgs/msg/Twist
```

Không publish `/cmd_vel`. Vận tốc phải bị giới hạn ở cả navigation config và
`cmd_vel_arbiter.py`. Baseline không được vượt:

```text
-0.22 m/s <= linear.x <= 0.30 m/s
|angular.z| <= 0.55 rad/s
```

Node lặp command ở `20 Hz` để không vượt timeout `0.60 s` của arbiter. Zero
velocity phải được publish nhiều lần khi cancel, abort, shutdown hoặc exception.

### 7.2. Global path

Mỗi plan hợp lệ được publish lên:

```text
/plan  nav_msgs/msg/Path
```

Path dùng frame `map`, timestamp hiện tại và là path đã resample mà local
planner thực sự theo. Publish empty path khi cancel, failure hoặc goal hoàn tất
để UI không giữ path cũ.

### 7.3. NavigateToPose action

Navigation node cung cấp:

```text
/navigate_to_pose  bringup/action/NavigateToPose
```

Luồng action:

1. Validate frame goal là `map`, tọa độ/quaternion finite và quaternion hợp lệ.
2. Kiểm tra map đã sẵn sàng; TF, odometry và scan được kiểm tra trước mỗi
   control step.
3. Chuyển goal thành `Point2D(x, y, yaw)`.
4. Chạy global plan; reject/abort nếu goal ngoài map, unknown, occupied hoặc
   không có path.
5. Action loop đọc sensor, gọi `navigator.step()` và trả feedback định kỳ.
6. Điền tối thiểu `feedback.distance_remaining`; đây là field
   `web_bridge.py` đang dùng.
7. Trả succeeded khi đạt cả position và yaw tolerance trong đủ số cycle ổn
   định.
8. Trả canceled sau khi nhận cancel và đã publish zero.
9. Trả aborted khi blocked, sensor stale, TF lỗi hoặc planner/controller lỗi.

Chỉ một goal được active tại một thời điểm. Cancel callback publish zero ngay;
action loop không có worker thread hoặc revision token riêng.

## 8. State mapping

State nội bộ và action result được ánh xạ như sau:

| Navigation state | Action/Web behavior |
|---|---|
| `NAVIGATING` | Action active, `web_bridge.py` publish `MOVING` |
| `SUCCEEDED` | Action succeed, Web UI nhận `SUCCEEDED` |
| `CANCELLED` | Action canceled, Web UI nhận `CANCELED` |
| `BLOCKED` | Action aborted, Web UI nhận `FAILED` |
| `ERROR` | Action aborted, Web UI nhận `FAILED` hoặc `ERROR` qua log/status |

`web_bridge.py` hiện là nơi publish `/web/nav_status`; navigation node không publish
cạnh tranh vào topic đó.

## 9. Cấu hình ban đầu

`navigation.yaml` phải dùng thông số robot hiện tại làm baseline, không
dùng nguyên giá trị của simulator:

```yaml
navigation:
  ros__parameters:
    map_frame: map
    base_frame: base_footprint
    scan_topic: /depth/scan
    odom_topic: /odom
    map_topic: /map
    cmd_vel_topic: /cmd_vel_nav
    plan_topic: /plan

    robot_radius_m: 0.05
    safety_margin_m: 0.05  # global inflation = 0.10 m
    occupied_threshold: 65

    min_linear_velocity_mps: -0.22
    default_linear_velocity_mps: 0.30
    max_linear_velocity_mps: 0.30
    max_angular_velocity_rps: 0.55
    linear_acceleration_mps2: 0.50
    angular_acceleration_rps2: 1.20

    goal_tolerance_m: 0.12
    goal_yaw_tolerance_rad: 0.20
    control_hz: 5.0
    sensor_timeout_seconds: 0.60
    no_progress_timeout_seconds: 12.0
```

Các trọng số DWA, lookahead, prediction horizon, sample count và clearance được
copy làm điểm bắt đầu nhưng phải benchmark trên Raspberry Pi 5 và tune với vận
tốc thấp của robot thật.

`front_obstacle_clearance_m` đặt khoảng hở phía trước tính từ mũi robot. Planner
chỉ áp dụng khoảng hở này cho scan points nằm trong hành lang rộng bằng thân
robot cộng `local_safety_margin_m`; khoảng hở quanh hai bên vẫn do bán kính và
local margin hiện tại quyết định. `robot_front_extent_m` và
`robot_half_width_m` được đo từ `base_footprint`. Kiểm tra phía trước dùng dữ
liệu của `scan_topic`; depth camera preview không phải đầu vào tránh vật cản.

Khi chạy qua `shell.sh`, launch đọc trực tiếp
`/workspace/src/bringup/config/navigation.yaml`. Sửa file này rồi restart launch
để áp dụng, không cần build lại. Có thể chọn file khác bằng
`navigation_config:=/path/to/navigation.yaml`.

## 10. Launch navigation

Lệnh chạy:

```bash
ros2 launch bringup web.launch.py
```

Quy tắc:

- `navigation_node.py` là navigation node duy nhất.
- Launch không tạo thêm planner, controller, smoother, behavior hoặc lifecycle
  navigation process nào khác.

## 11. An toàn và concurrency

Các invariant bắt buộc:

- Chỉ `cmd_vel_arbiter.py` publish lệnh cuối cùng lên `/cmd_vel`.
- Không có command non-zero khi action không active.
- Cancel, emergency stop, sensor timeout, TF failure, exception và shutdown đều
  dẫn tới zero velocity.
- Action loop là nơi duy nhất gọi planner/controller và publish command.
- Mỗi input dùng pose `map -> base` tại timestamp của scan để pose và obstacle
  points cùng thời điểm.
- Goal mới không được dùng map/pose/depth/scan chưa sẵn sàng.
- Unknown map cell không được coi là free.
- Local planner kiểm tra cả inflated static map và obstacle points từ scan.
- Log lỗi được throttle để tránh làm nghẽn Pi khi TF hoặc sensor mất liên tục.

Emergency stop vẫn do `web_bridge.py` hủy action và
`cmd_vel_arbiter.py` latch output zero. Navigation node phải xử lý cancel độc lập để
không phụ thuộc riêng vào arbiter cho trạng thái action.

## 12. Kế hoạch triển khai

### Giai đoạn 1: Vendor và unit test thuật toán

1. Copy các module được liệt kê ở mục 5.
2. Đổi import sang package local và bỏ dependency Gazebo/FastAPI.
3. Tạo config theo giới hạn robot hiện tại.
4. Port unit tests cho occupancy grid, Dijkstra, DWA và navigator state machine.
5. Xác nhận toàn bộ test chạy mà không cần ROS graph hoặc phần cứng.

Điều kiện hoàn tất:

- Không có import từ `/home/pi/quangnv/bk/amr`.
- Không có import `gz.*`.
- Planner test pass với map nhỏ, goal blocked, no-path và obstacle động.

### Giai đoạn 2: ROS adapter và action server

1. Triển khai subscriber `/map`, `/depth/scan`, `/odom` và TF listener.
2. Triển khai publisher `/cmd_vel_nav` và `/plan`.
3. Triển khai `/navigate_to_pose` action server.
4. Thêm cancel, shutdown và stale-sensor handling.
5. Test bằng ROS message giả, không nối motor.

Điều kiện hoàn tất:

- Action bị reject/abort đúng khi thiếu map/TF/sensor.
- Cancel tạo zero velocity trong tối đa một command period.
- Không publish non-zero khi action kết thúc.
- `/plan` có frame `map` và được clear khi mission kết thúc.

### Giai đoạn 3: Tích hợp launch

1. Khởi chạy `navigation_node.py` từ `web.launch.py`.
2. Cập nhật action interface, install/dependency và build checks.
3. Build workspace trong container hiện tại.
4. Kiểm tra chỉ có một action server `/navigate_to_pose`.

Điều kiện hoàn tất:

- Launch chỉ khởi động navigation process.
- `ros2 action list` có đúng một `/navigate_to_pose`.
- `/cmd_vel_nav` chỉ có một publisher navigation.

### Giai đoạn 4: Bench test không tải

1. Nâng bánh xe hoặc ngắt motor power.
2. Khởi động navigation backend và kiểm tra map/TF/sensor readiness.
3. Gửi goal, quan sát `/plan`, action feedback và `/cmd_vel_nav`.
4. Test Cancel, đóng browser, mất rosbridge, mất `/depth/scan`, mất `/odom` và dừng
   node bằng `Ctrl+C`.

Điều kiện hoàn tất:

- Mọi đường lỗi đều đưa command về zero.
- Vận tốc không vượt giới hạn arbiter.
- CPU/memory ổn định; control heartbeat không trễ quá timeout arbiter.

### Giai đoạn 5: Robot test tốc độ thấp

Thứ tự case:

1. Goal thẳng, không obstacle.
2. Goal yêu cầu quay tại chỗ.
3. Goal có static obstacle và cần đường vòng.
4. Obstacle tạm thời xuất hiện trên path.
5. Cancel khi đang chạy thẳng và khi đang quay.
6. Goal blocked, goal ngoài map và goal trong unknown area.
7. Sensor/TF interruption.
8. Nhiều goal liên tiếp sau success/cancel/failure.

Chỉ tăng tốc hoặc tune DWA sau khi các case dừng an toàn đã pass.

## 13. Lệnh kiểm tra dự kiến

Build:

```bash
./shell.sh build
```

Chạy navigation backend:

```bash
./shell.sh
ros2 launch bringup web.launch.py
```

Kiểm tra graph:

```bash
ros2 action info /navigate_to_pose
ros2 topic info /cmd_vel_nav -v
ros2 topic info /cmd_vel -v
ros2 topic echo /plan --once
ros2 topic hz /depth/scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_footprint
```

Các kiểm tra bổ sung phải xác nhận chỉ có một navigation process và một
action server `/navigate_to_pose`.

## 14. Tiêu chí nghiệm thu

Navigation được đặt làm backend mặc định khi đạt tất cả điều kiện:

- Build sạch trong Docker image hiện tại.
- Unit test thuật toán và ROS adapter pass.
- Web UI gửi goal, hiển thị path, nhận feedback, cancel và trạng thái kết thúc
  mà không phải sửa giao thức frontend.
- Robot đi tới goal trong tolerance cấu hình ở các case cơ bản.
- Không va vào static obstacle trong test map.
- Obstacle LiDAR làm robot dừng hoặc chọn trajectory an toàn.
- Cancel/emergency/sensor timeout/TF failure/shutdown đều tạo zero command.
- Không có hai publisher navigation cạnh tranh.
- CPU trên Raspberry Pi 5 đủ để giữ control loop và command heartbeat ổn định.
- Cập nhật README và tài liệu topic.
- Chạy soak test tối thiểu 30 phút với nhiều goal/cancel liên tiếp.

## 15. Rủi ro chính

| Rủi ro | Cách giảm thiểu |
|---|---|
| Thuật toán nguồn được tune cho simulator | Dùng config robot thật, giới hạn vận tốc thấp và tune theo từng test case |
| LiDAR thật không có đúng 360 ray | Adapter chấp nhận kích thước scan động và dùng metadata ROS |
| Static map nguồn không tương thích SLAM map | Tạo grid trực tiếp từ `/map`; không copy JSON simulator |
| Dijkstra/inflation tốn CPU khi map lớn | Snapshot map theo goal, benchmark thời gian plan, giới hạn tần suất rebuild và tối ưu distance transform nếu cần |
| Map đổi trong khi mission đang chạy | Giữ snapshot ổn định cho mission hiện tại, dùng map mới ở goal kế tiếp và tránh vật cản tức thời bằng LiDAR |
| TF hoặc sensor mất tạm thời | Freshness timeout, abort action và publish zero |
| Có nhiều navigation node cùng chạy | Launch chỉ tạo một node và graph check trong test |
| Action callback bị block bởi planner | Planner chạy tuần tự trong action loop ở `control_hz`; benchmark map lớn trước khi tăng tốc robot |
