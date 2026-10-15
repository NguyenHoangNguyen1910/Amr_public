# Robot thật và robot ảo

## Kiến trúc

Web UI, navigation và bộ phân xử vận tốc không phụ thuộc backend:

```text
web UI -> navigation/manual -> cmd_vel_arbiter -> /cmd_vel
                                                   |
                         +-------------------------+------------------------+
                         |                                                  |
                 hardware adapter                                  simulation adapter
                 micro-ROS + EKF                            2D kinematics + ray casting
                         |                                                  |
                  /odom /scan /TF                                   /odom /scan /TF
                         +-------------------------+------------------------+
                                                   |
                                         web bridge + navigation
```

Chỉ một nhánh adapter được launch. Contract chung là:

| Hướng | Topic/TF | Ý nghĩa |
|---|---|---|
| Input | `/cmd_vel` | Lệnh cuối cùng sau giới hạn và watchdog |
| Output | `/odom` | Pose/tốc độ odometry |
| Output | `/scan` | LiDAR |
| Output | `odom -> base_footprint` | Pose robot |
| Environment | `/map` và `map -> odom` | Occupancy map và hệ tọa độ toàn cục |

Simulator chạy headless trong ROS container. Nó không cần Gazebo, GPU, mô hình
3D, FastAPI hay UI riêng. `web/index.html`, planner và các topic web hiện tại
được dùng cho cả hai backend.

## Chạy robot ảo

Build sau khi checkout hoặc thay đổi source:

```bash
./build.sh
ROBOT_BACKEND=simulation ./bootstrap.sh
./shell.sh
```

Nếu container hardware đang chạy, dừng nó và micro-ROS Agent trước khi đổi
backend. Script sẽ từ chối khởi động simulation nếu Agent vẫn nối với xe thật,
để `/cmd_vel` thử nghiệm không thể tới motor thật:

```bash
docker stop yahboom-ros-main yahboom-micro-ros-agent
```

Trong container:

```bash
ros2 launch bringup web.launch.py \
  robot_backend:=simulation \
  use_camera:=false
```

Ở terminal host khác:

```bash
./serve_web.sh
```

Mặc định adapter dùng `bringup/maps/amr_world.json`, là map 2D được lấy từ
`amr_sim`. Có thể đổi vị trí khởi đầu:

```bash
ros2 launch bringup web.launch.py \
  robot_backend:=simulation use_camera:=false \
  sim_initial_x:=-2.0 sim_initial_y:=-0.5 sim_initial_yaw:=0.0
```

Reset robot ảo về pose tùy chọn:

```bash
ros2 service call /simulation/reset bringup/srv/ResetSimulation \
  '{x: 0.0, y: 0.0, yaw: 0.0}'
```

## Lưu map từ môi trường thật

Khi robot thật đang chạy ở `slam_mode:=mapping`, cho robot đi phủ đủ khu vực,
sau đó chạy trên host:

```bash
./save_map.sh warehouse_a
```

Script không ghi đè map cùng tên. Chỉ khi chủ động muốn thay thế bộ map cũ mới
dùng `FORCE=1 ./save_map.sh warehouse_a`.

Kết quả trong `maps/`:

- `warehouse_a.yaml` + `warehouse_a.pgm`: occupancy map chuẩn ROS, dùng cho
  simulator và hiển thị web;
- `warehouse_a.posegraph` + `warehouse_a.data`: graph và scan SLAM, dùng để
  localize robot thật khi quay lại môi trường đó.

Thư mục `maps/` được mount vào container tại `/workspace/maps`.

## Dùng map thật cho robot ảo

Không cần dựng lại map 3D. Adapter đọc trực tiếp YAML/PGM, dùng ô occupied làm
collision và ray-cast LiDAR:

```bash
ros2 launch bringup web.launch.py \
  robot_backend:=simulation use_camera:=false \
  sim_map:=/workspace/maps/warehouse_a.yaml \
  sim_initial_x:=1.0 sim_initial_y:=2.0 sim_initial_yaw:=0.0
```

Pose đầu phải nằm trong vùng free và cách vật cản ít nhất bán kính robot. Nếu
không, adapter dừng ngay với lỗi rõ ràng thay vì đặt robot xuyên tường.

Smoke test gồm 10 cặp start→goal cố định trên map `warehouse_a`. Mỗi trường
hợp reset robot về đúng start `(x, y, yaw)`, sau đó gửi một goal
`(x, y, yaw)`:

```bash
./smoke_simulation.sh
```

Smoke test tự chuyển arbiter sang mode `nav`, reset simulator trước mỗi case và
trả lại mode `manual` khi kết thúc. Map mặc định là
`/workspace/maps/warehouse_a.yaml`; có thể đổi bằng `SMOKE_MAP`.

## Dùng lại map cho robot thật

```bash
ros2 launch bringup web.launch.py \
  robot_backend:=hardware \
  slam_mode:=localization \
  slam_posegraph:=/workspace/maps/warehouse_a
```

Robot thật vẫn dùng LiDAR/odometry thực. Pose graph chỉ thay chế độ
`slam_toolbox` từ tạo map sang localization; UI và navigation không đổi. Với
cấu hình `map_start_at_dock`, đặt robot gần pose bắt đầu lúc tạo map trước khi
launch localization.

## Phạm vi mô phỏng

2D adapter mô phỏng differential drive, giới hạn tốc độ, command watchdog,
collision theo bán kính robot, odometry và LiDAR 360 tia. Nó phù hợp để test
web, ROS graph, global/local planner và luồng goal mà không có xe.

Nó chưa mô phỏng trượt bánh, nhiễu encoder/IMU, vật cản động hay camera. Nếu sau
này cần kiểm thử động lực học chi tiết, giữ nguyên contract topic ở trên và thay
adapter 2D bằng một Gazebo adapter; không cần đổi web/navigation.
