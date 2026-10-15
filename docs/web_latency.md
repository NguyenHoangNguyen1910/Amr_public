• Có. Sau khi đọc pipeline hiện tại, mình thấy 4 thay đổi có impact lớn nhất, theo thứ tự ưu tiên:

  Ưu tiên    Thay đổi                                                       Impact      Công sức
━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━
  1          Không truyền camera qua rosbridge JSON/base64                 Rất lớn       Thấp–TB
─────────  ────────────────────────────────────────────────  ─────────────────────  ────────────
  2          Không gửi/rebuild toàn bộ map định kỳ             Rất lớn với map lớn    Trung bình
─────────  ────────────────────────────────────────────────  ─────────────────────  ────────────
  3          Đổi LiDAR từ nav_msgs/Path sang mảng float32[]                    Lớn    Trung bình
─────────  ────────────────────────────────────────────────  ─────────────────────  ────────────
  4          Render theo canvas layer/dirty flag                    Lớn về độ mượt    Trung bình

### 1. Tách camera khỏi rosbridge — đã triển khai

Trước thay đổi, camera JPEG được bọc base64 trong JSON rồi gán thành data
URL tại web/index.html:581, đồng thời dùng chung `bulkRos` với map và scan.

Đã triển khai:

- Frontend không còn subscribe camera qua rosbridge.
- `camera_mjpeg.py` phục vụ JPEG nhị phân qua `/camera.ws` với `permessage-deflate`; endpoint MJPEG cũ vẫn dùng được.
- Camera không còn đi chung WebSocket với map, LiDAR và navigation.

### 2. Chỉ gửi map khi thay đổi — đã triển khai

`web_bridge.py` hiện nhận `/map`, so sánh metadata và `data[]` với map gần
nhất, rồi chỉ publish lần đầu hoặc khi nội dung thực sự đổi qua
`/web/map`. Frontend subscribe topic này với throttle `0`, nên không còn nhận
lại full OccupancyGrid giống hệt chỉ vì SLAM publish định kỳ.

Canvas map vẫn chỉ dựng `ImageData` khi nhận một map mới. Delta map/binary
encoding chưa cần thêm ở bước này; chúng chỉ có lợi nếu payload full map vẫn
trở thành nút thắt sau khi đo thực tế.

### 3. Đóng gói LiDAR thành dữ liệu 2D gọn — đã triển khai

Backend hiện biến tối đa 180 điểm thành mảng `float32[]` gồm các cặp `x, y`
trong `ros/src/bringup/scripts/web_bridge.py`.

UI thực tế chỉ dùng x và y.

Benchmark mô phỏng với 180 điểm:

- nav_msgs/Path dạng JSON: khoảng 29 KB/frame
- mảng [x0,y0,x1,y1,...]: khoảng 1.7 KB/frame
- binary float32: 1.44 KB/frame

Tức payload có thể giảm khoảng 17 lần trước cả khi dùng binary.

Topic đã triển khai:

/web/scan_points
  header
  uint32 seq
  float32[] points  # x0, y0, x1, y1...

Scan đã được chuyển từ `bulkRos` sang `realtimeRos`.

Backend transform scan sang frame `map` bằng TF tại đúng timestamp và đã bỏ
fallback TF “latest”; nếu không có TF phù hợp, scan được drop.

### 4. Tách render thành các layer độc lập — đã triển khai

Frontend hiện dùng bốn canvas chồng nhau trong `web/index.html`:

mapCanvas      chỉ cập nhật khi map đổi
scanCanvas     chỉ cập nhật khi scan đổi
pathCanvas     chỉ cập nhật khi plan đổi
overlayCanvas  robot và goal

Mỗi callback chỉ đánh dấu layer tương ứng là dirty. `requestAnimationFrame()`
coalesce các update gần nhau, sau đó chỉ redraw layer dirty; pose 20–30 Hz chỉ
clear/vẽ lại overlay, không đụng map, scan và path.

Đồng thời:

- Cache `sin`, `cos`, `resolution` và origin trong `mapTransform` thay vì tính
  lại trong mọi `worldToCanvas()`/`canvasToWorld()`.
- Các layer giữ cùng kích thước pixel với map, còn CSS chỉ scale cả stack một lần.
- Giữ `requestAnimationFrame()` vì cơ chế coalesce giúp tránh redraw trùng khi
  nhiều topic cập nhật cùng lúc.

### Reconnect safety — đã triển khai

Đã sửa trong `web/index.html`:

- Không còn `RosBridge.outbox`; message publish bị drop khi WebSocket disconnected.
- Không replay `cmd_vel` hoặc goal cũ sau reconnect.
- Clear keyboard state khi socket đóng hoặc lỗi.
- Gửi zero velocity ngay sau reconnect trước khi cho phép điều khiển lại.
- Vô hiệu hóa GO/Set Goal khi WebSocket chưa sẵn sàng.

Đây không chỉ là tối ưu memory/latency mà còn là yêu cầu an toàn khi mất kết nối.

Thứ tự triển khai mình khuyên:

1. ✅ Bỏ camera khỏi rosbridge và chuyển sang MJPEG.
2. ✅ Sửa reconnect safety để không replay command.
3. ✅ Chuyển scan sang realtime socket và packed XY.
4. ✅ Tách canvas layer.
5. ✅ Tối ưu full-map thành update-on-change.
6. Sau cùng mới tăng scan từ 5 Hz lên 10–15 Hz và benchmark lại.
