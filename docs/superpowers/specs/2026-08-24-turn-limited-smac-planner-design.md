# Turn-Limited Smac Planner Design

## Mục tiêu

Tạo một Nav2 global-planner dành cho map `small_city`, ưu tiên đường đi tiến và tuyệt đối không tạo một chuỗi rẽ liên tục làm xe đổi hướng từ 150 độ trở lên. Planner phải tiếp tục tìm đường vòng hợp lệ thay vì chỉ kiểm tra rồi từ chối path do Smac gốc trả về.

## Phạm vi

- Tạo plugin planner C++ riêng trong workspace, không sửa package Nav2 tại `/opt/ros/humble`.
- Dựa trên Smac Hybrid A* của ROS 2 Humble và giữ nguyên footprint, turning radius, costmap, smoothing cùng các penalty hiện tại nếu không được ghi đè rõ ràng.
- Chỉ kích hoạt planner mới cho mô phỏng `small_city`.
- Giữ `SmacPlannerHybrid` gốc trong cấu hình để rollback bằng cách đổi planner ID.
- Không tự fallback sang Smac gốc khi không tìm được đường trong giới hạn quay.

## Lý do cần trạng thái mở rộng

Smac Hybrid A* hiện định danh trạng thái bằng `(x, y, yaw)`. Chi phí phụ thuộc lịch sử như tổng góc rẽ liên tục không thể được cài đúng bằng cách chỉ thêm penalty khi mở rộng node: hai đường đến cùng `(x, y, yaw)` nhưng có lịch sử rẽ khác nhau sẽ bị gộp.

Planner mới mở rộng khóa trạng thái thành:

```text
(x, y, yaw, turn_direction, turn_bucket, straight_bucket)
```

Nhờ đó, một trạng thái đã gần giới hạn quay không làm mất một trạng thái khác có cùng pose nhưng lịch sử rẽ an toàn hơn.

## Kiến trúc

Tạo package `gcamp_turn_limited_smac` với các thành phần:

1. `TurnLimitedSmacPlannerHybrid`: triển khai `nav2_core::GlobalPlanner`, quản lý lifecycle, tham số, costmap, collision checker, smoother và xuất `nav_msgs/Path`.
2. `TurnLimitedAStar`: Hybrid A* dùng trạng thái mở rộng và luật loại nhánh theo góc rẽ.
3. `TurnState`: lưu hướng rẽ hiện tại, góc rẽ tích lũy đã lượng tử hóa và quãng đường gần thẳng.
4. Pluginlib manifest: đăng ký `gcamp_turn_limited_smac/TurnLimitedSmacPlannerHybrid`.

Không sửa hoặc ghi đè thư viện Nav2 cài đặt hệ thống.

## Luật chuyển trạng thái

Mỗi motion primitive được phân loại theo thay đổi yaw:

- Gần thẳng nếu `abs(delta_yaw) <= straight_yaw_tolerance_deg`. Trước khi so
  sánh, `delta_yaw` được chuẩn hóa và lượng tử ở mức một microdegree để primitive
  danh nghĩa đúng 5 độ không đổi phân loại do sai số float của angle bin.
- Rẽ trái nếu `delta_yaw` dương ngoài tolerance.
- Rẽ phải nếu `delta_yaw` âm ngoài tolerance.

Quy tắc cập nhật:

- Rẽ tiếp cùng chiều: cộng `abs(delta_yaw)` vào tổng góc liên tục.
- Đổi chiều rẽ: bắt đầu một chuỗi rẽ mới từ primitive hiện tại.
- Gần thẳng: tăng quãng đường thẳng tích lũy.
- Đi gần thẳng đủ 3 m: reset hướng rẽ và tổng góc rẽ về 0.
- Một đoạn thẳng ngắn hơn 3 m không reset chuỗi rẽ.
- Chuyển sang số lùi không tự reset chuỗi rẽ và không được dùng để lách giới
  hạn. Dấu tích lũy luôn là dấu thay đổi yaw; số lùi không đảo dấu này.
- Nếu tổng góc rẽ mới đạt hoặc vượt 150 độ, successor bị loại trước khi đưa vào open set.

## Hàm chi phí

Ngoài chi phí chuẩn của Smac:

- `0 <= turn < 90 deg`: không có penalty bổ sung.
- `90 <= turn < 120 deg`: penalty nhẹ.
- `120 <= turn < 150 deg`: penalty mạnh.
- `turn >= 150 deg`: cấm tuyệt đối.

Penalty được áp dụng tăng dần trong từng vùng để tránh bước nhảy chi phí không cần thiết. Các hệ số cụ thể là tham số ROS và sẽ được chọn bằng kiểm thử trên `small_city` mà không thay đổi ngưỡng cấm 150 độ.

`reverse_penalty`, `change_penalty`, `non_straight_penalty`, `cost_penalty` và collision cost tiếp tục được áp dụng như cấu hình Smac hiện tại.

## Tham số ROS

```yaml
TurnLimitedSmacPlanner:
  plugin: "gcamp_turn_limited_smac/TurnLimitedSmacPlannerHybrid"
  max_continuous_turn_deg: 150.0
  straight_reset_distance: 3.0
  straight_yaw_tolerance_deg: 5.0
  light_penalty_start_deg: 90.0
  strong_penalty_start_deg: 120.0
  light_turn_penalty: 1.0
  strong_turn_penalty: 5.0
```

Planner cũng nhận các tham số Smac Hybrid hiện có, gồm motion model, angle bins, minimum turning radius, iteration/time limits, analytic expansion, smoother và các penalty chuẩn.

Ba tham số chi phối kích thước khóa trạng thái hiện là invariant cố định và
mọi giá trị khác bị từ chối lúc configure:

```text
max_continuous_turn_deg == 150.0
straight_reset_distance == 3.0
straight_yaw_tolerance_deg == 5.0
0 <= light_start < strong_start < 150.0
0 <= light_turn_penalty <= strong_turn_penalty
```

Hai ngưỡng bắt đầu penalty và hai trọng số penalty vẫn cấu hình được trong
chuỗi quan hệ trên. Bảy tham số turn-limit là configure-only.

Sai cấu hình khiến lifecycle configure thất bại với thông báo chỉ rõ tham số lỗi.

## Tích hợp

- Thêm plugin mới vào `planner_server.planner_plugins`.
- `hybrid_pure_pursuit.py` yêu cầu planner ID `TurnLimitedSmacPlanner` khi chạy `small_city`.
- Giữ nguyên mục cấu hình `SmacPlannerHybrid` và không xóa plugin cũ.
- Không thay đổi local controller/pure pursuit trong phạm vi công việc này.

## Xử lý lỗi

- Nếu mọi tuyến đều vi phạm giới hạn 150 độ, planner trả path rỗng và ghi log `NO_PATH_WITHIN_TURN_LIMIT`.
- Xe nhận lệnh dừng; goal không được thực thi.
- Không fallback sang planner cho phép U-turn.
- Cancellation và goal mới phải được kiểm tra trong vòng tìm kiếm để một yêu cầu planning dài không khóa yêu cầu tiếp theo.
- Giữ nguyên cơ chế lỗi collision, start/goal ngoài costmap và timeout của Nav2.
- Path raw được audit sau khi đổi sang world coordinates. Path sau smoothing
  được audit lần nữa với cùng luật yaw/cusp/reverse/reset; nếu smoothing làm
  path vi phạm, planner trả lại path raw đã audit thay vì trả path vi phạm.

## Kiểm thử

### Unit test

- Chuỗi rẽ 149 độ hợp lệ; 150 độ và lớn hơn bị loại.
- Rẽ trái rồi rẽ phải bắt đầu chuỗi mới.
- Đoạn thẳng 2.9 m không reset; 3.0 m reset.
- Đi lùi không reset lịch sử rẽ.
- Hai trạng thái cùng `(x, y, yaw)` nhưng khác turn bucket tồn tại độc lập.
- Validate toàn bộ quan hệ giữa các tham số ngưỡng.

### Integration test

- Đi thẳng và rẽ 90 độ trên `small_city` vẫn trả path.
- Khi tuyến ngắn nhất yêu cầu U-turn, planner tìm tuyến vòng không vượt 150 độ.
- Khi không có tuyến hợp lệ, planner trả lỗi và xe đứng yên.
- Goal mới hủy planning cũ và được xử lý bình thường.
- Planner ID có thể đổi lại `SmacPlannerHybrid` mà không sửa code.

### Đánh giá runtime

Ghi lại cho các tình huống kiểm thử:

- Planning time.
- Số node mở rộng.
- Độ dài path.
- Góc rẽ liên tục lớn nhất.
- Kết quả success, timeout hoặc `NO_PATH_WITHIN_TURN_LIMIT`.

So sánh với Smac gốc để phát hiện mức tăng bộ nhớ/thời gian do trạng thái mở rộng. Không nới ngưỡng 150 độ để đạt chỉ tiêu hiệu năng.

## Tiêu chí hoàn thành

- Không path thành công nào chứa chuỗi rẽ liên tục từ 150 độ trở lên theo đúng luật reset 3 m.
- Planner tìm được đường vòng trong ca kiểm thử mà Smac gốc chọn U-turn, nếu đường vòng đó tồn tại.
- Các path thông thường đi thẳng/rẽ trái/rẽ phải không bị từ chối.
- Không làm thay đổi hành vi của Smac planner gốc.
- Package build được trên ROS 2 Humble và các test mới/cũ đều đạt.
