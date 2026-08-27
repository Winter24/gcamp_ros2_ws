# Small City Lane-Graph Planner Design

## Mục tiêu

Thay global-planning đang hoạt động trong Gazebo `small_city` bằng một planner
kiểu CARLA: tìm tuyến trên đồ thị đường cố định, ưu tiên đi thẳng, cho phép rẽ
trái/phải, tuyệt đối không lùi và không tạo U-turn trực tiếp. Planner chạy gần
tim toàn bộ mặt đường để tránh các xe đỗ cố định ở hai bên, tự quyết định yaw
cuối theo hướng tuyến và xuất `nav_msgs/Path` cho controller hiện tại.

SMAC Hybrid vẫn được giữ làm rollback thủ công và không bị xóa.

## Phạm vi bản đồ

Phiên bản đầu chỉ hỗ trợ world Gazebo `small_city_sdc_prius.world` và occupancy
map `small_city_map.pgm`:

- Đường dọc có tim tại `x = -45, -15, 45, 110, 120`.
- Đường ngang có tim tại `y = -45, 0, 45`.
- Đường dọc rộng `7.4 m`; đường ngang rộng `8.4 m`.
- Các xe `prius_parked` là vật cản tĩnh, vị trí không thay đổi trong một lần
  chạy và được mô tả trong world.

Không tổng quát hóa sang world khác trong phiên bản này.

## Kiến trúc ROS 2

Tạo executable Python `small_city_route_planner.py`. Node cung cấp
`nav2_msgs/action/ComputePathToPose` trên action riêng
`/compute_lane_path_to_pose`; không chiếm `/compute_path_to_pose` của Nav2.

Đầu vào:

- Start từ request nếu `use_start=true`; nếu không, lấy TF `map -> chassis` và
  đổi yaw chassis sang trục tiến thực của Prius giống controller hiện tại.
- Goal position từ action request. Orientation do người dùng gửi không ràng
  buộc kết quả.
- Dữ liệu occupancy từ `small_city_map.pgm` và metadata `small_city.yaml`.
- Danh sách xe đỗ cố định từ một file cấu hình lane graph được kiểm thử, không
  parse SDF trong vòng planning.

Đầu ra:

- Action result chứa `nav_msgs/Path` trong frame `map`.
- Publish cùng path lên `/plan` cho HMI.
- Path rỗng và diagnostic rõ ràng nếu snap, A*, tạo hình học hoặc kiểm tra
  footprint thất bại.

`hybrid_pure_pursuit.py` nhận parameter `global_planner_mode`:

- `lane_graph`: gọi `/compute_lane_path_to_pose`.
- `smac`: gọi Nav2 `/compute_path_to_pose` với planner ID
  `SmacPlannerHybrid`.

Giá trị mặc định cho simulation `small_city` là `lane_graph`. Cancellation và
request-ID stale-callback guard hiện tại phải hoạt động ở cả hai mode.

## Mô hình lane graph

Graph được lưu trong `config/small_city_lane_graph.yaml`. Node là các điểm nối
tim đường ở giao lộ và đầu mút hợp lệ. Edge là đoạn corridor có hướng. Hai
chiều của một con đường là hai edge khác nhau nhưng có thể dùng chung tim mặt
đường vì simulation không có xe chuyển động khác.

Mỗi edge chứa:

- ID ổn định.
- Polyline tim đường.
- Hướng di chuyển.
- Danh sách successor hợp lệ.
- Chiều dài và giới hạn corridor.

State tìm kiếm là `(edge_id, travel_direction)`. Transition được phân loại bằng
góc có dấu giữa heading vào và heading ra:

- `abs(delta_yaw) <= 30 deg`: đi thẳng.
- `30 deg < abs(delta_yaw) < 150 deg`: rẽ trái hoặc phải.
- `abs(delta_yaw) >= 150 deg`: cấm tuyệt đối.

Không tồn tại reverse primitive. Edge kế tiếp phải bắt đầu tại đầu ra của edge
hiện tại. A* dùng chiều dài Euclidean làm heuristic; chi phí gồm chiều dài edge
và một turn penalty nhỏ. U-turn không được biểu diễn bằng chi phí lớn mà bị
loại khỏi tập successor.

## Snap start và goal

Start được snap lên các edge gần nhất bằng:

```text
score = lateral_distance + heading_weight * abs(wrapped_heading_error)
```

Edge có heading error lớn hơn `90 deg` không được chọn cho start. Điều này buộc
path khởi hành theo đầu xe.

Goal chỉ dùng position. Planner chọn edge gần nhất có điểm snap nằm trong giới
hạn mặt đường. Yaw cuối bằng tangent của edge cuối tại điểm goal đã snap.

Nếu start/goal cách mọi corridor quá ngưỡng cấu hình thì action abort và trả
path rỗng. Planner không âm thầm dùng yaw goal của HMI và không fallback sang
SMAC trong cùng request.

## Tránh xe đỗ và lề

Sau khi A* chọn route theo edge, planner thử các lateral offset trong:

```text
[-0.75, -0.50, -0.25, 0.00, 0.25, 0.50, 0.75] m
```

Offset áp dụng trong corridor, không vượt khỏi phần đường sau khi trừ margin
footprint. Candidate bị loại nếu footprint mẫu giao:

- Occupied cell trong `small_city_map.pgm`.
- Bounding box mở rộng của xe đỗ cố định.
- Biên mặt đường sau margin.

Trong các candidate hợp lệ, chọn candidate tối đa hóa clearance nhỏ nhất;
chiều dài là tie-breaker. Không đổi offset đột ngột: chuyển offset bằng đoạn
nội suy liên tục có giới hạn độ cong.

Vì xe đỗ nằm khoảng `2.55 m` khỏi tim đường, offset `0` được kỳ vọng là lựa chọn
phổ biến. Offset search là lớp an toàn, không phải cơ chế vượt xe động.

## Hình học giao lộ

Polyline graph chỉ quyết định topology. Path điều khiển được xây riêng:

- Đoạn thẳng bám corridor đã chọn.
- Giao lộ dùng cung tròn tiếp tuyến hoặc cubic Bezier được kiểm tra độ cong.
- Bán kính cong tối thiểu `3.7 m`.
- Waypoint spacing mục tiêu `0.25 m`.
- Orientation của mỗi pose là tangent theo chiều chuyển động.
- Mọi segment có projection không âm theo yaw pose hiện tại.

Nếu không thể tạo corner thỏa bán kính và footprint, transition tương ứng bị
đánh dấu không khả thi và A* chạy lại mà không dùng transition đó. Planner giới
hạn số lần replan hình học bằng số transition hữu hạn trong graph.

## Kiểm tra an toàn cuối

Trước khi trả path, planner audit toàn bộ kết quả:

- Tất cả số hữu hạn.
- Khoảng cách giữa waypoint không tạo bước nhảy bất thường.
- Forward projection không âm.
- Không có thay đổi heading cục bộ `>= 150 deg`.
- Độ cong không vượt `1 / 3.7 m` ngoài tolerance số học.
- Footprint không giao occupied cell, xe đỗ hoặc biên corridor.
- Pose cuối nằm tại goal đã snap và yaw trùng tangent edge cuối.

Audit thất bại thì trả path rỗng; không trả path một phần.

## Xử lý lỗi và log

Mỗi request có ID và log các bước:

```text
LANE_PLAN_REQUEST
LANE_SNAP_START
LANE_SNAP_GOAL
LANE_ASTAR_RESULT
LANE_GEOMETRY_RESULT
LANE_CLEARANCE_RESULT
LANE_PLAN_RESULT
```

Lý do thất bại ổn định gồm:

- `START_OUTSIDE_ROAD`
- `GOAL_OUTSIDE_ROAD`
- `NO_GRAPH_ROUTE`
- `NO_VALID_CORNER`
- `NO_CLEAR_CORRIDOR`
- `FINAL_PATH_AUDIT_FAILED`
- `CANCELED`

Goal mới hủy request đang chạy; callback/result cũ không được phép thay path
mới.

## Kiểm thử

Unit tests:

- Parse và validate graph/config.
- Snap start ưu tiên edge cùng yaw đầu xe.
- Goal yaw lấy từ tangent, không lấy quaternion HMI.
- A* đi thẳng, rẽ trái, rẽ phải.
- Transition `>= 150 deg` không tồn tại.
- Goal phía sau phải đi vòng qua graph.
- Offset scoring ưu tiên clearance lớn nhất.
- Corner thỏa bán kính `3.7 m`.
- Final audit từ chối reverse, collision và curvature sai.

Integration tests:

- Action cancellation và stale result.
- Publish `/plan` đúng frame và orientation.
- Controller chọn đúng action theo `global_planner_mode`.
- Mode `smac` vẫn hoạt động làm rollback.

Manual `small_city` scenarios:

- Đi thẳng qua từng nhóm xe đỗ.
- Rẽ trái/phải ở mỗi kiểu giao lộ.
- Goal phía sau xe.
- Goal gần lề và gần xe đỗ.
- Goal mới trong khi xe đang chạy.
- Tuyến bị chặn hoàn toàn phải fail closed.

## Ngoài phạm vi

- Xe/vật cản chuyển động.
- Đèn giao thông và nhường đường.
- Tự động sinh graph cho world Gazebo bất kỳ.
- Chuyển lane để vượt xe động.
- Tự động fallback sang SMAC.
- Thay controller Pure Pursuit hiện tại.

## Rollback

Không xóa cấu hình hay package Nav2 Smac. Rollback bằng cách đặt:

```yaml
global_planner_mode: smac
```

và restart simulation. Không cần sửa source hoặc build lại khi chỉ đổi mode
qua launch parameter/config đã cài đặt.
