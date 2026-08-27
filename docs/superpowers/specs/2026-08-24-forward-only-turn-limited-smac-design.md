# Forward-Only Turn-Limited Smac Design

## Mục tiêu

Mở rộng `TurnLimitedSmacPlanner` để xe tuyệt đối không đi lùi, path phải xuất phát theo hướng đầu xe, giữ đúng yaw goal và tiếp tục tuân thủ giới hạn chuỗi rẽ liên tục dưới 150 độ.

## Ràng buộc

- Planner đang hoạt động là Smac Hybrid A*, không phải Smac 2D.
- Chỉ dùng motion model `DUBIN` cho custom planner.
- Không có reverse primitive, reverse analytic segment hoặc reverse cusp trong path thành công.
- Không tự fallback sang `REEDS_SHEPP` hay planner gốc.
- Goal phải đạt cả vị trí và yaw theo tolerance hiện tại; nếu không thể thì trả path rỗng.
- Giữ `SmacPlannerHybrid` gốc trong YAML chỉ để rollback thủ công.
- Giữ nguyên giới hạn quay liên tục 150 độ và reset sau 3 m gần thẳng.

## Thay đổi cấu hình

`TurnLimitedSmacPlanner.motion_model_for_search` đổi từ `REEDS_SHEPP` sang `DUBIN`.

Plugin validate lúc lifecycle configure rằng motion model của custom planner là `DUBIN`. Giá trị khác khiến configure thất bại với thông báo rõ tham số lỗi. Dynamic parameter callback cũng từ chối thay đổi sang model khác.

## Forward-path audit

Raw path do A* tạo và path sau smoothing đều được audit. Với mỗi cặp pose liên tiếp:

```text
movement = next.position - current.position
heading = (cos(current.yaw), sin(current.yaw))
forward_projection = dot(movement, heading)
```

- Bỏ qua đoạn có độ dài dưới epsilon số học.
- `forward_projection` âm ngoài tolerance là reverse và làm path không hợp lệ.
- Không suy ra forward/reverse chỉ từ thay đổi yaw.
- Raw path có reverse bị từ chối.
- Smoothed path có reverse hoặc vi phạm giới hạn quay thì fallback về raw path, nhưng chỉ khi raw path đã qua cả forward audit và turn-limit audit.
- Nếu không có path hợp lệ, trả path rỗng; xe dừng và goal không được thực thi.

## Goal yaw

Không sửa yaw goal và không chuyển sang position-only planning. DUBIN phải tìm được một tuyến chỉ tiến đến đúng yaw goal. Nếu không thể vì đường hẹp, footprint, turning radius hoặc giới hạn 150 độ, planner từ chối goal.

## Kiểm thử

- Cấu hình `DUBIN` hợp lệ; `REEDS_SHEPP` bị custom planner từ chối.
- Primitive expansion và analytic expansion của custom planner không sinh reverse.
- Audit chấp nhận path thẳng, rẽ trái và rẽ phải chỉ tiến.
- Audit từ chối path có đúng một đoạn lùi.
- Path smoothed sinh reverse fallback về raw path chỉ tiến.
- Goal nằm phía sau nhưng có vòng đường chỉ tiến: planner chọn đường vòng.
- Goal nằm phía sau và không có tuyến chỉ tiến đạt đúng yaw: planner trả path rỗng.
- Planner gốc vẫn có trong cấu hình nhưng không được gọi tự động.

## Tiêu chí hoàn thành

- Mọi path custom planner trả thành công đều chỉ có chuyển động tiến.
- Pose đầu path tiếp tục theo hướng hiện tại của xe, không khởi đầu bằng reverse.
- Không thay đổi yaw goal để ép planning thành công.
- Toàn bộ test planner hiện tại và test forward-only mới đều pass.
- Không commit hoặc push.
