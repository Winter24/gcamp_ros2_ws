import cv2
import numpy as np

# 1. Đọc ảnh map gốc
input_path = 'small_city.pgm'
output_path = 'small_city_fixed.pgm'
img = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)

if img is None:
    print(f"Không tìm thấy file {input_path}!")
    exit()

h, w = img.shape
mid_x = w // 2  # Vị trí cắt đôi trục giữa (pixel 250)

# 2. Cắt ảnh thành 2 nửa trái / phải
left_half = img[:, :mid_x]
right_half = img[:, mid_x:]

# 3. Tính toán số pixel mới theo đúng tỷ lệ Gazebo (Tổng vẫn bằng 500 pixel)
# Vế trái chiếm 30m/90m -> 1/3 map -> 167 pixel
# Vế phải chiếm 60m/90m -> 2/3 map -> 333 pixel
new_left_w = 167
new_right_w = 333

# 4. Tiến hành co dãn từng phần
left_resized = cv2.resize(left_half, (new_left_w, h), interpolation=cv2.INTER_AREA)
right_resized = cv2.resize(right_half, (new_right_w, h), interpolation=cv2.INTER_CUBIC)

# 5. Khâu 2 nửa lại với nhau thành bức ảnh mới
fixed_img = np.hstack((left_resized, right_resized))

# 6. Lưu ảnh map mới
cv2.imwrite(output_path, fixed_img)
print("=== ĐÃ NẮN BẢN ĐỒ THÀNH CÔNG! ===")
print(f"Ảnh mới '{output_path}' đã được đồng bộ cấu hình 30m-60m với Gazebo.")