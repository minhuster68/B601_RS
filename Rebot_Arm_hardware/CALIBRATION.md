# Calibration B601-RS

## Trạng thái đã xác minh

Ngày 2026-11-03, chủ robot xác nhận mapping ID 1–6 từ đế đến cổ tay, ID 7 là
kẹp; encoder tăng cùng chiều dương RViz (`direction: 1`). Chủ robot đưa tay
về đúng tư thế URDF q=0 và xác nhận tay được đỡ/cố định.

Đã đặt zero trong motor ID 1–6, gửi save và kiểm tra sau các lần tắt/bật
nguồn do chủ robot thực hiện. Đọc cuối: ID 1 ≈ -0.000170, ID 2 ≈ 0.001747,
ID 3 ≈ -0.000085, ID 4 ≈ 0, ID 5/6 ≈ -0.000038 rad. Sáu khớp dùng
`motor_zero_rad: 0.0`, `joint_zero: 0.0`. Không cần zero lại khi launch.

Motor 7 đã đặt zero ở mốc đóng, gửi save một lần và đọc lại sau khi chủ
robot tắt/bật nguồn: `mechPos ≈ -0.0000767 rad`, startup flag `0x7029=0`
(unsigned), CSP mode 5. Dùng `motor_zero_rad: 0.0`, không cần zero lại khi
launch. Biên bản trước/sau và xác minh nguồn:
`~/.local/share/rebot_arm_rs/calibration/zero_gripper_20261103_153655_097275.json`.
Không enable hoặc phát target chuyển động, không ghi lại zero ID 1–6.

Hệ số kẹp đã tính từ hành trình encoder và độ mở
100 mm được chủ robot đo, khớp với hành trình 50 mm/ngón trong URDF:
`motor_radians_per_joint_unit: 119.2602`. Sau khi xác nhận monitor, chủ robot
đã đổi `calibration_confirmed: true` để thử control;
giới hạn dòng đã chọn theo datasheet theo yêu cầu chủ robot: RS06 (ID 1–3)
14.3 A, RS00 (ID 4–7) 4.7 A. Việc sửa YAML chỉ chuẩn bị cấu hình; giới hạn
được ghi/đọc kiểm tra trong lúc activation của control.

Ngày 2026-11-03, chủ robot đã di chuyển tay thật trong chế độ monitor và
xác nhận mô hình RViz chuyển động khớp với tay thật. Kiểm tra phiên đang
chạy cho thấy plugin phần cứng ở `read_only: true`, dung sai 0.05 rad đã
nạp, joint_state_broadcaster active và MoveIt Current State nhận đủ 6
khớp tay cùng 2 ngón kẹp. Đã kiểm chứng đường encoder → ROS → MoveIt/RViz
với các chuyển động người dùng đã thử. Chưa kiểm chứng giữ torque hoặc
thực thi quỹ đạo từ MoveIt xuống motor.

Lần thử control của chủ robot bị lỗi `SocketCAN send: No buffer space
available` trong activation trên PCAN-USB, `can0 txqueuelen=10`. Controller
manager chưa khởi tạo xong nên dịch vụ list_controllers không sẵn sàng.
Đã sửa pacing/backoff TX có hạn và cách tính chu kỳ write đầu sau startup;
cần chủ robot khởi động lại để xác nhận activation trên phần cứng thật.
Đã kiểm chứng bản sửa trong user/network namespace riêng với Linux vcan:
monitor chỉ đọc; control bật 7 motor giả, MoveIt thực thi arm ready/home
và gripper open/closed, có kiểm tra feedback, mimic, CSP/watchdog và stop
motor giả lúc kết thúc. Kiểm thử này không truy cập `can0` thật.

Bản trước/sau và xác nhận giữ zero nằm trên máy robot tại
`~/.local/share/rebot_arm_rs/calibration/arm_zero_verified_20261103.json`,
trong đó dẫn tới các bản sao trước khi thay đổi từng nhóm motor.

Firmware đọc được qua version response: ID 1–3 có bytes
`00 c4 56 00 06 00 06 07`; ID 4–6 có bytes `00 c4 56 00 00 03 16 07`.
Firmware thực tế từ chối đọc `0x2005` qua type 17, và không trả ACK cho save
type 22. Công cụ lưu `mechanical_offset_rad: null`, ghi rõ save gửi một lần
và không tự retry. Việc giữ zero đã được xác minh bằng đọc sau tắt/bật nguồn,
không suy ra từ ACK hay giá trị RAM ngay sau khi ghi.

## Công cụ zero một lần

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to rebot_arm_rs_hardware
source install/setup.bash

# Chỉ đọc/preview; không stop, zero, save hoặc enable
ros2 run rebot_arm_rs_hardware motor_calibrate.py zero --ids 1,2,3,4,5,6

# Đọc góc, vận tốc, Iq lọc, limit_cur, mode và zero flag
ros2 run rebot_arm_rs_hardware motor_calibrate.py inspect --ids 1,2,3,4,5,6,7
```

Subcommand `zero --apply --backup /new/path.json` thực sự dừng và đặt mốc
tọa độ, rồi gửi save. Chỉ dùng khi từng khớp đã ở tư thế zero URDF và được
đỡ chắc, không có chương trình khác điều khiển. Đây không phải lệnh tự chạy
robot về zero. Không gọi lại cho ID 1–6 chỉ vì khởi động máy/launch mới.

Lệnh dùng type 4 stop, type 18 đặt CSP/startup flag nếu cần, type 6
với byte 0 bằng 1, type 22 với payload `01 02 03 04 05 06 07 08`. Không gửi
enable hoặc target chuyển động. Trước lần ghi đầu tiên, công cụ lưu JSON
trước thay đổi và kiểm tra cửa sổ vị trí không trôi quá 0.002 rad. Feedback
vận tốc thực tế từng có spike trong khi vị trí chỉ biến thiên <0.0003 rad,
nên kiểm tra đứng yên dựa trên nhiều mẫu vị trí thay vì một mẫu vận tốc.

Nếu thiếu ACK lưu, mặc định công cụ dừng và ghi lại tiến độ. Tùy chọn
`--allow-unacknowledged-save` chỉ dành cho firmware đã được kiểm chứng có
lưu dù không trả ACK; nó vẫn không gửi lặp save và luôn đánh dấu cần xác
minh qua tắt/bật nguồn. Save lưu các tham số hiện hành theo cơ chế firmware,
không chỉ riêng vị trí zero. Không thực hiện khi controller đang chạy.

Sau đặt zero, kiểm tra cùng tư thế trước và sau một lần mất điện. Cần kiểm
tra cả phía góc dương/âm và các giới hạn mềm trước khi vận hành. Flag signed
startup `0x7029=1` dùng cho các khớp tay có góc âm; không tự áp dụng lựa chọn
này cho kẹp: hành trình 5.96301 rad của kẹp vượt pi, nên ID 7 dùng
`--startup-range unsigned` (`0x7029=0`). Công cụ từ chối zero ID 7 với
signed hoặc trộn các khớp tay vào lệnh unsigned. Ví dụ preview kẹp:

```bash
ros2 run rebot_arm_rs_hardware motor_calibrate.py zero --ids 7 --startup-range unsigned
```

Chưa kiểm tra tắt/bật nguồn ở các độ mở trung gian hoặc mở hết. Cần kiểm
tra tọa độ ở các vị trí này trước khi vận hành để phát hiện nhầm nhánh
góc quanh 0/2pi; driver không tự unwrap hoặc đổi nhánh target motor.

## Đo hệ số kẹp

Số đo chủ robot cung cấp ngày 2026-11-03, trước khi đặt zero cho ID 7:

| Vị trí | `mechPos` ID 7 (rad) |
| --- | --- |
| Đóng hết | 1.53624 |
| Mở hết | 7.49925 |

Hành trình góc là `7.49925 - 1.53624 = 5.96301 rad` (khoảng 341.66°).
Góc tăng khi mở, phù hợp `direction: 1`. Chủ robot đã đo khoảng cách giữa
hai mặt trong khi mở hết là **100 mm**, phù hợp giới hạn **50 mm mỗi ngón**
trong URDF của hãng. Dùng mốc đóng `q=0` và mở hết `q=0.05 m` của model:

```text
k = (7.49925 - 1.53624) / 0.05 = 119.2602 rad/m
```

Giá trị đã được điền vào `motors.yaml`. Trong hệ tọa độ encoder của phiên
đo này, quan hệ là `q = (mechPos - 1.53624) / 119.2602`: đóng ứng với 0,
giữa hành trình 0.025 m, mở hết 0.05 m mỗi ngón. Đây là quan hệ quy đổi của
hai mốc đã đo; chưa ghi `1.53624` làm offset vào cấu hình vì chủ robot chọn
đặt zero trong motor. Đã đặt zero ID 7 tại mốc đóng, góc đóng gần 0,
góc mở gần 5.96301 rad và hệ số k không đổi. Cần kiểm tra việc giữ tọa độ
đúng sau mất điện trên toàn hành trình trước khi bật điều khiển.

Hệ số cần điền là **rad đầu ra motor trên mỗi mét hành trình một ngón**.
Nếu đo bằng thước cặp độ mở toàn bộ giữa hai mặt kẹp, thay đổi độ mở bằng
hai lần thay đổi hành trình một ngón trong mô hình mimic đối xứng.

1. Chọn vị trí A trong hành trình, ghi góc encoder ID 7 `theta1` và độ mở
   giữa hai mặt kẹp `W1` (mm).
2. Chọn vị trí B khác A, ghi `theta2`, `W2`. Tránh ép vào chặn cơ khí.
3. Tính hệ số, rồi kiểm tra thêm vài vị trí trung gian theo cả chiều mở và
   đóng để phát hiện độ rơ hoặc quan hệ không tuyến tính.

```text
Delta_q = (W2 - W1) / 2000          # mét, hành trình một ngón
k = abs((theta2 - theta1) / Delta_q) # rad/m
```

Ví dụ minh họa: góc tăng từ 1.2 lên 3.2 rad, độ mở tăng từ 20 lên 40 mm:
`Delta_q=0.01 m`, nên `k=200 rad/m`. Đây không phải kết quả đo robot này.

Công cụ tính offline, không mở CAN hoặc sửa cấu hình:

```bash
ros2 run rebot_arm_rs_hardware motor_calibrate.py gripper-scale \
  --theta1 1.2 --theta2 3.2 --gap1-mm 20 --gap2-mm 40
```

Nếu bạn đo trực tiếp chuyển dịch của một ngón thay vì tổng độ mở, dùng
`k=abs(Delta_theta/Delta_s)` và không chia đôi lần nữa. Zero ID 7 phải khớp
với mốc q=0 của kẹp trong model; đặt zero tại độ mở tùy ý sẽ làm model sai
vị trí dù hệ số đo đúng. Không giả định dữ liệu một vòng còn đủ để khôi
phục vị trí sau mất điện nếu toàn hành trình vượt khả năng xác định góc
tuyệt đối khi khởi động của firmware/encoder.

## Dung sai encoder tại giới hạn

Theo yêu cầu chủ robot, `position_limit_tolerance_rad: 0.05` cho phép
phản hồi vượt biên tối đa 0.05 rad đầu ra motor (khoảng 2.86° với khớp
tay 1:1). Kẹp có dung sai tương ứng `0.00041925 m` mỗi ngón, quy đổi
từ cùng ngưỡng góc motor. Đây là sai lệch so với **biên giới hạn**, không
phải khoảng cách tới tư thế zero hoặc target.

Trong mức này, trạng thái xuất cho ROS và target giữ lúc kích hoạt được
giới hạn về biên hợp lệ; encoder gốc được giữ nội bộ, cảnh báo khi clamp,
và tiếp tục dùng cho kiểm tra tracking. Vượt quá ngưỡng (có xét làm tròn
float32 trên CAN) gây lỗi chốt; control dừng các motor driver đã enable,
monitor không phát lệnh điều khiển. Lệnh chuyển động vẫn không được vượt
biên URDF. `max_tracking_error` giữ ý nghĩa và ngưỡng riêng.

Ví dụ joint3 giới hạn dưới 0: `-0.000043 rad` được xuất thành 0;
`-0.051 rad` báo lỗi. Ngưỡng này không xác nhận an toàn cơ khí của việc
vượt biên và không tự sửa nhầm nhánh góc 0/2pi. Không thay đổi zero motor.

## Đọc dòng và chọn giới hạn

`current_limit_a` là ngưỡng phải chọn, không phải một số đo duy nhất. Driver
ghi nó vào `limit_cur` (`0x7018`). Dòng phản hồi đọc từ `iqf` (`0x701A`).
Dòng DC tại nguồn 48 V không tương đương dòng Iq/pha dùng trong giới hạn
motor. Khi tham khảo datasheet, phân biệt đơn vị RMS và peak; không gán
trực tiếp hằng số torque theo Arms vào công thức với dòng peak chưa quy đổi.

Đọc thực tế khi calibration: ID 1–3 đang có `limit_cur=57 A`, ID 4–6 là
`16 A` (ID 7 cũng `16 A`), Iq gần 0 khi motor dừng. Đây là giá trị đang cài trong firmware,
không phải giới hạn đang chọn trong `motors.yaml`.

Chủ robot xác nhận thử ban đầu **không cầm vật**, dùng nguồn tổ ong
**48 V, 12.5 A (600 W danh định)** và dây nối **daisy-chain** qua các motor.
Đây là cùng đường nguồn phân phối đến các motor. Đoạn dây/cổng nguồn đầu
chuỗi mang tổng dòng DC cấp cho các motor phía sau, nên giới hạn dây/cổng
cũng cần được tính khi xác định khả năng cấp điện thực tế. Không tính
`12.5/7 A` để làm giới hạn motor, không cộng các giới hạn Iq rồi so trực
tiếp với 12.5 A DC. Chưa có số đo công suất thật để kết luận nguồn đáp ứng
mọi tư thế/tốc độ. Không có payload vẫn phải giữ trọng lượng chính cánh tay.

Thông số tham khảo từ mục Electrical characteristic trong manual chính hãng:

| Model / ID đã xác nhận | Dòng pha peak tại tải định mức | Dòng pha peak cực đại |
| --- | --- | --- |
| RS06 / 1–3 | 14.3 Apk ±10% | 57.0 Apk ±10% |
| RS00 / 4–7 | 4.7 Apk ±10% | 15.5 Apk ±10% |

Theo yêu cầu chủ robot ngày 2026-11-03, đã dùng cột **dòng pha peak tại tải
định mức** làm trần `current_limit_a`: 14.3 A cho joint1–joint3 và 4.7 A
cho joint4–joint6/gripper_joint1. Manual mô tả giới hạn dòng là maximum
phase current và register `limit_cur` (`0x7018`) dùng A trong chế độ
position/velocity. Không dùng cột dòng pha peak cực đại để đặt cấu hình.

Đây là lựa chọn theo datasheet, chưa phải kết quả đo trên robot. Tải định
mức được hãng thử ở 100 rpm; không suy ra rằng có thể giữ dòng này vô hạn
khi đứng yên, hoặc đã phù hợp lực kẹp và khả năng cấp điện thực tế. Bước
này chỉ sửa cấu hình; driver sẽ ghi/đọc kiểm tra giới hạn khi kích hoạt
control. Không ghi flash, enable hoặc phát lệnh chuyển động trong bước này.

Để chọn giới hạn cho từng khớp cần thông số model/firmware và tải dự kiến,
sau đó kiểm chứng trong điều kiện có đỡ cơ khí/giới hạn thử phù hợp:

- Ghi dòng khi giữ các tư thế chịu tải và khi chạy các chuyển động dự kiến,
  gồm payload, tốc độ và gia tốc thật sự sẽ dùng.
- Kiểm tra torque giữ trọng lực, sai lệch bám vị trí và nhiệt độ theo thời
  gian; dòng quá nhỏ cũng có thể làm khớp không giữ được tải.
- Chọn ngưỡng đủ cho tác vụ và dự phòng đã kiểm chứng, đồng thời nằm trong
  giới hạn continuous/duty-cycle/thermal của motor, driver, dây và nguồn.
  Không dùng mức peak datasheet làm giới hạn liên tục.
- Với kẹp, cần xét thêm lực kẹp cho vật cụ thể; nếu cần biết lực chính xác,
  đo bằng cảm biến lực, không suy ra chỉ từ dòng và ma sát cơ cấu.

Công cụ log chỉ đọc một motor và không enable hoặc phát lệnh chuyển động:

```bash
ros2 run rebot_arm_rs_hardware motor_calibrate.py current-log \
  --ids 2 --seconds 10 --rate 10 --output /tmp/joint2-current.csv
```

CSV gồm thời gian, góc, vận tốc, Iq lọc và giới hạn đang cài. File phải chưa
tồn tại. Khi motor đang dừng/không giữ torque, Iq gần 0 không cho biết mức
cần để nâng/giữ tay. Dòng lọc lấy mẫu 10 Hz cũng không bắt được mọi đỉnh
chuyển tiếp. Giá trị `max_observed_abs_iq_a` được in ra chỉ là quan sát,
không tự động trở thành giới hạn đề xuất. Chỉ một chương trình được phát
lệnh chuyển động; dừng mọi controller trước khi chạy lệnh zero.

Nguồn: [RS00 User Manual 2026-07-13](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/CAN2.0/RS00/RS00User%20Manual260713.pdf),
[RS06 User Manual 2026-07-13](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/CAN2.0/RS06/RS06User%20Manual260713.pdf)
và [định nghĩa register chính thức](https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/protocol.py).
