# B601-RS SocketCAN hardware interface (ROS 2 Humble)

Package `rebot_arm_rs_hardware` dùng `can0` qua Linux SocketCAN. Ba lớp:

- `socket_can`: RX không chặn, TX có pacing/chờ ngắn có hạn, lọc host ID và báo lỗi bus/controller.
- `robstride_protocol`: codec protocol private CAN mở rộng của RobStride.
- `rs_system`: plugin `ros2_control`, đọc encoder, điều khiển CSP và xử lý mimic.

Các chế độ:

| Launch | Hoạt động |
| --- | --- |
| `mode:=monitor` (mặc định) | Chỉ đọc position/velocity; phát `/joint_states`, TF; MoveIt không được thực thi quỹ đạo |
| `mode:=control` | Kích hoạt motor ở CSP, nhận quỹ đạo từ arm/gripper controller |

## Build

Từ thư mục gốc repository:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to rebot_arm_rs_hardware
source install/setup.bash
```

Các dependency phần cứng gồm `hardware_interface`, `pluginlib`, `rclcpp`,
`controller_manager`, `joint_state_broadcaster`, `joint_trajectory_controller`.
Để mở MoveIt cần cài MoveIt 2 như hướng dẫn trong `Rebot_Arm_moveit/README.md`.

## Cấu hình motor

**Mapping motor ID** là bảng nối tên khớp trong URDF với địa chỉ CAN của motor
ở vị trí đó. Chủ robot đã xác nhận mapping và đã điền vào `motors.yaml`:

| Khớp | CAN ID |
| --- | --- |
| `joint1`–`joint6` | `1`–`6`, tương ứng từ đế đến cổ tay |
| `gripper_joint1` | `7` |

Chủ robot đã đối chiếu chuyển động thực tế với RViz và đọc encoder bằng
`can_probe`: encoder tăng cùng chiều dương của khớp, nên cả 7 motor dùng
`direction: 1`.

Ngày 2026-11-03 đã đặt zero trong motor ID 1–6 tại tư thế URDF q=0 và đọc kiểm
tra sau khi chủ robot tắt/bật nguồn: cả sáu giữ zero. Vì vậy các khớp tay dùng
`motor_zero_rad: 0.0`, không lấy các góc quét cũ làm offset phần mềm.

Độ mở hết đã đo 100 mm, khớp URDF 50 mm/ngón; hệ số kẹp đã điền là
`119.2602 rad/m`. ID 7 đã đặt zero tại mốc đóng và kiểm tra giữ zero sau
tắt/bật nguồn, dùng `motor_zero_rad: 0.0` và startup flag unsigned.
Theo lựa chọn của chủ robot ngày 2026-11-03, giới hạn dòng trong YAML dùng
dòng pha peak tại tải định mức trong datasheet: **14.3 A cho RS06 (ID 1–3)**,
**4.7 A cho RS00 (ID 4–7)**. Đây là mức cấu hình theo datasheet, chưa được
kiểm chứng khi giữ/chạy tay thật. Chủ robot đã bật `calibration_confirmed`
để thử control; cờ này cho phép thử nghiệm, không thay thế xác nhận vận hành.
Xem [hướng dẫn calibration](CALIBRATION.md)
về công cụ zero một lần, đo hệ số kẹp và đọc/log dòng.

Sửa hoặc sao chép [`config/motors.yaml`](config/motors.yaml). Monitor chỉ đọc;
control từ chối khởi động trước khi mở CAN nếu `calibration_confirmed: false`
hoặc còn thiếu các tham số bắt buộc. Khi control được kích hoạt, driver ghi
và đọc kiểm tra `limit_cur` theo YAML trước khi enable từng motor.
Không giả định motor ID bằng số thứ tự joint nếu thay đổi robot hoặc dây.

| Trường | Ý nghĩa |
| --- | --- |
| `motor_id` | CAN ID thực tế của motor, khác host và không trùng motor khác |
| `direction` | `1` hoặc `-1`, xác minh theo trục khớp trong URDF |
| `motor_zero_rad` | Giá trị `mechPos` đọc được tại tư thế chuẩn đã biết |
| `joint_zero` | Giá trị khớp URDF tương ứng tư thế chuẩn đó |
| `motor_radians_per_joint_unit` | rad đầu ra motor trên 1 rad khớp tay hoặc 1 mét hành trình MỘT ngón |
| `max_velocity` | rad/s với tay, m/s với kẹp; cũng giới hạn tốc độ planner |
| `max_tracking_error` | Sai lệch tối đa giữa phản hồi khớp và target đã gửi |
| `current_limit_a` | Trần `limit_cur` (A) của motor; hiện chọn theo datasheet, bắt buộc dương khi control |
| `position_min`, `position_max` | Tùy chọn thu hẹp giới hạn URDF |

`model` là ghi chú theo BOM RS06/RS00 của B601-RS, cần kiểm tra với robot của
bạn. Codec đọc position/velocity dạng float nên không phụ thuộc thang đo
phản hồi đóng gói của từng model.

Công thức driver dùng:

```text
q = joint_zero + direction * (mechPos - motor_zero_rad) / motor_radians_per_joint_unit
mechPos_target = motor_zero_rad + direction * motor_radians_per_joint_unit * (q_target - joint_zero)
```

`mechPos` là góc phía tải/đầu ra motor, không phải góc rotor trước hộp số.
Không nhân/chia lại tỷ số hộp số bên trong motor. Nếu có truyền động ngoài,
đưa tỷ số đó vào hệ số quy đổi. Với kẹp, đo hành trình một ngón theo góc
motor để tính hệ số; driver tính `gripper_joint2` từ mimic của ngón chính,
không coi ngón phụ là một motor riêng.

Sau khi mapping, zero, chiều quay, quy đổi và giới hạn đã được xác minh,
đặt `calibration_confirmed: true` để cho phép chọn `mode:=control`.
Driver không tự đặt zero vào firmware hoặc lưu flash.

## Dung sai phản hồi sát giới hạn

`position_limit_tolerance_rad: 0.05` là mức encoder được phép vượt biên
vị trí, tính bằng rad **đầu ra motor**, áp dụng cả monitor và control.
Với các khớp tay tỷ lệ 1:1, tương đương 0.05 rad khớp (khoảng 2.86°).
Với kẹp, tương đương `0.05 / 119.2602 = 0.00041925 m` mỗi ngón.

Trong giới hạn, trạng thái giữ nguyên. Vượt biên trong dung sai (kể cả
đúng ngưỡng, có xét độ chính xác float32 của CAN), driver đưa trạng thái
ROS về biên và cảnh báo khi bắt đầu clamp; target giữ ban đầu cũng nằm
trong biên. Cảnh báo giới hạn tối đa một lần/5 giây mỗi motor để tránh
lặp log khi encoder dao động sát zero. Giá trị encoder gốc vẫn được giữ nội bộ và dùng để kiểm tra
sai lệch bám target. Có thể đọc encoder gốc bằng `can_probe`.

Vượt quá dung sai gây lỗi chốt; ở control, driver gửi stop các motor đã
enable bởi driver và không tự bật lại. Monitor chỉ báo lỗi, không phát
lệnh stop/enable. Dung sai không nới giới hạn URDF hoặc giới hạn lệnh,
không thay thế `max_tracking_error` và không tự unwrap góc 0/2pi.

## Chuẩn bị can0 và đọc encoder

`can0` phải được đưa UP bên ngoài driver, với bitrate khớp motor. Ví dụ cho
motor đang dùng private protocol ở 1 Mbit/s theo tài liệu hãng:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

Các lệnh trên thay đổi cấu hình bus; không chạy đồng thời với chương trình
khác đang điều khiển robot. Driver không tự đổi bitrate/protocol hoặc bật bus.

Công cụ đọc giá trị encoder thô, ví dụ khi bạn muốn kiểm tra các ID 1–7
(đây là ví dụ, không phải mapping đã xác minh cho robot của bạn):

```bash
ros2 run rebot_arm_rs_hardware can_probe --interface can0 --host 253 --ids 1,2,3,4,5,6,7
```

Tool chỉ gửi yêu cầu đọc `mechPos`/`mechVel`; không enable, disable hay đặt zero.

Nếu chưa biết ID, có thể quét các địa chỉ 1–254, bỏ qua host ID:

```bash
ros2 run rebot_arm_rs_hardware can_probe --interface can0 --scan
```

Lệnh quét gửi từng cặp yêu cầu đọc cách nhau tối thiểu 10 ms và in các ID có
phản hồi. Nó không biết motor nào nằm ở khớp nào. Giữ output này để đối chiếu
mapping theo nhãn/cấu hình motor hoặc quy trình nhận diện của nhà sản xuất;
không tự chạy motor chỉ từ thứ tự ID tìm thấy. Sau đó mới xác định chiều
quay/offset theo URDF và hệ số hành trình kẹp. Khi chưa có các thông số này,
dùng `can_probe` đọc giá trị thô; launch mô hình phần cứng sẽ từ chối cấu hình
còn `null` và chưa cho phép `mode:=control`.

## Theo dõi phần cứng

Dừng launch demo mock trước để tránh nhiều nguồn joint states/controller manager.
Sau khi điền file motor:

```bash
ros2 launch rebot_arm_rs_hardware hardware.launch.py mode:=monitor

# Chỉ kiểm tra encoder/RViz, chưa cần chạy MoveIt
ros2 launch rebot_arm_rs_hardware hardware.launch.py mode:=monitor moveit:=false

# Dùng file cấu hình riêng
ros2 launch rebot_arm_rs_hardware hardware.launch.py motors_file:=/absolute/path/motors.yaml
```

Chế độ monitor vẫn cần mapping/chiều quay/offset để hiển thị đúng; nó không
được lấy tư thế từ `initial_positions.yaml` của simulator.

## Điều khiển qua MoveIt

```bash
ros2 launch rebot_arm_rs_hardware hardware.launch.py mode:=control motors_file:=/absolute/path/motors.yaml
```

Chế độ này thực sự enable motor và cho phép **Plan & Execute**. Trạng thái
ban đầu lấy từ encoder; giới hạn tốc độ, dòng và CAN watchdog được ghi và
đọc kiểm tra trước khi enable. Motor/firmware không hỗ trợ CSP hoặc các
register tương ứng sẽ làm activation thất bại. Khi lỗi, driver thử stop các
motor nó đã enable và không tự bật lại. Dừng torque có thể làm tay rơi nếu
không được đỡ; firmware watchdog và dừng khẩn phần cứng cần được kiểm tra
trên robot thực tế. Chi tiết wire format và lifecycle: [`PROTOCOL.md`](PROTOCOL.md).

Nếu `ros2 control list_controllers` liên tục chờ dịch vụ, xem log của
`ros2_control_node` trước: phần cứng có thể đã thất bại khi configure/activate,
khiến controller manager chưa khởi tạo xong dù MoveIt/RViz vẫn mở.

Trên PCAN-USB thật, đã gặp `SocketCAN send: No buffer space available` khi
activation gửi dồn frame với `can0 txqueuelen=10`. Transport nay cách các
lần gửi thành công ít nhất 200 µs và chỉ thử lại frame chưa được kernel
chấp nhận khi gặp ENOBUFS/EAGAIN/EINTR, chờ tối đa 5 ms mỗi frame. Nếu hết
hạn hoặc lỗi bus thật, driver vẫn chốt lỗi và thử stop các motor đã bật;
không tự kích hoạt lại. Không cần tăng hàng đợi CAN để che lỗi này.

Vòng write đầu tiên kiểm tra thời gian thực kể từ lúc activation kết thúc.
Trong cửa sổ 50 ms đầu sau activation, period chứa thời gian handshake chỉ
được xử lý bằng gửi lại target giữ, không tăng target. Nếu vòng đầu đến
muộn hoặc period vượt ngưỡng ngoài cửa sổ khởi động này, driver vẫn báo lỗi.

Sau khi build bản sửa, dừng launch lỗi rồi mở lại trong điều kiện thử đã
chuẩn bị. Kiểm tra ba controller active trước khi gửi quỹ đạo. Bản sửa TX
được kiểm thử tự động; cần xác nhận lại activation trên PCAN-USB thật.

## Kiểm thử

```bash
colcon test --packages-select rebot_arm_rs_hardware --event-handlers console_direct+
colcon test-result --verbose
```

Test dùng transport/motor giả, không mở `can0` và không gửi frame tới robot.
Phải xác minh thêm với motor/firmware thật trước khi coi driver đã được kiểm
chứng trên phần cứng. Giới hạn còn lại: chỉ CSP position; chưa MIT/torque,
gravity compensation, gravity-safe parking, tự homing hoặc force control của kẹp.

Kiểm thử tích hợp SocketCAN tùy chọn, cần Linux hỗ trợ user/network namespace,
`vcan`, và dependency MoveIt đã cài. Script tạo CAN ảo cùng 7 motor giả trong
namespace riêng, kiểm tra quét ID, chế độ monitor, rồi 4 mục tiêu MoveIt:

```bash
ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1 unshare --user --map-root-user --net \
  python3 Rebot_Arm_hardware/test/socketcan_integration.py
```

Không chạy script trực tiếp trên bus thật. Script từ chối nếu vẫn nhìn thấy
`can0` của host. Các kiểm thử C++/Python đều qua; kiểm thử tích hợp trên Linux
vcan qua cả 4 mục tiêu Plan & Execute sau bản sửa TX và chu kỳ khởi động,
mimic, đọc kiểm tra CSP/watchdog và stop các motor giả khi tắt. Đây vẫn là
kiểm thử với motor giả, chưa xác minh trên motor RS thực tế.
