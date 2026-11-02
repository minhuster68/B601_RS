# B601-RS MoveIt 2 (ROS 2 Humble)

Package `rebot_arm_moveit` dùng model của `rebot_arm_rs_description`, gồm
MoveIt `move_group`, OMPL/RRTConnect, IK KDL, RViz MotionPlanning và
`ros2_control` với `mock_components/GenericSystem`.

## Cài dependency và build

Trên Ubuntu 22.04 với ROS 2 Humble, cài dependency hệ thống:

```bash
sudo apt update
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
```

Từ thư mục gốc repository:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to rebot_arm_moveit
source install/setup.bash
ros2 launch rebot_arm_moveit demo.launch.py
```

Chạy `demo.launch.py` riêng, dừng `display.launch.py` trước để tránh hai nguồn
`/joint_states` và `/robot_description` cùng điều khiển mô hình.

## Plan & Execute trong RViz

1. Chọn planning group `arm` trong panel MotionPlanning.
2. Đặt Start State là current, chọn Goal State `ready` hoặc `home`, hoặc kéo
   interactive marker ở `gripper_end`.
3. Chọn **Plan**, xem đường đi, rồi **Execute** hoặc **Plan & Execute**.
4. Chuyển group sang `gripper`, chọn named state `open` hoặc `closed` để thử kẹp.

Arm có 6 khớp độc lập. Gripper chỉ điều khiển `gripper_joint1`;
`gripper_joint2` mimic với tỷ lệ +1, bao gồm cả phản hồi mock hardware.
Tư thế khởi tạo nằm trong `config/initial_positions.yaml`.

Khung `world` được gắn cố định vào `base_link` trong mô hình do launch tạo.
URDF nguồn, mesh và giới hạn vị trí gốc không bị sửa. Các visual được xử lý
giống launch RViz của package description để giữ màu đúng trên Humble/Jazzy.

## Kiểm tra không cần màn hình

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rebot_arm_moveit demo.launch.py rviz:=false
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control list_controllers
ros2 run rebot_arm_moveit smoke_test.py
```

Smoke test lập kế hoạch và thực thi lần lượt `arm/ready`, `arm/home`,
`gripper/open`, `gripper/closed`, kiểm tra phản hồi `/joint_states` và mimic.
Thêm `--plan-only` nếu chỉ muốn kiểm tra planner.

Kiểm tra cấu hình:

```bash
colcon test --packages-select rebot_arm_moveit --event-handlers console_direct+
colcon test-result --verbose
```

## Các file cấu hình

| File trong `config/` | Vai trò |
| --- | --- |
| `rebot_arm_rs.srdf` | Nhóm arm/gripper, end effector, named states, các cặp collision được bỏ qua |
| `kinematics.yaml` | IK KDL cho chuỗi `base_link` → `gripper_end` |
| `joint_limits.yaml` | Giới hạn vận tốc/gia tốc cho lập kế hoạch mô phỏng |
| `ompl_planning.yaml` | Planner RRTConnect và request adapters cho Humble |
| `moveit_controllers.yaml` | Ánh xạ MoveIt sang hai FollowJointTrajectory actions |
| `ros2_controllers.yaml` | Bộ phát trạng thái và controller cho tay/kẹp |
| `rebot_arm_rs.ros2_control.xacro` | Mock hardware, command/state interfaces và mimic |
| `initial_positions.yaml` | Tư thế mock hardware ban đầu |
| `moveit.rviz` | RViz MotionPlanning |

Các giới hạn vận tốc/gia tốc ở đây dùng cho mô phỏng, chưa phải tuning cho
động cơ RS thật. Mock hardware bám lệnh lý tưởng, không mô phỏng lực,
trọng lực hoặc tiếp xúc. Collision checking của MoveIt dùng mesh hiện có;
không thay thế kiểm thử gắp vật trong simulator vật lý.

Tài liệu tham khảo: [MoveIt controller configuration](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html)
và [ros2_control Humble mock components](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html).

## Kết quả kiểm thử trên máy hiện tại

- Build hai package thành công với ROS 2 Humble; 3 kiểm tra cấu hình pytest qua.
- Đã thực thi đủ 4 mục tiêu trong smoke test và xác nhận phản hồi khớp/mimic.
- Dịch vụ FK và IK KDL có kiểm tra va chạm trả về thành công tại `gripper_end`.
- RViz mở panel MotionPlanning và hiển thị `Global Status: Ok`.

Máy chưa cài MoveIt hệ thống và `sudo` cần mật khẩu, nên kiểm thử runtime dùng
MoveIt 2.5.10 giải nén trong thư mục tạm, kết hợp với ROS Humble hiện có.
Khi dừng phiên GUI bằng Ctrl+C, bộ thư viện kiểm thử gặp lỗi cleanup ở
`move_group`, và RViz cần launch cưỡng chế kết thúc sau timeout. Lỗi được
quan sát lúc thoát; các bước lập kế hoạch/thực thi phía trên đã hoàn tất.
Cần kiểm tra lại việc thoát sau khi cài MoveIt hệ thống bằng lệnh ở đầu tài liệu.
