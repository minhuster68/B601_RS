# ReBot Arm RS ROS 2 Description Package

## ROS 2 Humble: build và mở RViz

Package: `rebot_arm_rs_description` (build type `ament_cmake`).

Launch [`launch/display.launch.py`](launch/display.launch.py) chạy
`robot_state_publisher`, `joint_state_publisher_gui` và RViz với cấu hình
[`rviz/rs.rviz`](rviz/rs.rviz). GUI có thanh trượt cho 6 khớp tay và 1 khớp
kẹp `gripper_joint1`. Ngón còn lại (`gripper_joint2`) dùng `mimic` với
`multiplier="1"`, `offset="0"`: hai ngón trượt cùng quãng đường 0–0.05 m.
Hai hệ tọa độ khớp quay ngược nhau nên cùng giá trị khớp tạo chuyển động
đóng/mở đối xứng. Giới hạn riêng 0.0715 m của ngón phụ trong URDF được giữ
nguyên; khi mimic, hành trình thực tế theo khớp chính chỉ tới 0.05 m.

Chạy từ thư mục gốc repository:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths Rebot_Arm_description/RS_description --ignore-src -r -y
colcon build --symlink-install --packages-select rebot_arm_rs_description
source install/setup.bash
ros2 launch rebot_arm_rs_description display.launch.py
```

Nếu chưa có `rosdep` hoặc `colcon`, cài `python3-rosdep` và
`python3-colcon-common-extensions`; khởi tạo `rosdep` bằng `sudo rosdep init`
(chỉ lần đầu trên máy), rồi chạy `rosdep update`.

Các chế độ khác:

```bash
# RViz với trạng thái khớp mặc định, không mở bảng thanh trượt
ros2 launch rebot_arm_rs_description display.launch.py gui:=false

# Chạy TF và joint_states không cần màn hình
ros2 launch rebot_arm_rs_description display.launch.py gui:=false rviz:=false

# Xem toàn bộ tùy chọn launch
ros2 launch rebot_arm_rs_description display.launch.py --show-args
```

RViz dùng fixed frame `base_link`, nhận mô hình qua `/robot_description`, và
nhận TF từ trạng thái `/joint_states`. Có thể bật mục `TF` trong Displays để
xem các hệ tọa độ. `use_sim_time` mặc định là `false`; chỉ bật khi có simulator
phát `/clock`.

Đây là mô phỏng động học/hiển thị: thanh trượt cập nhật tư thế mô hình và TF.
Package chưa có động lực học, phát hiện va chạm, MoveIt, `ros2_control` hay
driver CAN điều khiển động cơ thật. Không dùng giới hạn effort/velocity trong
URDF như thông số điều khiển phần cứng đã được xác minh.

URDF nguồn giữ đường dẫn mesh tương đối để dùng với các loader khác. Khi
launch, các đường dẫn được chuyển sang `package://rebot_arm_rs_description/`
trong bộ nhớ và script `../tools/rviz_urdf_compat.py` tách visual đa vật liệu
để hiển thị đúng màu trên RViz. Build package từ repository này để CMake có
thể cài kèm script dùng chung đó. Các file URDF và collision gốc không bị sửa.

## Mechanical model resources

<p align="center">
  <strong>
    <a href="./README_zh.md">简体中文</a> &nbsp;|&nbsp;
    <a href="./README.md">English</a>
  </strong>
</p>

This directory contains the self-contained B601-RS mechanical description resources. See the parent [`README.md`](../README.md) for the overall RS/DM differences and cross-engine reuse guidance.

```text
RS_description/
├── package.xml
├── CMakeLists.txt
├── launch/display.launch.py
├── rviz/rs.rviz
├── mjcf/                 # MJCF XML and source metadata
├── urdf/
│   └── ReBot_Arm_RS.urdf
└── meshes/
    ├── visual/             # Arm and wrist-camera parts used only for rendering
    ├── shared/             # Complete meshes shared by rendering and URDF collision
    └── mujoco_collision/   # Detailed MuJoCo finger-collision meshes
```

- `urdf/ReBot_Arm_RS.urdf` is the current main RS model and defines both the arm and gripper.
- `meshes/visual/` contains 54 STL files used only for rendering and colors, including wrist-camera visuals.
- `meshes/shared/` contains 10 STL files used for both rendering and collision. RS currently has no independent meshes used only for URDF collision.
- `meshes/mujoco_collision/` contains 10 MuJoCo collision meshes for the front, middle, and rear finger segments, carriages, and travel stops.
- The URDF uses `../meshes/...` relative paths and does not depend on ROS package URIs. Copy the complete `RS/` directory to use it.
- The ROS 2 runtime model remains in `rebotarm_ros2_RS/src/rebotarm_bringup/description/`. Compare and synchronize the two locations after model updates.

For example, load it from the repository root with:

```python
from pathlib import Path

urdf_path = Path("Rebot_Arm_description/RS_description/urdf/ReBot_Arm_RS.urdf")
```

## MJCF / MuJoCo

The complete model and scene XML files are in [`mjcf/`](mjcf/). Copy this entire arm directory to preserve relative mesh references. See the parent [MuJoCo instructions](../README.md#mujoco) for model selection, loading and validation. Source commits and hashes are in [`../source.json`](../source.json). Existing DigitalTwin runtime projects are unchanged.

The RS runtime source identifies `LAN-GER/reBot-B601-RS-for-mujoco_sim` revision `1249cb6efdf393ba636056fc41df30dc6ba389aa` as its original model source and records that no upstream LICENSE file existed at that revision. The archived files retain this provenance. Camera asset attribution and license texts are in [`mjcf/THIRD_PARTY_MODELS.md`](mjcf/THIRD_PARTY_MODELS.md) and [`mjcf/licenses/`](mjcf/licenses/).
