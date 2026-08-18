# RM75 Robot Arm Operations

基于 **ROS 2 Humble** 的睿尔曼 **RM75-B 七轴机械臂视觉操作工作空间**。项目使用 RealSense 相机、AprilTag、手眼标定和 RM 官方驱动，实现机械臂按按钮、旋转旋钮以及相关调试与部署流程。

> 这是实机控制项目。运行任何运动命令前，必须核对机械臂型号、控制器 IP、关节状态、TF、手眼标定、目标偏置和现场安全空间，并保持急停可用。

## 功能

- AprilTag 视觉定位与 `base_link` 目标坐标转换
- RM75 按按钮：预接近、直线接触、撤回、返回固定关节姿态
- RM75 旋转旋钮：预接近、第七轴预旋转、夹爪开合、接触旋转、撤回和返回
- MoveIt 2、RViz、RM 驱动、RealSense 与手眼标定集成
- 面向 RK 板的启动脚本、安装说明和运行日志管理
- 左臂局放检测、深度相机和 HTTP 任务服务辅助程序

## 主要目录

| 路径 | 说明 |
| --- | --- |
| `src/rm75_fixed_grasp/` | 按按钮和旋钮操作的核心 ROS 2 包 |
| `src/rm75_fixed_grasp/src/fixed_grasp_node.cpp` | 按按钮运动节点 |
| `src/rm75_fixed_grasp/src/turn_knob_node.cpp` | 旋转旋钮运动节点 |
| `src/rm75_fixed_grasp/config/grasp.yaml` | 按按钮参数 |
| `src/rm75_fixed_grasp/config/knob.yaml` | 旋钮、夹爪及 AprilTag ID 1 参数 |
| `src/rm75_fixed_grasp/launch/` | 相机、AprilTag、手眼、按按钮和旋钮 launch 文件 |
| `src/ros2_rm_robot/` | RM ROS 2 驱动、接口、描述和 MoveIt 配置 |
| `right_arm_press_button.sh` | 一键执行右臂按按钮任务 |
| `right_arm_turn_knob.sh` | 一键执行右臂旋转旋钮任务 |
| `RK_INSTALL.md` | RK 板环境安装与构建说明 |
| `LEFT_PROBE_PD_README.md` | 左臂局放检测子系统说明 |

## 环境与硬件

- Ubuntu 22.04
- ROS 2 Humble
- RealMan RM75-B 七轴机械臂
- Intel RealSense D435 系列相机
- AprilTag `36h11`
- MoveIt 2、RViz 2、`apriltag_ros`、`easy_handeye2`

当前配置中包含特定设备的相机序列号、机械臂网络参数和实机姿态。部署到另一台设备前必须按现场情况修改。

## 安装与编译

完整依赖和 RK 板部署方法见 [RK_INSTALL.md](RK_INSTALL.md)。已经安装 ROS 2 Humble 和依赖时，可在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r -y

colcon build \
  --symlink-install \
  --executor sequential

source install/setup.bash
```

如果只修改了视觉操作包：

```bash
colcon build \
  --symlink-install \
  --packages-select rm75_fixed_grasp

source install/setup.bash
```

## 手眼标定

默认启动流程读取以下标定：

```text
~/.ros2/easy_handeye2/calibrations/rm75_right_handeye.calib
```

该文件与具体机械臂和相机安装位置绑定，不应直接复用其他设备的结果。运行任务前检查 TF 链：

```bash
timeout 5 ros2 run tf2_ros tf2_echo base_link grab_tag
timeout 5 ros2 run tf2_ros tf2_echo base_link Link7
```

## 按按钮

按按钮的一键脚本会启动完整视觉 bringup，等待稳定的 `grab_tag`，执行预接近和接触，然后撤回并返回 `grasp.yaml` 中配置的固定关节姿态：

```bash
chmod +x right_arm_press_button.sh
./right_arm_press_button.sh
```

停止脚本管理的任务：

```bash
./right_arm_press_button.sh stop
```

关键参数位于 `src/rm75_fixed_grasp/config/grasp.yaml`：

- `target_offset_base_xyz`：目标相对二维码中心在 `base_link` XYZ 方向的偏置，单位为米
- `precontact_distance`：预接近点到二维码平面的距离
- `contact_standoff`：最终接触阶段 Link7 到二维码平面的保留距离
- `return_joint_values`：任务完成后的七关节返回姿态，单位为弧度
- `driver_speed`：RM 控制器运动速度
- `auto_retract`：完成接触后是否先撤回预接近点

## 旋转旋钮

旋钮使用 AprilTag 数字 ID `1`，TF 名称为 `knob_tag`：

```bash
chmod +x right_arm_turn_knob.sh
./right_arm_turn_knob.sh
```

停止脚本管理的任务：

```bash
./right_arm_turn_knob.sh stop
```

关键参数位于 `src/rm75_fixed_grasp/config/knob.yaml`：

- `target_offset_base_xyz`：旋钮目标偏置，单位为米
- `precontact_distance` / `contact_standoff`：预接近与接触距离
- `knob_pre_rotate_degrees`：接触前第七轴旋转角度
- `knob_turn_degrees`：夹住旋钮后的第七轴旋转角度
- `knob_max_abs_rotation_degrees`：允许的最大旋转安全限制
- `gripper_open_value` / `gripper_close_value`：夹爪开合寄存器值
- `return_joint_values`：任务结束后的固定姿态

如果实机的左右旋转方向与预期相反，应同时核对第七轴正方向和旋钮安装方向，再调整两个角度的符号。

## 手动启动和诊断

启动按按钮完整系统：

```bash
ros2 launch rm75_fixed_grasp rm75_vision_bringup.launch.py \
  start_auto_grasp:=true \
  auto_execute:=true \
  allow_motion:=true \
  dry_run:=false \
  execute_contact:=true
```

启动旋钮系统：

```bash
ros2 launch rm75_fixed_grasp turn_knob.launch.py \
  allow_motion:=true \
  dry_run:=false
```

在另一个终端触发旋钮任务：

```bash
ros2 service call /rm75_turn_knob/execute \
  std_srvs/srv/Trigger '{}'
```

常用诊断命令：

```bash
ros2 node list | sort
ros2 topic echo /joint_states --once
timeout 5 ros2 topic echo /rm_driver/udp_rm_err --once
timeout 5 ros2 topic echo /rm_driver/udp_joint_error_code --once
pgrep -af 'rm_driver|rm_control|realsense2_camera|apriltag|rviz2'
```

## 安全说明

1. 第一次使用新参数时先设置 `allow_motion:=false` 或 `dry_run:=true`。
2. 确认只启动一个对应机械臂的 `rm_driver`，避免重复节点和指令竞争。
3. 实机执行前检查 `/joint_states`、RM 系统错误码、关节使能状态和完整 TF 树。
4. `target_offset_base_xyz`、接近距离和返回关节姿态不能照搬到不同安装现场。
5. 相机 TF 丢失、目标过期、运动距离超过安全阈值或控制器拒绝命令时，不应绕过保护继续运动。
6. 机械臂周围应保持无人，操作人员必须能够立即触发急停。

## 第三方组件

仓库包含或集成 RealMan ROS 2、RealSense ROS、MoveIt 配置、AprilTag 与 easy_handeye2 等组件。各组件继续适用其自身目录中的许可证和上游项目条款。

