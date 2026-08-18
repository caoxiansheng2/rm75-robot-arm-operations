# RK board installation guide (Ubuntu 22.04 + ROS 2 Humble)

This workspace uses official ROS 2 drivers instead of compiling the preserved
ROS 1 sources under `moveit_config`, `msg`, `sensors`, and `start`.

## 1. Prepare the shell

Do not build ROS 2 inside Conda/Miniforge:

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
which python3
```

`which python3` must print `/usr/bin/python3`.

## 2. Install packages

```bash
sudo apt update
sudo apt install -y \
  git python3-colcon-common-extensions python3-empy python3-rosdep python3-vcstool \
  ros-humble-moveit ros-humble-rviz2 ros-humble-tf2-tools \
  ros-humble-librealsense2-* ros-humble-realsense2-* \
  ros-humble-apriltag-ros
```

## 3. Download the required RM75-B ROS 2 stack

```bash
cd ~/ros2_ws
vcs import src < ros2_dependencies_required.repos
```

Then install the RealMan driver library:

```bash
cd ~/ros2_ws/src/ros2_rm_robot/rm_driver/lib
sudo bash lib_install.sh
```

## 4. Optional AG95 driver

Only use this when the AG95 RS-485/USB adapter is connected directly to the RK
board. If the gripper is controlled through the RM75 controller/tool port, use
the RealMan end-effector interface instead.

```bash
cd ~/ros2_ws
vcs import src < ros2_dependencies_optional_ag95.repos
```

Install its `serial` CMake library outside the ROS 2 workspace. Do not place
this Catkin-era library under `~/ros2_ws/src`:

```bash
cd ~
git clone https://github.com/ian-chuang/serial-ros2.git
cd ~/serial-ros2
make -j2
sudo make install
sudo ldconfig
```

The AG95 repository is a community driver, not an official DH Robotics ROS 2
release. Validate it with the arm powered off before enabling combined motion.

## 5. Dependencies and build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

colcon build --packages-select rm_ros_interfaces --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
colcon build --symlink-install --executor sequential \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## 6. Validate without motion

The RK board IP and robotic-arm controller IP are different devices. Configure
the real RM75 controller IP in the official driver before launching it.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rm_driver rm_75_driver.launch.py
```

In a second terminal:

```bash
ros2 topic echo /joint_states --once
```

The RM75-B must report seven joints. Do not command motion if the model, joint
count, joint order, or current pose is wrong.

## 7. Launch order

```bash
# Terminal 1: official RM75-B driver + MoveIt2
ros2 launch rm_bringup rm_75_bringup.launch.py

# Terminal 2: RealSense + AprilTag
ros2 launch rm75_fixed_grasp camera_tag.launch.py

# Terminal 3: hand-eye transform (replace every ... value)
ros2 launch rm75_fixed_grasp handeye_static.launch.py \
  x:=... y:=... z:=... roll:=... pitch:=... yaw:=... \
  parent_frame:=Link7 child_frame:=camera_link

# Terminal 4: fixed-grasp coordinator
ros2 launch rm75_fixed_grasp fixed_grasp.launch.py
```

First call is planning-only because motion and gripper output are locked:

```bash
ros2 service call /rm75_fixed_grasp/execute std_srvs/srv/Trigger '{}'
```

Do not change the safety flags in `src/rm75_fixed_grasp/config/grasp.yaml`
until TF, hand-eye calibration, collision geometry, seven joint states, and
dry-run plans have all been verified.
