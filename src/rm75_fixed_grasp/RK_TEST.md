# RM75 AprilTag two-stage motion test

The node now performs only this sequence:

1. Read `base_link -> grab_tag` and `base_link -> Link7`.
2. Select the tag Z direction that points from the tag toward the current Link7.
3. Send the precontact pose through the official RM `MoveJ_P` interface. Its
   orientation is copied from `/rm_driver/udp_arm_position`.
4. Move along the tag normal through the official RM `MoveL` interface.

No MoveIt/KDL pose IK, position-only IK, orientation-candidate search,
TCP/gripper compensation or RRT fallback is used for these two movements.

## Build on the RK board

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rm75_fixed_grasp
source ~/ros2_ws/install/setup.bash
```

## Start and preview without motion

```bash
ros2 launch rm75_fixed_grasp rm75_vision_bringup.launch.py \
  start_auto_grasp:=true allow_motion:=false dry_run:=true
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 service call /rm75_fixed_grasp/preview std_srvs/srv/Trigger '{}'
```

Check the printed tag position, selected normal, precontact and contact points.
The selected normal must point from the wall/tag toward the robot.

## First real-machine test

Keep the emergency stop ready and restart the complete launch:

```bash
ros2 launch rm75_fixed_grasp rm75_vision_bringup.launch.py \
  start_auto_grasp:=true allow_motion:=true dry_run:=false \
  execute_contact:=false
```

Then trigger exactly one motion:

```bash
ros2 service call /rm75_fixed_grasp/execute std_srvs/srv/Trigger '{}'
```

This first command tests only RM MoveJ_P to the 10 cm precontact point. After
that succeeds and the physical direction is confirmed, restart with
`execute_contact:=true` to add the 5 cm RM MoveL segment.

The default target leaves Link7 5 cm in front of the tag. It does not include
the gripper-tip offset and therefore must not be treated as a button press yet.

Retract along the current tag normal:

```bash
ros2 service call /rm75_fixed_grasp/retract std_srvs/srv/Trigger '{}'
```

Return to the configured MoveIt named target:

```bash
ros2 service call /rm75_fixed_grasp/return_home std_srvs/srv/Trigger '{}'
```
