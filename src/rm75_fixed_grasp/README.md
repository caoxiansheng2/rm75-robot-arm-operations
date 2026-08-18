# rm75_fixed_grasp

ROS 2 Humble test node for moving an RM75 toward an AprilTag without executing
an unconstrained position-only IK solution or an RRT fallback.

The active implementation is intentionally limited to validating the robot and
vision geometry:

```text
current pose
  -> official RM MoveJ_P to 10 cm in front of grab_tag
  -> official RM MoveL along the verified tag normal
  -> stop with Link7 5 cm in front of grab_tag
```

The target quaternion is copied from the RM controller's live
`/rm_driver/udp_arm_position` message, avoiding a second MoveIt/KDL IK solve.
The code automatically chooses the sign of the tag Z axis that points toward
the current Link7. This avoids depending on an assumed AprilTag Z convention.
It does not yet apply a camera-to-tool or gripper-tip compensation.

See `RK_TEST.md` for build, preview and real-machine test commands.
