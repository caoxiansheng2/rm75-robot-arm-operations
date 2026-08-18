#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

set -eo pipefail

PROFILE="${1:-640x480x30}"

echo "[D435] depth profile: ${PROFILE}"

exec ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=left_probe \
  camera_name:=d435 \
  serial_no:=_348122075281 \
  enable_color:=false \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  depth_module.depth_profile:="${PROFILE}" \
  pointcloud.enable:=false \
  initial_reset:=true reconnect_timeout:=2.0 wait_for_device_timeout:=-1.0 depth_module.global_time_enabled:=false depth_module.error_polling_enabled:=false
