#!/usr/bin/env bash
set -e

WS="$HOME/ros2_tim_projekt"
PROFILE="${1:-color}"

case "$PROFILE" in
  color) CFG="$WS/config/basler_preview_color.yaml" ;;
  slam)  CFG="$WS/config/basler_slam_mono.yaml" ;;
  *) echo "Pouzitie: $0 [color|slam]"; exit 1 ;;
esac

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "Spustam Basler s configom: $CFG"

ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py \
  node_name:=basler_cam \
  camera_id:=my_camera \
  config_file:="$CFG" &

CAM_PID=$!

sleep 3

echo "Spustam rqt_image_view"
ros2 run rqt_image_view rqt_image_view &

wait $CAM_PID
