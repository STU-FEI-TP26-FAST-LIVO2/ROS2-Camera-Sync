#!/usr/bin/env bash
set -e

WS="$HOME/ros2_tim_projekt"
PROFILE="${1:-slam}"

case "$PROFILE" in
  slam)  CFG="$WS/config/basler_slam_mono.yaml"; GAIN="0.0" ;;
  color) CFG="$WS/config/basler_preview_color.yaml"; GAIN="1.0" ;;
  *) echo "Usage: $0 [slam|color]"; exit 1 ;;
esac

NS="/my_camera"
NODE="basler_cam"
TOPIC="$NS/$NODE/image_raw"

source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble
export ROS_VERSION=2
source "$WS/install/setup.bash"

ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py \
  node_name:="$NODE" \
  camera_id:="my_camera" \
  config_file:="$CFG" &
LAUNCH_PID=$!

cleanup() {
  echo "[run_basler.sh] Stopping..."
  kill $RQT_PID 2>/dev/null || true
  kill $LAUNCH_PID 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "[run_basler.sh] Waiting for $TOPIC ..."
for i in {1..80}; do
  if ros2 topic list 2>/dev/null | grep -qx "$TOPIC"; then
    break
  fi
  sleep 0.25
done

# set gain reliably via service
ros2 service call $NS/$NODE/set_gain pylon_ros2_camera_interfaces/srv/SetGain "{target_gain: $GAIN}" >/dev/null || true

ros2 run rqt_image_view rqt_image_view &
RQT_PID=$!

sleep 0.6
ros2 param set /rqt_image_view topic "$TOPIC" 2>/dev/null || true

echo "[run_basler.sh] Profile=$PROFILE CFG=$CFG"
wait $LAUNCH_PID
