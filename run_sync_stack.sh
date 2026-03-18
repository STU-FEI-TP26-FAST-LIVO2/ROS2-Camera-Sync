#!/usr/bin/env bash
set -euo pipefail

WS="${WS:-$HOME/ros2_tim_projekt}"
MODE="${1:-mock}"        # mock | stm32
PROFILE="${2:-color}"    # color | slam

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

cleanup_old() {
  pkill -f pylon_ros2_camera_wrapper 2>/dev/null || true
  pkill -f "ros2 launch pylon_ros2_camera_wrapper" 2>/dev/null || true
  pkill -f hesai_ros_driver_node 2>/dev/null || true
  pkill -f lidar_stamp_bridge_node.py 2>/dev/null || true
  pkill -f mock_trigger_node.py 2>/dev/null || true
  pkill -f stm32_trigger_node.py 2>/dev/null || true
  pkill -f pairing_node.py 2>/dev/null || true
  pkill -f dataset_logger_node.py 2>/dev/null || true
}

cleanup_old

echo "MODE=$MODE PROFILE=$PROFILE"

"$WS/run_basler.sh" "$PROFILE" &
BASLER_PID=$!

sleep 3

if [ "$MODE" = "mock" ]; then
  echo "Spustam mock trigger"
  ros2 run liv_sync mock_trigger_node.py &
  TRIG_PID=$!
elif [ "$MODE" = "stm32" ]; then
  echo "Spustam STM32 trigger node"
  ros2 run liv_sync stm32_trigger_node.py --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200 &
  TRIG_PID=$!
else
  echo "Neznamy mode: $MODE"
  exit 1
fi

ros2 run liv_sync pairing_node.py &
PAIR_PID=$!
ros2 run liv_sync dataset_logger_node.py &
LOG_PID=$!

cleanup() {
  kill ${BASLER_PID:-} ${TRIG_PID:-} ${PAIR_PID:-} ${LOG_PID:-} 2>/dev/null || true
}
trap cleanup INT TERM EXIT

wait
