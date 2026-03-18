#!/usr/bin/env bash
set -euo pipefail

WS="${WS:-$HOME/ros2_tim_projekt}"
PROFILE="${1:-color}"   # color | slam
STM_PORT="${2:-/dev/ttyUSB0}"
STM_BAUD="${3:-115200}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "MODE=LIDAR_STM PROFILE=$PROFILE PORT=$STM_PORT BAUD=$STM_BAUD"

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

mkdir -p "$WS/dataset"

"$WS/run_basler.sh" "$PROFILE" &
BASLER_PID=$!

sleep 3

ros2 launch hesai_ros_driver start.py config_path:="$WS/config/hesai_lidar.yaml" &
HESAI_PID=$!

sleep 3

ros2 run liv_sync lidar_stamp_bridge_node.py &
LIDAR_BRIDGE_PID=$!

echo "Spustam STM32 trigger node na porte $STM_PORT"
ros2 run liv_sync stm32_trigger_node.py --ros-args -p port:="$STM_PORT" -p baudrate:="$STM_BAUD" &
TRIG_PID=$!

ros2 run liv_sync pairing_node.py &
PAIR_PID=$!
ros2 run liv_sync dataset_logger_node.py &
LOG_PID=$!

cleanup() {
  kill ${BASLER_PID:-} ${HESAI_PID:-} ${LIDAR_BRIDGE_PID:-} ${TRIG_PID:-} ${PAIR_PID:-} ${LOG_PID:-} 2>/dev/null || true
}
trap cleanup INT TERM EXIT

wait
