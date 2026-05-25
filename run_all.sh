#!/usr/bin/env bash
set -Eeuo pipefail

# ============================================================
# run_all.sh
# - otvara terminalove okna ako povodne
# - LiDAR/bridge/FAST terminale NESOURCUJU Pylon
# - Pylon sa sourcuje iba v Basler Camera terminali
# - terminaly sa spustaju cez setsid + &, cize hlavny skript nepocka/zamrzne
# - automaticky build STM32 je vypnuty defaultne
# - pouziva existujuci funkcny cyclonedds.xml, nie vynuteny force XML
# - VSETKY VYPISY TERMINALOV uklada do ~/terminalz/run_DATUM_CAS/
# ============================================================

ROS_SETUP="/opt/ros/humble/setup.bash"
BRIDGE_WS="$HOME/ROS2-LIDAR-CAMERA-BRIDGE-main"
CAMERA_WS="$HOME/ROS2-Camera-Sync-dominik"
FAST_WS="$HOME/ROS2-FAST-LIVO2-WS"

BRIDGE_SETUP="$BRIDGE_WS/install/setup.bash"
CAMERA_SETUP="$CAMERA_WS/install/setup.bash"
FAST_SETUP="$FAST_WS/install/setup.bash"


# ============================================================
# LOGY TERMINALOV
# ============================================================
TERMINAL_LOG_BASE="$HOME/terminalz"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
TERMINAL_LOG_DIR="$TERMINAL_LOG_BASE/run_$RUN_ID"
MAIN_LOG="$TERMINAL_LOG_DIR/main.log"

FAST_CONFIG="$FAST_WS/src/FAST-LIVO2-ROS2-MID360-Fisheye/config/dominik_sync.yaml"
CAMERA_CONFIG="$FAST_WS/src/FAST-LIVO2-ROS2-MID360-Fisheye/config/basler_camera.yaml"

RUN_FAST_LIVO="${RUN_FAST_LIVO:-1}"
RUN_IMU="${RUN_IMU:-1}"
RUN_RVIZ="${RUN_RVIZ:-True}"
RUN_MONITOR="${RUN_MONITOR:-0}"
RUN_BUILD_IMU="${RUN_BUILD_IMU:-0}"
KILL_OLD="${KILL_OLD:-1}"

# Ak FAST config ma imu_en:true a /imu nebezi:
# 1 = FAST-LIVO nespustim, aby hned nespadol
# 0 = FAST-LIVO spustim aj tak
RUN_REQUIRE_IMU_FOR_LIVO="${RUN_REQUIRE_IMU_FOR_LIVO:-1}"

LIDAR_IFACE="${LIDAR_IFACE:-eno1}"
LIDAR_HOST_IP="${LIDAR_HOST_IP:-192.168.1.100/24}"
LIDAR_SENSOR_IP="${LIDAR_SENSOR_IP:-192.168.1.201}"
IMU_PORT="${IMU_PORT:-auto}"

REQUIRED_RMEM_MIN=10485760
TARGET_RMEM=2147483647
TARGET_RMEM_DEFAULT=26214400
TARGET_WMEM=2147483647
TARGET_WMEM_DEFAULT=26214400

CYCLONE_XML_CANDIDATES=(
  "$CAMERA_WS/src/basler_ext_trigger_cpp/config/cyclonedds.xml"
  "$CAMERA_WS/install/basler_ext_trigger_cpp/share/basler_ext_trigger_cpp/config/cyclonedds.xml"
  "$BRIDGE_WS/src/lidar_stamp_bridge/config/cyclonedds.xml"
  "$BRIDGE_WS/install/lidar_stamp_bridge/share/lidar_stamp_bridge/config/cyclonedds.xml"
  "$BRIDGE_WS/src/HesaiLidar_ROS_2.0/config/cyclonedds.xml"
  "$BRIDGE_WS/install/hesai_ros_driver/share/hesai_ros_driver/config/cyclonedds.xml"
)

CYCLONE_XML=""
for candidate in "${CYCLONE_XML_CANDIDATES[@]}"; do
  if [ -f "$candidate" ]; then
    CYCLONE_XML="$candidate"
    break
  fi
done

mkdir -p "$TERMINAL_LOG_DIR"

exec > >(tee -a "$MAIN_LOG") 2>&1

echo "===== START $(date) ====="
echo "Logy z tohto spustenia sa ukladaju do:"
echo "$TERMINAL_LOG_DIR"
echo

need_file() {
  if [ ! -f "$1" ]; then
    echo "[ERROR] Chyba subor: $1"
    exit 1
  fi
}

need_dir() {
  if [ ! -d "$1" ]; then
    echo "[ERROR] Chyba priecinok: $1"
    exit 1
  fi
}

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[ERROR] Chyba prikaz: $1"
    exit 1
  fi
}

need_file "$ROS_SETUP"
need_file "$BRIDGE_SETUP"
need_file "$CAMERA_SETUP"
need_dir "$BRIDGE_WS"
need_dir "$CAMERA_WS"
need_cmd ros2
need_cmd ping
need_cmd ip
need_cmd sysctl

if [ "$RUN_FAST_LIVO" = "1" ] || [ "$RUN_IMU" = "1" ]; then
  need_file "$FAST_SETUP"
  need_dir "$FAST_WS"
fi

if [ "$RUN_FAST_LIVO" = "1" ]; then
  need_file "$FAST_CONFIG"
  need_file "$CAMERA_CONFIG"
fi

if [ -z "$CYCLONE_XML" ]; then
  echo "[ERROR] Nenasiel som ziadny cyclonedds.xml v BRIDGE/CAMERA workspace."
  exit 1
fi

echo "CYCLONE_XML=$CYCLONE_XML"
echo "RUN_FAST_LIVO=$RUN_FAST_LIVO RUN_IMU=$RUN_IMU RUN_RVIZ=$RUN_RVIZ RUN_MONITOR=$RUN_MONITOR RUN_BUILD_IMU=$RUN_BUILD_IMU"
echo "KILL_OLD=$KILL_OLD RUN_REQUIRE_IMU_FOR_LIVO=$RUN_REQUIRE_IMU_FOR_LIVO"
echo

make_env_block() {
  local include_bridge="$1"
  local include_camera="$2"
  local include_fast="$3"
  local include_pylon="$4"

  cat <<ENVEOF
set +u
: "\${AMENT_TRACE_SETUP_FILES:=}"
source '$ROS_SETUP'
if [ '$include_bridge' = '1' ]; then
  source '$BRIDGE_SETUP'
fi
if [ '$include_camera' = '1' ]; then
  source '$CAMERA_SETUP'
fi
if [ '$include_fast' = '1' ]; then
  source '$FAST_SETUP'
fi
if [ '$include_pylon' = '1' ]; then
  if [ -f /opt/pylon/bin/pylon-setup-env.sh ]; then
    source /opt/pylon/bin/pylon-setup-env.sh /opt/pylon
  else
    export PYLON_ROOT=/opt/pylon
    export LD_LIBRARY_PATH=/opt/pylon/lib:\${LD_LIBRARY_PATH:-}
    export PATH=/opt/pylon/bin:\${PATH:-}
  fi
fi
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='file://$CYCLONE_XML'
export ROS2CLI_DISABLE_DAEMON=1
ENVEOF
}

# Dolezite:
# - LiDAR a bridge bez Pylonu.
# - Kamera s Pylonom.
# - FAST s FAST workspace, ale bez Pylonu.
LIDAR_ENV_BLOCK="$(make_env_block 1 0 0 0)"
BRIDGE_ENV_BLOCK="$(make_env_block 1 0 0 0)"
CAMERA_ENV_BLOCK="$(make_env_block 0 1 0 1)"
COMMON_ENV_BLOCK="$(make_env_block 1 1 0 0)"
FAST_ENV_BLOCK="$(make_env_block 1 1 1 0)"

source_common_here() {
  set +u
  eval "$COMMON_ENV_BLOCK"
  set -u
}

source_fast_here() {
  set +u
  eval "$FAST_ENV_BLOCK"
  set -u
}

run_term_with_env() {
  local title="$1"
  local workdir="$2"
  local logfile="$3"
  local env_block="$4"
  local cmd="$5"

  local safe_title
  safe_title="$(echo "$title" | tr ' /:' '___' | tr -cd '[:alnum:]_-' )"
  local runner_file="$TERMINAL_LOG_DIR/runner_${safe_title}.sh"

  cat > "$runner_file" <<RUNNER
#!/usr/bin/env bash
set -Eeuo pipefail
cd "$workdir"

$env_block

{
  echo "===== START: \$(date) ====="
  echo "Title: $title"
  echo "Workdir: $workdir"
  echo "Logfile: $logfile"
  echo "RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-}"
  echo "CYCLONEDDS_URI=\${CYCLONEDDS_URI:-}"
  echo
  $cmd
} 2>&1 | tee -a "$logfile"

echo
echo "===== PROCESS ENDED. Terminal ostava otvoreny. ====="
exec bash -i
RUNNER

  chmod +x "$runner_file"

  echo "[TERM] Otvaram terminal: $title"
  echo "       runner: $runner_file"
  echo "       log:    $logfile"

  if command -v gnome-terminal >/dev/null 2>&1; then
    setsid gnome-terminal --title="$title" -- bash -lc "exec '$runner_file'" >/dev/null 2>&1 &
  elif command -v x-terminal-emulator >/dev/null 2>&1; then
    setsid x-terminal-emulator -T "$title" -e bash -lc "exec '$runner_file'" >/dev/null 2>&1 &
  else
    echo "[ERROR] Nenasiel som terminal: gnome-terminal ani x-terminal-emulator"
    exit 1
  fi

  sleep 1
}

wait_for_topic_common() {
  local topic="$1"
  local timeout_s="$2"
  source_common_here
  local start
  start=$(date +%s)
  while true; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      echo "[OK] Topic dostupny: $topic"
      return 0
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout_s" ]; then
      echo "[WARN] Timeout topic: $topic"
      return 1
    fi
    sleep 1
  done
}

wait_for_topic_fast() {
  local topic="$1"
  local timeout_s="$2"
  source_fast_here
  local start
  start=$(date +%s)
  while true; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      echo "[OK] Topic dostupny: $topic"
      return 0
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout_s" ]; then
      echo "[WARN] Timeout topic: $topic"
      return 1
    fi
    sleep 1
  done
}

find_launch_file() {
  source_fast_here

  if ros2 pkg prefix fast_livo >/dev/null 2>&1; then
    if ros2 launch fast_livo dominik_launch.py --show-args >/dev/null 2>&1; then
      echo "ros2 launch fast_livo dominik_launch.py"
      return 0
    fi

    if ros2 launch fast_livo Dominik_launch.py --show-args >/dev/null 2>&1; then
      echo "ros2 launch fast_livo Dominik_launch.py"
      return 0
    fi
  fi

  echo "ros2 launch fast_livo hesaihilti.launch.py avia_params_file:=$FAST_CONFIG camera_params_file:=$CAMERA_CONFIG use_rviz:=$RUN_RVIZ"
}

ensure_kernel_buffers() {
  local current_rmem current_rmem_default current_wmem current_wmem_default
  current_rmem=$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
  current_rmem_default=$(sysctl -n net.core.rmem_default 2>/dev/null || echo 0)
  current_wmem=$(sysctl -n net.core.wmem_max 2>/dev/null || echo 0)
  current_wmem_default=$(sysctl -n net.core.wmem_default 2>/dev/null || echo 0)

  echo "Kernel buffers pred kontrolou:"
  echo "  net.core.rmem_max=$current_rmem"
  echo "  net.core.rmem_default=$current_rmem_default"
  echo "  net.core.wmem_max=$current_wmem"
  echo "  net.core.wmem_default=$current_wmem_default"
  echo

  if [ "$current_rmem" -lt "$REQUIRED_RMEM_MIN" ] || \
     [ "$current_rmem_default" -lt "$TARGET_RMEM_DEFAULT" ] || \
     [ "$current_wmem" -lt "$TARGET_WMEM" ] || \
     [ "$current_wmem_default" -lt "$TARGET_WMEM_DEFAULT" ]; then
    echo "Nastavujem sysctl buffre pre ROS2/CycloneDDS..."
    sudo sysctl -w net.core.rmem_max="$TARGET_RMEM" || true
    sudo sysctl -w net.core.rmem_default="$TARGET_RMEM_DEFAULT" || true
    sudo sysctl -w net.core.wmem_max="$TARGET_WMEM" || true
    sudo sysctl -w net.core.wmem_default="$TARGET_WMEM_DEFAULT" || true
  else
    echo "Kernel buffre su dostatocne."
  fi

  echo
}

ensure_lidar_network() {
  echo "Kontrolujem a nastavujem LiDAR interface $LIDAR_IFACE..."

  if ! ip link show "$LIDAR_IFACE" >/dev/null 2>&1; then
    echo "[ERROR] Interface $LIDAR_IFACE neexistuje. Skus napr.:"
    echo "        LIDAR_IFACE=enp12s0 ./run_all.sh"
    exit 1
  fi

  local current_ip
  current_ip="$(ip -4 -o addr show dev "$LIDAR_IFACE" | awk '{print $4}' | head -n1 || true)"
  echo "Aktualna IP na $LIDAR_IFACE: ${current_ip:-ziadna}"

  if [ "${current_ip:-}" != "$LIDAR_HOST_IP" ]; then
    echo "Nastavujem $LIDAR_IFACE na $LIDAR_HOST_IP..."
    sudo ip addr flush dev "$LIDAR_IFACE" || true
    sudo ip addr add "$LIDAR_HOST_IP" dev "$LIDAR_IFACE"
  else
    echo "$LIDAR_IFACE uz ma spravnu IP $LIDAR_HOST_IP"
  fi

  sudo ip link set "$LIDAR_IFACE" up

  echo
  echo "Stav $LIDAR_IFACE:"
  ip a show "$LIDAR_IFACE"
  echo

  echo "Ping test na $LIDAR_SENSOR_IP..."
  ping -I "$LIDAR_IFACE" -c 3 "$LIDAR_SENSOR_IP" || {
    echo "[ERROR] LiDAR neodpoveda"
    exit 1
  }

  echo
}

kill_old_optional() {
  if [ "$KILL_OLD" = "1" ]; then
    echo "Zabijam stare procesy..."
    pkill -f hesai_ros_driver || true
    pkill -f lidar_stamp_bridge || true
    pkill -f basler_ext_trigger || true
    pkill -f fast_livo || true
    pkill -f stm32_sync_driver || true
    pkill -f rviz2 || true
    sleep 1
    echo
  fi
}

ensure_kernel_buffers
ensure_lidar_network
kill_old_optional

echo "Kontrola STM32 IMU: automaticky build je vypnuty defaultne (RUN_BUILD_IMU=$RUN_BUILD_IMU)."

if [ "$RUN_IMU" = "1" ] && [ "$RUN_BUILD_IMU" = "1" ]; then
  if [ ! -d "$FAST_WS/src/stm32_sync_driver" ]; then
    echo "[WARN] Chyba $FAST_WS/src/stm32_sync_driver, IMU build preskakujem."
  else
    echo "Buildim STM32 IMU driver na poziadanie..."
    cd "$FAST_WS"
    source_fast_here
    colcon build --packages-select stm32_sync_driver --event-handlers console_direct+
  fi
fi

source_common_here
ros2 daemon stop >/dev/null 2>&1 || true
rm -f /dev/shm/lidar_stamp.bin /dev/shm/liv_sync_ring.bin /dev/shm/liv_sync_stamp || true

echo
echo "Spustam LiDAR driver v terminali..."
run_term_with_env "LiDAR Driver" "$BRIDGE_WS" "$TERMINAL_LOG_DIR/lidar_driver.txt" "$LIDAR_ENV_BLOCK" \
  "ros2 launch hesai_ros_driver start.py"

sleep 5
wait_for_topic_common "/lidar_points" 40 || true

echo
echo "Spustam LiDAR Stamp Bridge v terminali..."
run_term_with_env "LiDAR Stamp Bridge" "$BRIDGE_WS" "$TERMINAL_LOG_DIR/lidar_stamp_bridge.txt" "$BRIDGE_ENV_BLOCK" \
  "ros2 run lidar_stamp_bridge lidar_stamp_bridge_node --ros-args -p topic:=/lidar_points -p mmap_path:=/dev/shm/lidar_stamp.bin"

sleep 3

echo
echo "Spustam Basler kameru v terminali..."
run_term_with_env "Basler Camera" "$CAMERA_WS" "$TERMINAL_LOG_DIR/basler_camera.txt" "$CAMERA_ENV_BLOCK" \
  "ros2 launch basler_ext_trigger_cpp ext_trigger_camera.launch.py"

sleep 5

if [ "$RUN_IMU" = "1" ]; then
  if [ "$IMU_PORT" = "auto" ]; then
    IMU_PORT=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n1 || true)
  fi

  if [ -z "${IMU_PORT:-}" ]; then
    echo "[WARN] IMU port nenajdeny. IMU terminal nespustam."
  else
    echo
    echo "Spustam STM32 IMU na $IMU_PORT v terminali..."
    run_term_with_env "STM32 IMU" "$FAST_WS" "$TERMINAL_LOG_DIR/stm32_imu.txt" "$FAST_ENV_BLOCK" \
      "ros2 run stm32_sync_driver stm32_sync_driver --ros-args -p port:=$IMU_PORT"
    wait_for_topic_fast "/imu" 20 || true
  fi
else
  echo "IMU nespustam (RUN_IMU=0)."
fi

wait_for_topic_common "/basler/image_raw" 40 || true

if [ "$RUN_FAST_LIVO" = "1" ]; then
  echo
  echo "Pripravujem FAST-LIVO2 launch..."

  START_LIVO=1
  if grep -Eq "imu_en:[[:space:]]*true" "$FAST_CONFIG"; then
    if wait_for_topic_fast "/imu" 5; then
      echo "FAST config ma imu_en=true a /imu bezi."
    else
      if [ "$RUN_REQUIRE_IMU_FOR_LIVO" = "1" ]; then
        echo "[WARN] FAST config ma imu_en=true, ale /imu nebezi. FAST-LIVO2 nespustam, aby nespadol."
        echo "       Ak to chces spustit aj tak:"
        echo "       RUN_REQUIRE_IMU_FOR_LIVO=0 ./run_all.sh"
        START_LIVO=0
      else
        echo "[WARN] FAST config ma imu_en=true, ale /imu nebezi. Spustam FAST-LIVO2 aj tak, lebo RUN_REQUIRE_IMU_FOR_LIVO=0."
      fi
    fi
  fi

  if [ "$START_LIVO" = "1" ]; then
    FAST_LIVO_CMD="$(find_launch_file)"
    echo "Spustam FAST-LIVO2 v terminali:"
    echo "$FAST_LIVO_CMD"
    run_term_with_env "FAST-LIVO2" "$FAST_WS" "$TERMINAL_LOG_DIR/fast_livo2.txt" "$FAST_ENV_BLOCK" "$FAST_LIVO_CMD"
  fi
else
  echo "FAST-LIVO2 nespustam (RUN_FAST_LIVO=0)."
fi

if [ "$RUN_MONITOR" = "1" ]; then
  echo
  echo "Spustam monitor v terminali..."
  run_term_with_env "Sync Monitor" "$CAMERA_WS" "$TERMINAL_LOG_DIR/sync_monitor.txt" "$COMMON_ENV_BLOCK" "
while true; do
  clear
  date
  echo '===== topic list ====='
  ros2 topic list || true
  echo
  echo '===== hz /lidar_points ====='
  timeout 6 ros2 topic hz /lidar_points || true
  echo
  echo '===== hz /basler/image_raw ====='
  timeout 6 ros2 topic hz /basler/image_raw || true
  echo
  echo '===== hz /imu ====='
  timeout 6 ros2 topic hz /imu || true
  echo
  sleep 2
done
"
else
  echo "Monitor nespustam (RUN_MONITOR=0), aby zbytocne neznizoval Hz."
fi

echo
echo "HOTOVO"
echo "Logy z terminalov:"
echo "$TERMINAL_LOG_DIR"
echo
echo "Subory logov budu napr.:"
echo "  $TERMINAL_LOG_DIR/main.log"
echo "  $TERMINAL_LOG_DIR/lidar_driver.txt"
echo "  $TERMINAL_LOG_DIR/lidar_stamp_bridge.txt"
echo "  $TERMINAL_LOG_DIR/basler_camera.txt"
echo "  $TERMINAL_LOG_DIR/stm32_imu.txt"
echo "  $TERMINAL_LOG_DIR/fast_livo2.txt"
echo "  $TERMINAL_LOG_DIR/sync_monitor.txt"
echo
echo "Pouzitie:"
echo "  ~/run_all.sh"
echo "  RUN_IMU=0 RUN_FAST_LIVO=0 RUN_RVIZ=False ~/run_all.sh"
echo "  RUN_RVIZ=False ~/run_all.sh"
echo "  RUN_MONITOR=1 ~/run_all.sh"
echo "  RUN_REQUIRE_IMU_FOR_LIVO=0 ~/run_all.sh"
echo
echo "Najnovsie logy otvoris takto:"
echo "  cd \"\$(ls -td ~/terminalz/run_* | head -n1)\""
echo "  ls -lah"
echo
echo "10 Hz kontrola:"
echo "  ros2 topic hz /lidar_points"
echo "  ros2 topic hz /basler/image_raw"
