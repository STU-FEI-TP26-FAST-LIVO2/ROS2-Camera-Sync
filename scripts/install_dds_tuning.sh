#!/bin/bash
# install_dds_tuning.sh
# Skopiruje sysctl config a okamzite ho aplikuje.
# Spusti raz ako root: sudo bash scripts/install_dds_tuning.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONF_SRC="${SCRIPT_DIR}/../system/60-ros2-buffers.conf"
CONF_DST="/etc/sysctl.d/60-ros2-buffers.conf"

if [[ $EUID -ne 0 ]]; then
    echo "Spusti ako root: sudo bash $0"
    exit 1
fi

echo "Kopírujem ${CONF_SRC} -> ${CONF_DST}"
cp "${CONF_SRC}" "${CONF_DST}"
chmod 644 "${CONF_DST}"

echo "Aplikujem sysctl..."
sysctl --system

echo ""
echo "Overenie:"
sysctl net.core.rmem_max net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh

echo ""
echo "Hotovo. Nastavenia sa zachovaju po reštarte."
echo "Ak chces otestovat bez rebuildу ROS ws, spusti:"
echo "  source scripts/source_cyclone_env.sh"
echo "  ros2 launch hesai_ros_driver start.py"
