#!/bin/bash
# source_cyclone_env.sh
#
# Pouzitie:
#   source scripts/source_cyclone_env.sh
#   ros2 launch basler_ext_trigger_cpp ext_trigger_camera.launch.py

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CYCLONE_XML="${SCRIPT_DIR}/../src/basler_ext_trigger_cpp/config/cyclonedds.xml"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONE_XML}"

# Pylon runtime
if [ -f /opt/pylon/bin/pylon-setup-env.sh ]; then
    source /opt/pylon/bin/pylon-setup-env.sh
else
    export PYLON_ROOT=/opt/pylon
    export LD_LIBRARY_PATH=/opt/pylon/lib:${LD_LIBRARY_PATH}
fi

echo "Cyclone DDS nastavene:"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "  CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "Pylon nastavene:"
echo "  PYLON_ROOT=${PYLON_ROOT}"
echo "  LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"