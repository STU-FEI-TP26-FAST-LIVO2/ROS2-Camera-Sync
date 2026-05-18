#!/bin/bash
# Launch script with network setup and health checks for Basler + Hesai LiDAR
# This script:
# 1. Configures eno1 network interface with static IP
# 2. Verifies network link state
# 3. Pings LiDAR (192.168.1.201) to verify connectivity
# 4. Launches ROS2 LiDAR and Camera drivers only if network is healthy

set -e

echo "======================================================================"
echo "LiDAR-Camera Sync Launch with Network Health Check"
echo "======================================================================"

LIDAR_IP="192.168.1.201"
JETSON_IP="192.168.1.100"
JETSON_NETMASK="24"
INTERFACE="eno1"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to log with color
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_info "Starting network configuration..."

# Check if running as root for network commands
if [ "$EUID" -ne 0 ]; then
    log_warn "Not running as root. Some network commands may require sudo."
fi

# Step 1: Configure eno1 interface
log_info "Step 1: Configuring $INTERFACE with IP $JETSON_IP/$JETSON_NETMASK..."
sudo ip addr flush dev $INTERFACE 2>/dev/null || true
sudo ip addr add $JETSON_IP/$JETSON_NETMASK dev $INTERFACE || {
    log_error "Failed to set IP on $INTERFACE"
    exit 1
}
log_info "IP address configured: $JETSON_IP/$JETSON_NETMASK"

# Step 2: Bring interface UP
log_info "Step 2: Bringing up $INTERFACE..."
sudo ip link set dev $INTERFACE up
sleep 1

# Step 3: Check link state
log_info "Step 3: Verifying link state..."
LINK_STATE=$(sudo ip link show $INTERFACE | grep "state")
if [[ $LINK_STATE == *"DOWN"* ]]; then
    log_warn "Link state: DOWN - checking if hardware is available..."
    # Sometimes interface needs a moment to come up
    sleep 2
fi
LINK_STATE=$(sudo ip link show $INTERFACE | grep "state")
log_info "Link state: $LINK_STATE"

# Step 4: Ping LiDAR
log_info "Step 4: Pinging LiDAR at $LIDAR_IP..."
PING_TIMEOUT=5
PING_COUNT=3

if ping -W $PING_TIMEOUT -c $PING_COUNT $LIDAR_IP >/dev/null 2>&1; then
    log_info "✓ LiDAR is reachable at $LIDAR_IP"
else
    log_error "✗ LiDAR is NOT reachable at $LIDAR_IP"
    log_error "Network connectivity check failed. Aborting launch."
    echo ""
    log_error "Troubleshooting steps:"
    echo "  1. Verify LiDAR is powered on"
    echo "  2. Check ethernet cable connection to $INTERFACE"
    echo "  3. Verify LiDAR network configuration (should be on 192.168.1.x subnet)"
    echo "  4. Run: ethtool $INTERFACE (check if link detected)"
    echo "  5. Run: sudo ip route show (verify routing)"
    exit 1
fi

# Step 5: Verify Jetson IP configuration
log_info "Step 5: Verifying Jetson IP configuration..."
CURRENT_IP=$(ip addr show $INTERFACE | grep "inet " | awk '{print $2}')
if [ -z "$CURRENT_IP" ]; then
    log_error "Failed to verify IP configuration"
    exit 1
fi
log_info "Jetson IP: $CURRENT_IP"

# Step 6: Check for ROS environment
log_info "Step 6: Checking ROS2 environment..."
if [ -z "$ROS_DOMAIN_ID" ]; then
    log_warn "ROS_DOMAIN_ID not set, will use default (0)"
else
    log_info "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
fi

# Step 7: Source ROS2 setup
log_info "Step 7: Sourcing ROS2 environment..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    log_info "ROS2 humble sourced"
else
    log_error "ROS2 setup script not found"
    exit 1
fi

# Source workspace if it exists
WORKSPACE_SETUP="$(dirname "$(dirname "$0")")/install/setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    source "$WORKSPACE_SETUP"
    log_info "Workspace setup sourced"
else
    log_warn "Workspace setup not found at $WORKSPACE_SETUP"
fi

echo ""
log_info "======================================================================"
log_info "Network health check PASSED ✓"
log_info "LiDAR is reachable and network is configured"
log_info "======================================================================"
echo ""

# Step 8: Launch ROS2 drivers
log_info "Step 8: Launching ROS2 LiDAR and Camera drivers..."
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

# Launch with environment variables for optimization
export CYCLONEDDS_URI="file://$PROJECT_ROOT/src/basler_ext_trigger_cpp/config/cyclonedds.xml"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

log_info "Launching: ros2 launch basler_ext_trigger_cpp ext_trigger_camera.launch.py"
log_info "LiDAR IP: $LIDAR_IP"
log_info "Jetson IP: $JETSON_IP"
log_info "Network Interface: $INTERFACE"
echo ""

# Launch the application
ros2 launch basler_ext_trigger_cpp ext_trigger_camera.launch.py

# If we get here, the launch exited normally
log_info "Launch completed"
