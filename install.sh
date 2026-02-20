#!/bin/bash
# =============================================================================
#  KAHU Stack Installer
#  Installs the full KAHU marine radar tracking stack on a Raspberry Pi
#  running Ubuntu 24.04 LTS with ROS2 Jazzy.
#
#  Usage:
#    curl -fsSL https://raw.githubusercontent.com/KAHU-radar/kahu-stack/master/install.sh | bash
#  OR, after cloning this repo:
#    bash install.sh
#
#  What this script does:
#    1. Installs system dependencies and ROS2 Jazzy
#    2. Clones all KAHU repositories
#    3. Builds the ROS2 workspace
#    4. Installs and enables systemd services
#    5. Creates /etc/default/kahu with default config
#
#  After running, edit /etc/default/kahu to set your vessel ID and
#  radar interface, then reboot.
# =============================================================================

set -euo pipefail

# ── Colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()    { echo -e "${GREEN}[KAHU]${NC} $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ── Checks ────────────────────────────────────────────────────────────────────
[[ $EUID -ne 0 ]] && error "Run as root: sudo bash install.sh"
[[ "$(uname -m)" != "aarch64" ]] && warn "This script targets ARM64 (Raspberry Pi). Proceeding anyway..."

INSTALL_USER="${SUDO_USER:-pi}"
INSTALL_HOME=$(getent passwd "$INSTALL_USER" | cut -d: -f6)
STACK_DIR="$INSTALL_HOME/kahu-stack"
ROS2_WS="$INSTALL_HOME/ros2_ws"

info "Installing KAHU stack for user: $INSTALL_USER"
info "Stack directory: $STACK_DIR"
info "ROS2 workspace: $ROS2_WS"
echo ""

# ── 1. System packages ────────────────────────────────────────────────────────
info "Step 1/6 — Installing system packages..."
apt-get update -qq
apt-get install -y -qq \
    git curl wget python3-pip python3-venv \
    build-essential pkg-config \
    net-tools iproute2 \
    sqlite3

# ── 2. ROS2 Jazzy ─────────────────────────────────────────────────────────────
info "Step 2/6 — Installing ROS2 Jazzy..."
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | tee /etc/apt/sources.list.d/ros2.list > /dev/null

    apt-get update -qq
    apt-get install -y -qq \
        ros-jazzy-ros-base \
        python3-colcon-common-extensions \
        ros-jazzy-tf2-ros \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-sensor-msgs \
        python3-sensor-msgs-py
    info "ROS2 Jazzy installed."
else
    info "ROS2 Jazzy already installed, skipping."
fi

# Add ROS2 to shell for this user
sudo -u "$INSTALL_USER" bash -c \
    "grep -q 'ros/jazzy/setup.bash' ~/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc"

# ── 3. Clone repositories ─────────────────────────────────────────────────────
info "Step 3/6 — Cloning KAHU repositories..."
sudo -u "$INSTALL_USER" mkdir -p "$STACK_DIR"

clone_or_pull() {
    local url="$1" dest="$2"
    if [ -d "$dest/.git" ]; then
        info "  Updating $(basename "$dest")..."
        sudo -u "$INSTALL_USER" git -C "$dest" pull --ff-only 2>&1 | tail -1
    else
        info "  Cloning $(basename "$dest")..."
        sudo -u "$INSTALL_USER" git clone "$url" "$dest"
    fi
}

# Mayara server (external — upstream Rust binary)
clone_or_pull \
    https://github.com/MarineYachtRadar/mayara-server.git \
    "$STACK_DIR/mayara-server"

# Radar emulator (dev/testing only)
clone_or_pull \
    https://github.com/KAHU-radar/radar-emulator-navico.git \
    "$STACK_DIR/radar-emulator-navico"

# Mayara→ROS2 bridge
clone_or_pull \
    https://github.com/KAHU-radar/mayara-ros-bridge.git \
    "$STACK_DIR/mayara_ros_bridge"

# ── 4. Build mayara-server binary ─────────────────────────────────────────────
info "Step 4/6 — Building mayara-server (Rust)..."
MAYARA_BIN="$STACK_DIR/mayara-server/target/release/mayara-server"
if [ -f "$MAYARA_BIN" ]; then
    info "  mayara-server binary already exists, skipping build."
else
    # Try to download a pre-built ARM64 binary first
    LATEST_TAG=$(curl -s https://api.github.com/repos/MarineYachtRadar/mayara-server/releases/latest \
        | grep '"tag_name"' | cut -d'"' -f4 || echo "")

    if [ -n "$LATEST_TAG" ]; then
        BIN_URL="https://github.com/MarineYachtRadar/mayara-server/releases/download/${LATEST_TAG}/mayara-server-linux-arm64"
        info "  Downloading pre-built binary ($LATEST_TAG)..."
        if sudo -u "$INSTALL_USER" wget -q "$BIN_URL" -O "$MAYARA_BIN" 2>/dev/null; then
            chmod +x "$MAYARA_BIN"
            chown "$INSTALL_USER:$INSTALL_USER" "$MAYARA_BIN"
            info "  Binary downloaded successfully."
        else
            warn "  Pre-built binary not available for this release. Building from source..."
            LATEST_TAG=""
        fi
    fi

    if [ -z "$LATEST_TAG" ] || [ ! -f "$MAYARA_BIN" ]; then
        info "  Installing Rust toolchain (this takes a few minutes)..."
        sudo -u "$INSTALL_USER" bash -c \
            'curl --proto "=https" --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path'
        export PATH="$INSTALL_HOME/.cargo/bin:$PATH"
        info "  Building mayara-server from source (10–20 min on Pi 4)..."
        sudo -u "$INSTALL_USER" bash -c \
            "source $INSTALL_HOME/.cargo/env && cd $STACK_DIR/mayara-server && cargo build --release"
    fi
fi

# ── 5. Build ROS2 workspace ───────────────────────────────────────────────────
info "Step 5/6 — Building ROS2 workspace..."

# Set up workspace source directory
sudo -u "$INSTALL_USER" mkdir -p "$ROS2_WS/src"

# Clone ROS2 packages into workspace/src
clone_or_pull \
    https://github.com/SeawardScience/marine_sensor_msgs.git \
    "$ROS2_WS/src/marine_sensor_msgs"

# Link the KAHU ROS2 packages into the workspace
for pkg in mayara_ros_bridge echoflow_lite; do
    if [ ! -d "$ROS2_WS/src/$pkg" ]; then
        # Clone from KAHU-radar org if not yet a symlink
        case "$pkg" in
            mayara_ros_bridge)
                clone_or_pull \
                    https://github.com/KAHU-radar/mayara-ros-bridge.git \
                    "$ROS2_WS/src/$pkg"
                ;;
            echoflow_lite)
                clone_or_pull \
                    https://github.com/KAHU-radar/echoflow-lite.git \
                    "$ROS2_WS/src/$pkg"
                ;;
        esac
    fi
done

# Python deps
info "  Installing Python dependencies..."
pip3 install --break-system-packages websocket-client numpy scipy scikit-learn requests

# Install emulator venv deps
sudo -u "$INSTALL_USER" bash -c \
    "cd $STACK_DIR/radar-emulator-navico && python3 -m venv venv && venv/bin/pip install -q numpy"

# Build the workspace
info "  Running colcon build (this takes a few minutes)..."
sudo -u "$INSTALL_USER" bash -c \
    "source /opt/ros/jazzy/setup.bash && cd $ROS2_WS && colcon build --symlink-install 2>&1 | tail -5"

# Source workspace in .bashrc
sudo -u "$INSTALL_USER" bash -c \
    "grep -q 'ros2_ws/install/setup.bash' ~/.bashrc || echo 'source $ROS2_WS/install/setup.bash' >> ~/.bashrc"

# Copy node scripts into workspace dir so they can be found at known paths
cp "$STACK_DIR/kahu-stack/nmea_vessel_node.py" "$ROS2_WS/nmea_vessel_node.py" 2>/dev/null || \
    cp "$STACK_DIR/nmea_vessel_node.py"         "$ROS2_WS/nmea_vessel_node.py" 2>/dev/null || true
cp "$STACK_DIR/kahu-stack/kahu_uploader_node.py" "$ROS2_WS/kahu_uploader_node.py" 2>/dev/null || \
    cp "$STACK_DIR/kahu_uploader_node.py"         "$ROS2_WS/kahu_uploader_node.py" 2>/dev/null || true

chown "$INSTALL_USER:$INSTALL_USER" "$ROS2_WS"/*.py 2>/dev/null || true

# ── 6. Install systemd services ───────────────────────────────────────────────
info "Step 6/6 — Installing systemd services..."

SYSTEMD_SRC="$STACK_DIR/kahu-stack/systemd"

# Substitute the actual install user into service files before copying
for svc in mayara-server radar-emulator mayara-ros-bridge kahu-stack nmea-vessel kahu-uploader; do
    src="$SYSTEMD_SRC/${svc}.service"
    dest="/etc/systemd/system/${svc}.service"
    if [ -f "$src" ]; then
        sed "s|/home/pi|$INSTALL_HOME|g" "$src" > "$dest"
        info "  Installed $dest"
    else
        warn "  Service file not found: $src (skipping)"
    fi
done

# Install environment config if not already present
if [ ! -f /etc/default/kahu ]; then
    if [ -f "$SYSTEMD_SRC/kahu.env.template" ]; then
        cp "$SYSTEMD_SRC/kahu.env.template" /etc/default/kahu
        info "  Created /etc/default/kahu from template"
    fi
fi

systemctl daemon-reload

# Enable production services (real vessel)
for svc in mayara-server mayara-ros-bridge kahu-stack nmea-vessel kahu-uploader; do
    systemctl enable "$svc" 2>/dev/null && info "  Enabled: $svc"
done

# radar-emulator is NOT enabled by default (real hardware deployment)
systemctl disable radar-emulator 2>/dev/null || true
info "  radar-emulator service installed but NOT enabled (enable manually for dev testing)"

# ── Done ─────────────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}======================================================${NC}"
echo -e "${GREEN}  KAHU Stack installation complete!${NC}"
echo -e "${GREEN}======================================================${NC}"
echo ""
echo "Next steps:"
echo ""
echo "  1. Edit /etc/default/kahu to set your vessel config:"
echo "       sudo nano /etc/default/kahu"
echo "     Key settings:"
echo "       RADAR_INTERFACE=eth0   ← run 'ip link' to find your interface"
echo "       KAHU_VESSEL_ID=vessel-001"
echo "       KAHU_UPLOAD_URL=https://your-backend/api/tracks"
echo ""
echo "  2. Reboot to start all services:"
echo "       sudo reboot"
echo ""
echo "  3. After reboot, check service status:"
echo "       sudo systemctl status mayara-server"
echo "       sudo systemctl status kahu-stack"
echo "       journalctl -u kahu-stack -f   ← follow logs"
echo ""
echo "  4. View radar in Foxglove:"
echo "       Connect to ws://\$(hostname -I | awk '{print \$1}'):8765"
echo ""
echo "  For development testing (no radar hardware):"
echo "       sudo systemctl enable radar-emulator"
echo "       sudo systemctl disable mayara-server"
echo "       # Edit mayara-server.service to use --interface lo"
echo "       sudo reboot"
echo ""
