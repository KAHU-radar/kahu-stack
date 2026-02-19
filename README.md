# KAHU Stack — Setup Guide

Complete installation and run instructions for the KAHU marine radar perception stack on a Raspberry Pi.

---

## System Overview

```
Radar Hardware (or Emulator)
        │  UDP multicast (Navico/Halo protocol)
        ▼
Mayara Server  (Rust)          ← discovers radar, exposes WebSocket API
        │  ws://<pi-ip>:6502/v2/api/radars/<id>/spokes
        ▼
mayara_ros_bridge  (Python)    ← decodes protobuf spokes → ROS2 topics
        │  /aura/perception/sensors/halo_a/data  (~33 Hz)
        ▼
echoflow_lite  (Python)        ← threshold → cluster → track targets
        │  /aura/perception/sensors/halo_a/lite/targets
        ▼
foxglove_bridge                ← WebSocket visualization
        │  ws://<pi-ip>:8765
        ▼
Foxglove App  (your laptop)    ← view radar targets remotely
```

---

## 1. Prerequisites

### Hardware
- Raspberry Pi 4 or 5 (4 GB RAM minimum recommended)
- Navico/Halo radar connected to the same network as the Pi
- OR: run the included radar emulator for testing without hardware

### Operating System
- Ubuntu 24.04 LTS (64-bit ARM) — **required** for ROS2 Jazzy
- Flash to SD card: https://ubuntu.com/download/raspberry-pi

### Network
- Pi and radar (or your laptop) must be on the same LAN
- Note your Pi's IP address: `hostname -I`

---

## 2. OS & ROS2 Installation

### 2a. System update

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl python3-pip build-essential
```

### 2b. Install ROS2 Jazzy

```bash
# Add ROS2 apt repo
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions \
  ros-jazzy-tf2-ros ros-jazzy-foxglove-bridge
```

### 2c. Add ROS2 to your shell

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Install Mayara Server

Mayara connects to the radar hardware and streams spoke data over WebSocket.

```bash
mkdir -p ~/kahu-stack
cd ~/kahu-stack

git clone https://github.com/MarineYachtRadar/mayara-server.git
cd mayara-server
```

**Option A — Build from source (10–20 min on Pi):**
```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

cargo build --release
```

**Option B — Download pre-built ARM64 binary (faster):**

Check https://github.com/MarineYachtRadar/mayara-server/releases for the latest
`mayara-server-linux-arm64` release asset and download it directly:

```bash
# Replace <version> with the latest release tag shown on GitHub
wget https://github.com/MarineYachtRadar/mayara-server/releases/download/<version>/mayara-server-linux-arm64 \
  -O ~/kahu-stack/mayara-server/target/release/mayara-server
chmod +x ~/kahu-stack/mayara-server/target/release/mayara-server
```

---

## 4. Install the Radar Emulator (for testing without hardware)

```bash
cd ~/kahu-stack
git clone https://github.com/kahu-app/radar-emulator-navico.git
cd radar-emulator-navico

python3 -m venv venv
source venv/bin/activate
pip install numpy
deactivate
```

---

## 5. Set Up the ROS2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 5a. Clone marine_sensor_msgs (required message types)

```bash
git clone https://github.com/SeawardScience/marine_sensor_msgs.git
```

### 5b. Clone the KAHU packages

```bash
git clone https://github.com/kahu-app/mayara-ros-bridge.git mayara_ros_bridge
git clone https://github.com/kahu-app/echoflow-lite.git echoflow_lite
```

### 5c. Install Python dependencies

```bash
pip3 install websocket-client numpy scipy scikit-learn
```

### 5d. Build the workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

This will take a few minutes. If it succeeds you should see:
```
Summary: X packages finished [...]
```

### 5e. Source the workspace

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 6. Copy the Launch File

```bash
cp ~/kahu-stack/kahu_stack.launch.py ~/ros2_ws/
# Or if it came from the repo, it will already be there.
```

---

## 7. Running the Stack

You need **4 terminals** open on the Pi (use `tmux` or `screen` to manage them over SSH).

### Recommended: use tmux

```bash
# Install tmux if needed
sudo apt install -y tmux

# Start a session
tmux new-session -s kahu

# Split into panes: Ctrl+B then " (horizontal) or % (vertical)
# Navigate between panes: Ctrl+B then arrow keys
```

---

### Terminal 1 — Mayara Server (radar interface)

**With real radar hardware:**
```bash
cd ~/kahu-stack/mayara-server
./target/release/mayara-server --interface wlan0 --brand navico --allow-wifi -v
```

> Replace `wlan0` with your actual network interface (`ip link` to list them).
> Use `--brand furuno` or `--brand raymarine` for other hardware.

**With emulator (no hardware needed):**
```bash
# Start the emulator first (Terminal 2), then run mayara in loopback mode
cd ~/kahu-stack/mayara-server
./target/release/mayara-server --interface lo --brand navico -v
```

You should see: `Radar discovered: ...` when it finds the radar.

---

### Terminal 2 — Radar Emulator (skip if using real hardware)

```bash
cd ~/kahu-stack/radar-emulator-navico
source venv/bin/activate
python3 emulate_with_arpa.py
```

You should see: spokes being emitted at ~2048/revolution.

---

### Terminal 3 — Bridge (Mayara → ROS2)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run mayara_ros_bridge mayara_ros_bridge
```

You should see: `Connected to ws://...` and spoke data flowing.

**Verify in a separate terminal:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic hz /aura/perception/sensors/halo_a/data
# Expected: ~33 Hz
```

---

### Terminal 4 — EchoFlow Lite + Foxglove (via launch file)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ~/ros2_ws/kahu_stack.launch.py
```

Optional parameter overrides:
```bash
ros2 launch ~/ros2_ws/kahu_stack.launch.py \
  intensity_threshold:=0.2 \
  dbscan_eps:=200.0 \
  foxglove_port:=8765
```

---

## 8. Viewing in Foxglove (your laptop)

1. Download Foxglove Studio: https://foxglove.dev/download
2. Open → **Open connection** → **Foxglove WebSocket**
3. URL: `ws://<pi-ip-address>:8765`
4. Subscribe to topics:
   - `/aura/perception/sensors/halo_a/data` — raw radar sectors
   - `/aura/perception/sensors/halo_a/lite/targets` — detected targets (PointCloud2)
   - `/aura/nav/odom` — ship position

> Find your Pi's IP with `hostname -I` on the Pi.
> Both your laptop and the Pi must be on the **same LAN** or connected via VPN.

---

## 9. Verification Checklist

Run these on the Pi to confirm everything is working:

```bash
# List all active topics
ros2 topic list

# Expected output should include:
# /aura/perception/sensors/halo_a/data
# /aura/perception/sensors/halo_a/lite/targets
# /aura/nav/odom
# /tf_static

# Check data rates
ros2 topic hz /aura/perception/sensors/halo_a/data        # ~33 Hz
ros2 topic hz /aura/perception/sensors/halo_a/lite/targets # ~1 Hz (per revolution)

# Inspect a single message
ros2 topic echo /aura/perception/sensors/halo_a/data --once
ros2 topic echo /aura/perception/sensors/halo_a/lite/targets --once
```

---

## 10. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Mayara finds no radar | Wrong network interface or brand | Check `ip link`, try `--interface eth0`, verify radar is on same LAN |
| Bridge can't connect | Mayara not running or wrong radar ID | Check the radar ID in mayara output, update `MAYARA_WS_URL` |
| No ROS2 topics | Workspace not sourced | `source ~/ros2_ws/install/setup.bash` |
| Foxglove shows no data | Wrong Pi IP or port blocked | Confirm IP with `hostname -I`, check firewall: `sudo ufw allow 8765` |
| `colcon build` fails | Missing dependencies | `rosdep install --from-paths src --ignore-src -r -y` |
| High CPU on Pi | DBSCAN params too loose | Increase `intensity_threshold`, decrease `dbscan_eps` |

---

## 11. Remote Access for Team Members

To work remotely without being on the same LAN, the Pi owner (you) needs to set up one of:

**Option A — Tailscale (easiest, recommended for team use)**
```bash
# On the Pi:
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
# Share the Tailscale IP with teammates — it works over the internet
```

**Option B — SSH tunnel**
```bash
# Teammates run this on their laptop to forward the Foxglove port:
ssh -L 8765:<pi-local-ip>:8765 pi@<your-public-ip>
# Then connect Foxglove to ws://localhost:8765
```

---

## Component Versions

| Component | Language | Source |
|---|---|---|
| mayara-server | Rust | github.com/MarineYachtRadar/mayara-server |
| radar-emulator-navico | Python | github.com/kahu-app/radar-emulator-navico |
| marine_sensor_msgs | C++ | github.com/SeawardScience/marine_sensor_msgs |
| mayara_ros_bridge | Python | github.com/kahu-app/mayara-ros-bridge |
| echoflow_lite | Python | github.com/kahu-app/echoflow-lite |
| ROS2 | — | Jazzy (Ubuntu 24.04) |
| Foxglove Studio | — | foxglove.dev |
