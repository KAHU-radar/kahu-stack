# KAHU Stack — Deployment Guide

Complete reference for deploying KAHU on a real vessel, managing systemd services, and migrating to the cloud backend.

---

## Contents

1. [Emulator → Real Hardware](#1-emulator--real-hardware)
2. [Boot-time Services (systemd)](#2-boot-time-services-systemd)
3. [Cloud Backend Migration](#3-cloud-backend-migration-sqlite--postgresql--cloud-run)
4. [New Pi Install Script](#4-new-pi-install-script)
5. [Troubleshooting](#5-troubleshooting)

---

## 1. Emulator → Real Hardware

### What changes

| Aspect | Emulator (dev) | Real Navico/Halo radar |
|---|---|---|
| Radar discovery | Emulator multicasts on loopback (`lo`) | Radar multicasts on ethernet subnet |
| Mayara interface | `--interface lo` | `--interface eth0` (or actual iface name) |
| Emulator process | `python3 emulate_with_arpa.py` running | Not needed — disable/stop the service |
| Network topology | All on the same loopback | Pi and radar on same ethernet switch |
| Radar discovery time | Instant | 5–15 seconds after mayara-server starts |

### Step-by-step

**1. Find your ethernet interface**

```bash
ip link show
# Look for the interface connected to the radar network, e.g. eth0, enp2s0
```

**2. Update `/etc/default/kahu`**

```bash
sudo nano /etc/default/kahu
```

Set:
```
RADAR_INTERFACE=eth0     # ← your actual interface
KAHU_VESSEL_ID=vessel-001
```

**3. Disable the emulator, ensure mayara-server is enabled**

```bash
sudo systemctl disable radar-emulator
sudo systemctl stop radar-emulator
sudo systemctl enable mayara-server
sudo systemctl start mayara-server
```

**4. Verify radar discovery**

```bash
journalctl -u mayara-server -f
# You should see: "Radar discovered: ..." within 5–15 seconds of the radar powering on
```

**5. That's it.** No other changes needed. Mayara auto-discovers the radar ID over the multicast network.

### Network requirements

- Pi and radar must be on the **same ethernet subnet** (same switch is ideal)
- No firewall should block UDP multicast between the Pi and the radar
- Typical Navico/Halo multicast groups used: `236.6.7.4`, `236.6.7.8`, `236.6.7.9`
- If using a managed switch, ensure IGMP snooping is either disabled or correctly configured

### Checking radar connectivity

```bash
# Verify multicast traffic is arriving on the interface
sudo tcpdump -i eth0 -n host 236.6.7.9
# You should see packets from the radar's IP at ~33 Hz when the radar is spinning
```

---

## 2. Boot-time Services (systemd)

All Pi processes are managed as systemd services. They start automatically on boot and restart on failure.

### Service overview

| Service | What it runs | Enabled by default |
|---|---|---|
| `mayara-server` | Navico/Halo radar interface | Yes (real vessel) |
| `radar-emulator` | Radar emulator (dev only) | No |
| `mayara-ros-bridge` | Mayara → ROS2 spoke decoder | Yes |
| `kahu-stack` | EchoFlow Lite + Foxglove bridge | Yes |
| `nmea-vessel` | NMEA GPS/heading publisher | Yes |
| `kahu-uploader` | Track uploader to cloud backend | Yes |

### Start order

```
network-online.target
    ├── mayara-server      (or radar-emulator on dev)
    │       └── mayara-ros-bridge  (waits 3s for radar discovery)
    │               └── kahu-stack  (EchoFlow + Foxglove)
    ├── nmea-vessel        (independent — just needs network)
    └── kahu-uploader      (depends on kahu-stack + nmea-vessel)
```

### Common commands

```bash
# Check status of all KAHU services at once
sudo systemctl status mayara-server mayara-ros-bridge kahu-stack nmea-vessel kahu-uploader

# Follow logs for a specific service
journalctl -u kahu-stack -f
journalctl -u mayara-server -f

# Restart a service after config change
sudo systemctl restart mayara-server

# Stop everything
sudo systemctl stop kahu-uploader kahu-stack mayara-ros-bridge mayara-server nmea-vessel

# Start everything
sudo systemctl start mayara-server mayara-ros-bridge kahu-stack nmea-vessel kahu-uploader
```

### Environment config

All services read `/etc/default/kahu`. Edit this file to change vessel-specific settings — you do not need to modify the service files themselves.

```bash
sudo nano /etc/default/kahu
```

```bash
# Example /etc/default/kahu for a production vessel
RADAR_INTERFACE=eth0
MAYARA_HOST=localhost:6502
KAHU_UPLOAD_URL=https://kahu-backend-xxxx-uc.a.run.app/api/tracks
KAHU_VESSEL_ID=sv-pacific-dream
```

After editing, reload:
```bash
sudo systemctl daemon-reload
sudo systemctl restart kahu-uploader   # uploader reads KAHU_UPLOAD_URL
sudo systemctl restart mayara-server   # reads RADAR_INTERFACE
```

### Switching between emulator and real hardware

**Development (emulator):**
```bash
sudo systemctl disable mayara-server
sudo systemctl enable radar-emulator
# Edit /etc/systemd/system/mayara-ros-bridge.service:
#   change --interface eth0 to --interface lo in mayara-server, or
#   start mayara-server manually with: mayara-server --interface lo --brand navico -v
sudo systemctl daemon-reload
sudo reboot
```

**Production (real radar):**
```bash
sudo systemctl disable radar-emulator
sudo systemctl enable mayara-server
sudo reboot
```

---

## 3. Cloud Backend Migration (SQLite → PostgreSQL + Cloud Run)

### Current architecture (development)

```
Pi (kahu_uploader_node.py)
    → POST http://<mac-ip>:8000/api/tracks
    → kahu_backend.py (FastAPI + SQLite, runs on Mac)
    → http://localhost:8000  (dashboard, Mac only)
```

### Target architecture (production)

```
Pi (kahu_uploader_node.py)
    → POST https://kahu-backend-xxxx-uc.a.run.app/api/tracks
    → Cloud Run (FastAPI container, auto-scales)
    → Cloud SQL PostgreSQL (managed database)
    → https://kahu.earth  (dashboard, public)
```

### Migration steps

#### 3a. Provision Cloud SQL PostgreSQL (Google Cloud Console)

```bash
# Install Google Cloud SDK if not already done
# https://cloud.google.com/sdk/docs/install

gcloud sql instances create kahu-db \
    --database-version=POSTGRES_15 \
    --tier=db-f1-micro \
    --region=us-central1

gcloud sql databases create kahu --instance=kahu-db

gcloud sql users set-password postgres \
    --instance=kahu-db \
    --password=<YOUR_STRONG_PASSWORD>
```

#### 3b. Update `kahu_backend.py` for PostgreSQL

Replace the SQLite connection in `kahu_backend.py` with:

```python
# requirements: pip install asyncpg sqlalchemy psycopg2-binary
import os
from sqlalchemy import create_engine

DATABASE_URL = os.environ.get(
    "DATABASE_URL",
    "postgresql://postgres:<password>@/<db>?host=/cloudsql/<project>:<region>:<instance>"
)
engine = create_engine(DATABASE_URL)
```

The schema is unchanged — only the connection string moves from SQLite to PostgreSQL.

#### 3c. Containerise the backend

```dockerfile
# Dockerfile (alongside kahu_backend.py)
FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY kahu_backend.py .
CMD ["uvicorn", "kahu_backend:app", "--host", "0.0.0.0", "--port", "8080"]
```

```txt
# requirements.txt
fastapi
uvicorn[standard]
sqlalchemy
psycopg2-binary
asyncpg
```

#### 3d. Deploy to Cloud Run

```bash
# From the directory containing kahu_backend.py and Dockerfile

gcloud run deploy kahu-backend \
    --source . \
    --region us-central1 \
    --allow-unauthenticated \
    --set-env-vars="DATABASE_URL=postgresql://..." \
    --add-cloudsql-instances=<PROJECT>:<REGION>:<INSTANCE>
```

Cloud Run gives you a URL like `https://kahu-backend-xxxx-uc.a.run.app`.

#### 3e. Update the Pi

```bash
sudo nano /etc/default/kahu
# Set:
# KAHU_UPLOAD_URL=https://kahu-backend-xxxx-uc.a.run.app/api/tracks

sudo systemctl restart kahu-uploader
journalctl -u kahu-uploader -f   # confirm uploads succeed
```

#### 3f. Cost estimate

| Resource | Tier | Estimated monthly cost |
|---|---|---|
| Cloud Run | First 2M requests free | $0–5 |
| Cloud SQL (PostgreSQL) | db-f1-micro | ~$10 |
| Egress | Minimal (JSON payloads) | <$1 |
| **Total** | | **~$10–15/month** |

Use `db-g1-small` for multiple active vessels.

---

## 4. New Pi Install Script

A single script handles everything on a fresh Pi:

```bash
# On the Pi (as root):
curl -fsSL https://raw.githubusercontent.com/KAHU-radar/kahu-stack/master/install.sh | sudo bash
```

Then:
```bash
sudo nano /etc/default/kahu   # set RADAR_INTERFACE, KAHU_VESSEL_ID, KAHU_UPLOAD_URL
sudo reboot
```

**What the script does:**
1. Installs system packages and ROS2 Jazzy
2. Clones all KAHU repositories
3. Downloads pre-built mayara-server ARM64 binary (falls back to Rust build)
4. Builds the ROS2 workspace with colcon
5. Installs all systemd services and enables the production set
6. Creates `/etc/default/kahu` from the template

See `install.sh` for full source.

---

## 5. Troubleshooting

### Radar not discovered

```bash
# Check mayara-server logs
journalctl -u mayara-server -f

# Verify multicast traffic is arriving
sudo tcpdump -i eth0 -n host 236.6.7.9   # should see packets at ~33 Hz

# Try a different interface
sudo systemctl stop mayara-server
/home/pi/kahu-stack/mayara-server/target/release/mayara-server \
    --interface eth0 --brand navico -v   # watch output for discovery
```

### No ROS2 topics publishing

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list   # should show /aura/... topics

# If empty, check bridge
journalctl -u mayara-ros-bridge -f
```

### Bridge can't connect to mayara-server

```bash
# Verify mayara is up and responding
curl http://localhost:6502/v2/api/radars
# Should return JSON with radar info
```

### Uploader not sending data

```bash
journalctl -u kahu-uploader -f
# Look for: "Uploading N records to <URL>"
# Or: "No internet — records buffered locally"

# Test endpoint manually
curl -X POST $KAHU_UPLOAD_URL \
    -H 'Content-Type: application/json' \
    -d '{"vessel_id":"test","tracks":[]}'
```

### Service won't start

```bash
systemctl status <service-name>   # shows last error
journalctl -u <service-name> -n 50 --no-pager
```

### Foxglove shows no data

```bash
# Confirm Foxglove bridge is up
ros2 topic list | grep -i foxglove   # should see rosout at minimum
# Try connecting to ws://<pi-ip>:8765
# Check firewall
sudo ufw allow 8765
```

---

## Appendix: Terminal Reference (Manual Operation)

For running individual components without systemd (useful for debugging):

| Terminal | Command | Purpose |
|---|---|---|
| 1 | `cd ~/kahu-stack/mayara-server && ./target/release/mayara-server --interface eth0 --brand navico -v` | Real radar |
| 1 (dev) | `cd ~/kahu-stack/radar-emulator-navico && source venv/bin/activate && python3 emulate_with_arpa.py` | Emulator |
| 2 (dev) | `cd ~/kahu-stack/mayara-server && ./target/release/mayara-server --interface lo --brand navico -v` | Mayara (loopback) |
| 3 | `cd ~/kahu-stack/mayara_ros_bridge && ./run_bridge.sh` | Mayara→ROS2 bridge |
| 4 | `source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch echoflow_lite kahu_stack.launch.py` | EchoFlow + Foxglove |
| 5 | `source ~/ros2_ws/install/setup.bash && python3 ~/kahu-stack/kahu-stack/nmea_vessel_node.py` | NMEA vessel position |
| 6 | `source ~/ros2_ws/install/setup.bash && python3 ~/kahu-stack/kahu-stack/kahu_uploader_node.py` | Track uploader |
