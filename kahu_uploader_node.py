#!/usr/bin/env python3
"""
KAHU Track Uploader ROS2 Node
==============================
Runs on the Raspberry Pi.

Reads:
    /aura/.../lite/targets   → EchoFlow radar tracks
    /vessel/gps              → vessel position (from nmea_vessel_node)
    /vessel/heading          → vessel heading

Buffers locally to SQLite when offline, then flushes to:
    - Local TimescaleDB/FastAPI (for development/testing)
    - kahu.earth REST endpoint (swap UPLOAD_URL when ready)

To switch to kahu.earth:
    Change UPLOAD_URL below, or set env var KAHU_UPLOAD_URL

Install deps:
    pip3 install requests --break-system-packages

Run:
    source ~/ros2_ws/install/setup.bash
    KAHU_UPLOAD_URL=https://your-endpoint.com/api/tracks python3 kahu_uploader_node.py
"""

import os
import json
import time
import sqlite3
import threading
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# ─── Configuration ────────────────────────────────────────────────────────────
UPLOAD_URL = os.environ.get(
    'KAHU_UPLOAD_URL',
    'http://localhost:8000/api/tracks'   # local FastAPI during dev
)
VESSEL_ID = os.environ.get('KAHU_VESSEL_ID', 'vessel-001')
UPLOAD_INTERVAL_S = 5.0       # how often to flush buffer to cloud
BUFFER_DB_PATH = '/tmp/kahu_track_buffer.db'
MAX_BATCH_SIZE = 100           # max records per upload batch

# EchoFlow lite target topic — adjust if your topic name differs
ECHOFLOW_TOPIC = '/aura/perception/sensors/halo_a/lite/targets'

# ──────────────────────────────────────────────────────────────────────────────


def init_buffer_db(path: str) -> sqlite3.Connection:
    """Initialize local SQLite buffer for offline storage."""
    conn = sqlite3.connect(path, check_same_thread=False)
    conn.execute('''
        CREATE TABLE IF NOT EXISTS track_buffer (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp   REAL NOT NULL,
            payload     TEXT NOT NULL,
            uploaded    INTEGER DEFAULT 0
        )
    ''')
    conn.commit()
    return conn


class KahuUploaderNode(Node):
    def __init__(self):
        super().__init__('kahu_uploader')

        # Local state
        self.vessel_lat = None
        self.vessel_lon = None
        self.vessel_heading = 0.0
        self.lock = threading.Lock()

        # SQLite buffer
        self.db = init_buffer_db(BUFFER_DB_PATH)

        # Subscriptions
        self.gps_sub = self.create_subscription(
            NavSatFix, '/vessel/gps', self.on_gps, 10
        )
        self.hdg_sub = self.create_subscription(
            Float64, '/vessel/heading', self.on_heading, 10
        )

        # Try to subscribe to EchoFlow targets
        # Using a generic String subscription since the exact message type
        # depends on your echoflow_lite package — adjust msg type as needed
        try:
            #from sensor_msgs.msg import PointCloud2
            #import sensor_msgs_py.point_cloud2 as pc2

            self.target_sub = self.create_subscription(
                PointCloud2,
                ECHOFLOW_TOPIC,
                self.on_targets,
                10
            )
            self.get_logger().info(f'Subscribed to {ECHOFLOW_TOPIC}')
        except Exception as e:
            self.get_logger().warn(f'Could not subscribe to targets: {e}')

        # Upload timer
        self.create_timer(UPLOAD_INTERVAL_S, self.flush_to_cloud)

        self.get_logger().info(
            f'KAHU Uploader started\n'
            f'  Vessel ID : {VESSEL_ID}\n'
            f'  Upload URL: {UPLOAD_URL}\n'
            f'  Buffer DB : {BUFFER_DB_PATH}'
        )

    def on_gps(self, msg: NavSatFix):
        with self.lock:
            self.vessel_lat = msg.latitude
            self.vessel_lon = msg.longitude

    def on_heading(self, msg: Float64):
        with self.lock:
            self.vessel_heading = msg.data

    def on_targets(self, msg: PointCloud2):
        """
        Called each time EchoFlow publishes a target update.
        Buffers the track to SQLite immediately.
        """
        with self.lock:
            vessel_lat = self.vessel_lat
            vessel_lon = self.vessel_lon
            heading = self.vessel_heading

        try:
            targets = []
            for point in pc2.read_points(msg, field_names=None, skip_nans=True):
                fields = msg.fields
                names  = [f.name for f in fields]
                target = {k: float(v) if hasattr(v, 'item') else v for k, v in zip(names, point)}
                targets.append(target)

            if not targets:
                return

            ts = time.time()
            for target in targets:
                record = {
                    'vessel_id': VESSEL_ID,
                    'timestamp': ts,
                    'vessel_position': {
                        'lat': vessel_lat,
                        'lon': vessel_lon,
                        'heading_deg': heading,
                    },
                    'target': target,
                }
                self.db.execute(
                    'INSERT INTO track_buffer (timestamp, payload) VALUES (?, ?)',
                    (ts, json.dumps(record))
                )
            self.db.commit()
            self.get_logger().info(f'Buffered {len(targets)} targets')

        except Exception as e:
            self.get_logger().warn(f'Failed to buffer target: {e}')

    def flush_to_cloud(self):
        """Upload buffered records to the cloud endpoint."""
        rows = self.db.execute(
            'SELECT id, payload FROM track_buffer WHERE uploaded=0 ORDER BY id LIMIT ?',
            (MAX_BATCH_SIZE,)
        ).fetchall()

        if not rows:
            return

        ids = [r[0] for r in rows]
        batch = [json.loads(r[1]) for r in rows]

        self.get_logger().info(
            f'Uploading {len(batch)} records to {UPLOAD_URL}...'
        )

        try:
            resp = requests.post(
                UPLOAD_URL,
                json={'vessel_id': VESSEL_ID, 'tracks': batch},
                timeout=10,
                headers={'Content-Type': 'application/json'},
            )
            if resp.status_code in (200, 201, 204):
                self.db.execute(
                    f'UPDATE track_buffer SET uploaded=1 WHERE id IN ({",".join("?" * len(ids))})',
                    ids
                )
                self.db.commit()
                self.get_logger().info(f'  ✓ Uploaded {len(batch)} records')
            else:
                self.get_logger().warn(
                    f'  ✗ Upload failed: HTTP {resp.status_code} — will retry'
                )
        except requests.exceptions.ConnectionError:
            self.get_logger().warn(
                '  ✗ No internet — records buffered locally, will retry'
            )
        except Exception as e:
            self.get_logger().warn(f'  ✗ Upload error: {e}')

        # Prune old uploaded records (keep last 10k)
        self.db.execute(
            'DELETE FROM track_buffer WHERE uploaded=1 AND id < '
            '(SELECT id FROM track_buffer ORDER BY id DESC LIMIT 1 OFFSET 10000)'
        )
        self.db.commit()


def main(args=None):
    rclpy.init(args=args)
    node = KahuUploaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
