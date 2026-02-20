#!/usr/bin/env python3
"""
KAHU NMEA Vessel Position ROS2 Node
=====================================
Runs on the Raspberry Pi.
Listens for NMEA sentences (UDP port 10110) from the Mac relay,
parses $GPRMC and $GPHDG, and publishes to ROS2 topics:

    /vessel/gps      → sensor_msgs/NavSatFix
    /vessel/heading  → std_msgs/Float64

These topics are visible in Foxglove alongside your radar targets.

Install:
    Place in ~/kahu_stack/mayara_ros_bridge/kahu_nmea_listener/
    and add to your ROS2 workspace CMakeLists / setup.py

Run standalone (for testing):
    source ~/ros2_ws/install/setup.bash
    python3 nmea_vessel_node.py
"""

import socket
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

LISTEN_PORT = 10110


def parse_gprmc(fields):
    """
    Parse $GPRMC fields.
    Returns (lat, lon, speed_kts, heading) or None if invalid.
    """
    try:
        if fields[2] != 'A':  # status must be Active
            return None

        lat_raw = float(fields[3])
        lat_hem = fields[4]
        lon_raw = float(fields[5])
        lon_hem = fields[6]

        lat_deg = int(lat_raw / 100)
        lat_min = lat_raw - lat_deg * 100
        lat = lat_deg + lat_min / 60.0
        if lat_hem == 'S':
            lat = -lat

        lon_deg = int(lon_raw / 100)
        lon_min = lon_raw - lon_deg * 100
        lon = lon_deg + lon_min / 60.0
        if lon_hem == 'W':
            lon = -lon

        speed_kts = float(fields[7]) if fields[7] else 0.0
        heading = float(fields[8]) if fields[8] else 0.0

        return lat, lon, speed_kts, heading
    except (ValueError, IndexError):
        return None


def parse_gphdg(fields):
    """
    Parse $GPHDG/$HCHDG fields.
    Returns heading in degrees or None.
    """
    try:
        return float(fields[1])
    except (ValueError, IndexError):
        return None


class NmeaVesselNode(Node):
    def __init__(self):
        super().__init__('kahu_nmea_vessel')

        # Publishers
        self.gps_pub = self.create_publisher(NavSatFix, '/vessel/gps', 10)
        self.hdg_pub = self.create_publisher(Float64, '/vessel/heading', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/vessel/velocity', 10)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', LISTEN_PORT))
        self.sock.settimeout(0.1)

        # State
        self.lat = None
        self.lon = None
        self.heading = 0.0
        self.speed_kts = 0.0

        # Poll the UDP socket at 10 Hz
        self.create_timer(0.1, self.poll_nmea)

        self.get_logger().info(
            f'KAHU NMEA Vessel Node started — listening on UDP:{LISTEN_PORT}'
        )

    def poll_nmea(self):
        try:
            data, addr = self.sock.recvfrom(4096)
        except socket.timeout:
            return

        for raw_line in data.decode('ascii', errors='ignore').splitlines():
            line = raw_line.strip()
            if not line:
                continue

            # Strip checksum
            if '*' in line:
                line = line[:line.index('*')]

            fields = line.split(',')
            sentence_type = fields[0].lstrip('$!').upper()

            if sentence_type == 'GPRMC' or sentence_type == 'GNRMC':
                result = parse_gprmc(fields)
                if result:
                    self.lat, self.lon, self.speed_kts, hdg = result
                    if hdg > 0:
                        self.heading = hdg
                    self.publish_gps()
                    self.publish_velocity()

            elif sentence_type in ('GPHDG', 'HCHDG', 'GPHDG', 'GPHDM'):
                hdg = parse_gphdg(fields)
                if hdg is not None:
                    self.heading = hdg
                    self.publish_heading()

    def publish_gps(self):
        if self.lat is None:
            return
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'vessel'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = self.lat
        msg.longitude = self.lon
        msg.altitude = 0.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.gps_pub.publish(msg)
        self.get_logger().debug(
            f'GPS: {self.lat:.6f}, {self.lon:.6f}'
        )

    def publish_heading(self):
        msg = Float64()
        msg.data = self.heading
        self.hdg_pub.publish(msg)

    def publish_velocity(self):
        # Convert knots to m/s
        speed_ms = self.speed_kts * 0.514444
        heading_rad = math.radians(self.heading)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'vessel'
        msg.twist.linear.x = speed_ms * math.sin(heading_rad)
        msg.twist.linear.y = speed_ms * math.cos(heading_rad)
        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NmeaVesselNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
