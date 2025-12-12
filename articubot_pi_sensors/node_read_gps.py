#!/usr/bin/env python3
# articubot_pi_sensors/node_read_gps.py

import re
import math
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import hw_config as cfg


def nmea_to_decimal(deg_min, direction):
    """
    Convert NMEA degree/minute format (ddmm.mmmm) to decimal degrees.
    """
    if not deg_min:
        return math.nan
    try:
        val = float(deg_min)
        degrees = int(val / 100)
        minutes = val - degrees * 100
        dec = degrees + minutes / 60.0
        if direction in ("S", "W"):
            dec = -dec
        return dec
    except ValueError:
        return math.nan


class GPSNode(Node):
    """
    node_read_gps:
    - Reads NMEA sentences from BerryGPS-IMU via serial.
    - Publishes sensor_msgs/NavSatFix on cfg.TOPIC_GPS.
    """

    def __init__(self):
        super().__init__("node_read_gps")
        cfg.check_domain_id(self.get_logger())

        self.publisher_ = self.create_publisher(NavSatFix, cfg.TOPIC_GPS, 10)

        try:
            self.ser = serial.Serial(
                cfg.GPS_SERIAL_PORT,
                cfg.GPS_BAUD,
                timeout=1.0
            )
            self.get_logger().info(
                f"Opened GPS serial {cfg.GPS_SERIAL_PORT} at {cfg.GPS_BAUD} baud"
            )
        except Exception as e:
            self.get_logger().error(f"Error opening GPS serial: {e}")
            self.ser = None

        self.timer = self.create_timer(0.2, self.read_gps)  # 5 Hz

    def read_gps(self):
        if self.ser is None:
            return
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line.startswith("$GP"):
                return

            parts = line.split(",")
            if line.startswith("$GPGGA"):
                # $GPGGA,utc,lat,NS,lon,EW,quality,numSV,HDOP,alt,unit,...
                if len(parts) < 10:
                    return
                lat = nmea_to_decimal(parts[2], parts[3])
                lon = nmea_to_decimal(parts[4], parts[5])
                alt = float(parts[9]) if parts[9] else 0.0
            elif line.startswith("$GPRMC"):
                # $GPRMC,utc,status,lat,NS,lon,EW,sog,cog,date,...
                if len(parts) < 7:
                    return
                lat = nmea_to_decimal(parts[3], parts[4])
                lon = nmea_to_decimal(parts[5], parts[6])
                alt = 0.0
            else:
                return

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = cfg.FRAME_GPS
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt

            # simple covariance placeholder
            msg.position_covariance = [1.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0,
                                       0.0, 0.0, 4.0]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error reading GPS: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
