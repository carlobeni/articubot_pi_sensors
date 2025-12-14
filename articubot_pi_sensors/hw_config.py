# articubot_pi_sensors/hw_config.py

"""
Hardware and ROS configuration for Raspberry Pi 4 + sensors.

All nodes import from here so changing pins/topics here affects everything.
"""

import os

# ---------------- ROS DOMAIN ID ----------------
# Set this in your shell BEFORE running ROS:
#   export ROS_DOMAIN_ID=42
ROS_DOMAIN_ID = 42

def check_domain_id(logger=None):
    env_val = os.getenv("ROS_DOMAIN_ID")
    if env_val is None:
        msg = (f"ROS_DOMAIN_ID env var is NOT set. "
               f"Recommended: export ROS_DOMAIN_ID={ROS_DOMAIN_ID}")
    else:
        msg = f"ROS_DOMAIN_ID env var = {env_val} (config expects {ROS_DOMAIN_ID})"
    if logger:
        logger.warn(msg)
    else:
        print("[hw_config]", msg)


# --------------- GPIO PIN ASSIGNMENTS (BCM) ---------------
# IMPORTANT:
# - Avoid GPIO2 (SDA) and GPIO3 (SCL) used by I2C (BerryGPS-IMU).
# - Avoid GPIO14/15 used by UART0 (GPS or other serial devices).

# IR SENSORS (digital type, e.g. IR obstacle sensors)
IR1_GPIO = 5     # BCM5
IR2_GPIO = 6     # BCM6

# HC-SR04 ULTRASONIC (5V sensor, level-shift recommended for ECHO to 3.3V)
ULTRASONIC_TRIG = 23   # BCM23
ULTRASONIC_ECHO = 24   # BCM24

# JS40F DIGITAL DISTANCE (3-wire, digital output)
# Brown = 5V, Blue = GND, Black = digital signal
JS40F_GPIO = 16        # BCM16 (only reserved here, node optional)

# BerryGPS-IMU v3
# - IMU via I2C on GPIO2 (SDA), GPIO3 (SCL), 3.3V, GND
# - GPS usually via serial (e.g. /dev/ttyAMA0 or /dev/ttyS0)
I2C_BUS_ID = 1
IMU_GYRO_ACC_ADDR = 0x6A   # gyroscope+accelerometer I2C address (default) :contentReference[oaicite:5]{index=5}
IMU_MAG_ADDR = 0x1C        # magnetometer I2C address (default) :contentReference[oaicite:6]{index=6}

GPS_SERIAL_PORT = "/dev/ttyAMA0"  # adjust if needed
GPS_BAUD = 9600                   # typical NMEA baud, adjust if configured differently

# Raspberry Pi Camera:
CAMERA_DEVICE_INDEX = 0  # for cv2.VideoCapture(0)

# Arduino Mega (serial link for hexapod commands)
MEGA_SERIAL_PORT = "/dev/ttyAMA3"   # adjust according to 'ls /dev/ttyACM*'
MEGA_BAUD = 9600


# --------------- ROS TOPIC NAMES ---------------
# READY TO USE
TOPIC_IMU_GIR_ACC = "pi/sensor/imu_data"
TOPIC_IMU_MAG = "pi/sensor/imu_mag"
TOPIC_GPS = "pi/sensor/gps_fix"
TOPIC_CAMERA = "pi/camera/image_raw"
# used only internally
TOPIC_MONITOR = "pi/system/monitor"  
TOPIC_CMD_SERIAL_MEGA = "/cmd_serial"

# IN DEVELOPMENT
TOPIC_IR = "pi/sensor/IR_measure"
TOPIC_ULTRASONIC = "pi/sensor/ultrasonic_read"

# Misc
FRAME_ULTRASONIC = "ultrasonic_link"
FRAME_IMU = "imu_link"
FRAME_GPS = "gps_link"
