#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from utils.BerryIMU import IMU
import hw_config as cfg
from transforms3d.euler import euler2quat

# =============================
# CONSTANTES
# =============================
DEG_TO_RAD = math.pi / 180.0
G_GAIN = 0.070

ACC_LPF_FACTOR = 0.4
MAG_LPF_FACTOR = 0.4
ACC_MEDIANTABLESIZE = 9
MAG_MEDIANTABLESIZE = 9

factor_to_ms2 = (0.224/1000)*9.81  # 1G = 9.81m/s², 1LSB = 0.224mg

# =============================
# DECLINACIÓN MAGNÉTICA
# =============================
MAG_DECLINATION_DEG = -15.24
MAG_DECLINATION_RAD = float(MAG_DECLINATION_DEG * DEG_TO_RAD)

# =============================
# CALIBRACIÓN MAGNETÓMETRO
# =============================
magXmin = -510
magYmin = -245
magZmin = -746
magXmax = 1922
magYmax = 1614
magZmax = 650


class BerryIMUNode(Node):

    def __init__(self):
        super().__init__("node_read_imu")

        cfg.check_domain_id(self.get_logger())

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_imu = self.create_publisher(Imu, cfg.TOPIC_IMU_GIR_ACC, qos)
        self.pub_mag = self.create_publisher(MagneticField, cfg.TOPIC_IMU_MAG, qos)

        IMU.detectIMU()
        if IMU.BerryIMUversion == 99:
            self.get_logger().fatal("No se detectó BerryIMU")
            sys.exit(1)

        IMU.initIMU()
        self.get_logger().info("BerryIMU inicializada correctamente")

        self.oldAcc = [0.0, 0.0, 0.0]
        self.oldMag = [0.0, 0.0, 0.0]

        self.acc_table = [[1.0]*ACC_MEDIANTABLESIZE for _ in range(3)]
        self.mag_table = [[1.0]*MAG_MEDIANTABLESIZE for _ in range(3)]

        self.last_time = self.get_clock().now()

        self.create_timer(0.03, self.loop)

    def median(self, table, val):
        table.pop()
        table.insert(0, val)
        return sorted(table)[len(table)//2]

    def loop(self):
        now = self.get_clock().now()

        # =============================
        # LECTURAS CRUDAS (forzar float)
        # =============================
        ACC_raw = [
            float(IMU.readACCx()),
            float(IMU.readACCy()),
            float(IMU.readACCz())
        ]

        ACC = [a * factor_to_ms2 for a in ACC_raw]

        GYR = [float(IMU.readGYRx()), float(IMU.readGYRy()), float(IMU.readGYRz())]
        MAG = [float(IMU.readMAGx()), float(IMU.readMAGy()), float(IMU.readMAGz())]

        # =============================
        # CALIBRACIÓN MAG
        # =============================
        MAG[0] -= float((magXmin + magXmax) / 2.0)
        MAG[1] -= float((magYmin + magYmax) / 2.0)
        MAG[2] -= float((magZmin + magZmax) / 2.0)

        # =============================
        # LOW PASS FILTER
        # =============================
        for i in range(3):
            ACC[i] = float(ACC[i] * ACC_LPF_FACTOR + self.oldAcc[i] * (1.0 - ACC_LPF_FACTOR))
            MAG[i] = float(MAG[i] * MAG_LPF_FACTOR + self.oldMag[i] * (1.0 - MAG_LPF_FACTOR))

        self.oldAcc = ACC[:]
        self.oldMag = MAG[:]

        # =============================
        # MEDIAN FILTER
        # =============================
        for i in range(3):
            ACC[i] = float(self.median(self.acc_table[i], ACC[i]))
            MAG[i] = float(self.median(self.mag_table[i], MAG[i]))

        # =============================
        # GIROSCOPIO (rad/s) -> float puro
        # =============================
        gyro_rate = [float(g * G_GAIN * DEG_TO_RAD) for g in GYR]

        # =============================
        # ROLL / PITCH (ACC)
        # =============================
        acc_norm = float(math.sqrt(ACC[0]**2 + ACC[1]**2 + ACC[2]**2))
        if acc_norm < 1e-6:
            return

        accXn = float(ACC[0] / acc_norm)
        accYn = float(ACC[1] / acc_norm)
        accZn = float(ACC[2] / acc_norm)

        roll = float(math.atan2(accYn, accZn))
        pitch = float(math.atan2(-accXn, math.sqrt(accYn**2 + accZn**2)))

        if abs(math.cos(pitch)) < 1e-3:
            return

        # =============================
        # YAW (MAG + TILT + DECLINACIÓN)
        # =============================
        magXc = float(MAG[0] * math.cos(pitch) + MAG[2] * math.sin(pitch))
        magYc = float(
            MAG[0] * math.sin(roll) * math.sin(pitch)
            + MAG[1] * math.cos(roll)
            - MAG[2] * math.sin(roll) * math.cos(pitch)
        )

        yaw = float(math.atan2(magYc, magXc) + MAG_DECLINATION_RAD)

        if yaw > math.pi:
            yaw -= float(2.0 * math.pi)
        elif yaw < -math.pi:
            yaw += float(2.0 * math.pi)

        # =============================
        # CUATERNION (forzar float)
        # transforms3d devuelve (w,x,y,z)
        # =============================
        qw, qx, qy, qz = euler2quat(roll, pitch, yaw)
        qw, qx, qy, qz = float(qw), float(qx), float(qy), float(qz)

        # =============================
        # MENSAJE IMU
        # =============================
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = "imu_link"

        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        imu.angular_velocity.x = float(gyro_rate[0])
        imu.angular_velocity.y = float(gyro_rate[1])
        imu.angular_velocity.z = float(gyro_rate[2])

        imu.linear_acceleration.x = float(ACC[0])
        imu.linear_acceleration.y = float(ACC[1])
        imu.linear_acceleration.z = float(ACC[2])

        imu.orientation_covariance = [float(x) for x in [
            0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,
            0.0,  0.0,  0.0
        ]]
        imu.angular_velocity_covariance = [float(x) for x in [
            0.0, 0.0,  0.0,
            0.0,  0.0, 0.0,
            0.0,  0.0,  0.0
        ]]
        imu.linear_acceleration_covariance = [float(x) for x in [
            0.0, 0.0,  0.0,
            0.0, 0.0,  0.0,
            0.0, 0.0,  0.0
        ]]

        self.pub_imu.publish(imu)

        # =============================
        # MAGNETIC FIELD
        # =============================
        mag = MagneticField()
        mag.header = imu.header
        mag.magnetic_field.x = float(MAG[0])
        mag.magnetic_field.y = float(MAG[1])
        mag.magnetic_field.z = float(MAG[2])

        self.pub_mag.publish(mag)


def main(args=None):
    rclpy.init(args=args)
    node = BerryIMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
