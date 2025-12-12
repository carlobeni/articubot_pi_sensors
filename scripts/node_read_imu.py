#!/usr/bin/env python3
# ROS2 Node: node_read_imu


import sys
import math
import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped

# Imports locales (estructura scripts/)
from utils.BerryIMU import IMU
import hw_config as cfg


#############################
# CONSTANTES
#############################
RAD_TO_DEG = 57.29578
M_PI = math.pi
G_GAIN = 0.070
AA = 0.40

MAG_LPF_FACTOR = 0.4
ACC_LPF_FACTOR = 0.4

ACC_MEDIANTABLESIZE = 9
MAG_MEDIANTABLESIZE = 9


#############################
# CALIBRACIÓN DEL COMPÁS
#############################
magXmin = 0
magYmin = 0
magZmin = 0
magXmax = 0
magYmax = 0
magZmax = 0


#############################
# VARIABLES DEL KALMAN
#############################
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005

y_bias = 0.0
x_bias = 0.0

XP_00 = XP_01 = XP_10 = XP_11 = 0.0
YP_00 = YP_01 = YP_10 = YP_11 = 0.0

KFangleX = 0.0
KFangleY = 0.0


#########################################
# KALMAN FILTERS
#########################################
def kalmanFilterY(accAngle, gyroRate, DT):
    global KFangleY, y_bias, YP_00, YP_01, YP_10, YP_11

    KFangleY += DT * (gyroRate - y_bias)

    YP_00 += (-DT * (YP_10 + YP_01) + Q_angle * DT)
    YP_01 += (-DT * YP_11)
    YP_10 += (-DT * YP_11)
    YP_11 += (Q_gyro * DT)

    y = accAngle - KFangleY
    S = YP_00 + R_angle
    K_0 = YP_00 / S
    K_1 = YP_10 / S

    KFangleY += K_0 * y
    y_bias += K_1 * y

    YP_00 -= K_0 * YP_00
    YP_01 -= K_0 * YP_01
    YP_10 -= K_1 * YP_00
    YP_11 -= K_1 * YP_01

    return KFangleY


def kalmanFilterX(accAngle, gyroRate, DT):
    global KFangleX, x_bias, XP_00, XP_01, XP_10, XP_11

    KFangleX += DT * (gyroRate - x_bias)

    XP_00 += (-DT * (XP_10 + XP_01) + Q_angle * DT)
    XP_01 += (-DT * XP_11)
    XP_10 += (-DT * XP_11)
    XP_11 += (Q_gyro * DT)

    x = accAngle - KFangleX
    S = XP_00 + R_angle
    K_0 = XP_00 / S
    K_1 = XP_10 / S

    KFangleX += K_0 * x
    x_bias += K_1 * x

    XP_00 -= K_0 * XP_00
    XP_01 -= K_0 * XP_01
    XP_10 -= K_1 * XP_00
    XP_11 -= K_1 * XP_01

    return KFangleX


#########################################
# NODO ROS2
#########################################
class BerryIMUNode(Node):

    def __init__(self):
        super().__init__("node_read_imu")

        cfg.check_domain_id(self.get_logger())

        self.pub_accel = self.create_publisher(
            Vector3Stamped, cfg.TOPIC_IMU_ACCEL, 10)

        self.pub_mag = self.create_publisher(
            Vector3Stamped, cfg.TOPIC_IMU_MAG, 10)

        self.pub_heading = self.create_publisher(
            Float64, cfg.TOPIC_IMU_COMPASS, 10)

        self.get_logger().info("Inicializando BerryIMU...")
        IMU.detectIMU()
        if IMU.BerryIMUversion == 99:
            self.get_logger().error("No se detectó BerryIMU")
            sys.exit(1)

        IMU.initIMU()
        self.get_logger().info("BerryIMU inicializada correctamente")

        self.oldAccX = self.oldAccY = self.oldAccZ = 0.0
        self.oldMagX = self.oldMagY = self.oldMagZ = 0.0

        self.accX = [1] * ACC_MEDIANTABLESIZE
        self.accY = [1] * ACC_MEDIANTABLESIZE
        self.accZ = [1] * ACC_MEDIANTABLESIZE

        self.magX = [1] * MAG_MEDIANTABLESIZE
        self.magY = [1] * MAG_MEDIANTABLESIZE
        self.magZ = [1] * MAG_MEDIANTABLESIZE

        self.last_time = datetime.datetime.now()
        self.timer = self.create_timer(0.033, self.loop)

    def _median(self, table, value):
        table.pop()
        table.insert(0, value)
        return sorted(table)[len(table)//2]

    def loop(self):
        global KFangleX, KFangleY

        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        GYRx = IMU.readGYRx()
        GYRy = IMU.readGYRy()
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()

        MAGx -= (magXmin + magXmax) / 2
        MAGy -= (magYmin + magYmax) / 2
        MAGz -= (magZmin + magZmax) / 2

        now = datetime.datetime.now()
        LP = (now - self.last_time).microseconds / 1_000_000.0
        self.last_time = now

        ACCx = ACCx * ACC_LPF_FACTOR + self.oldAccX * (1 - ACC_LPF_FACTOR)
        ACCy = ACCy * ACC_LPF_FACTOR + self.oldAccY * (1 - ACC_LPF_FACTOR)
        ACCz = ACCz * ACC_LPF_FACTOR + self.oldAccZ * (1 - ACC_LPF_FACTOR)

        MAGx = MAGx * MAG_LPF_FACTOR + self.oldMagX * (1 - MAG_LPF_FACTOR)
        MAGy = MAGy * MAG_LPF_FACTOR + self.oldMagY * (1 - MAG_LPF_FACTOR)
        MAGz = MAGz * MAG_LPF_FACTOR + self.oldMagZ * (1 - MAG_LPF_FACTOR)

        self.oldAccX, self.oldAccY, self.oldAccZ = ACCx, ACCy, ACCz
        self.oldMagX, self.oldMagY, self.oldMagZ = MAGx, MAGy, MAGz

        ACCx = self._median(self.accX, ACCx)
        ACCy = self._median(self.accY, ACCy)
        ACCz = self._median(self.accZ, ACCz)

        MAGx = self._median(self.magX, MAGx)
        MAGy = self._median(self.magY, MAGy)
        MAGz = self._median(self.magZ, MAGz)

        AccXangle = math.atan2(ACCy, ACCz) * RAD_TO_DEG
        AccYangle = (math.atan2(ACCz, ACCx) + M_PI) * RAD_TO_DEG
        AccYangle = AccYangle - 270 if AccYangle > 90 else AccYangle + 90

        rateX = GYRx * G_GAIN
        rateY = GYRy * G_GAIN

        KFangleX = kalmanFilterX(AccXangle, rateX, LP)
        KFangleY = kalmanFilterY(AccYangle, rateY, LP)

        heading = math.atan2(MAGy, MAGx) * RAD_TO_DEG
        if heading < 0:
            heading += 360

        acc_norm = math.sqrt(ACCx**2 + ACCy**2 + ACCz**2)
        pitch = math.asin(ACCx / acc_norm)
        roll = -math.asin(ACCy / (acc_norm * math.cos(pitch)))

        magXcomp = MAGx * math.cos(pitch) + MAGz * math.sin(pitch)
        magYcomp = MAGx * math.sin(roll)*math.sin(pitch) + MAGy * math.cos(roll) - MAGz * math.sin(roll)*math.cos(pitch)

        tilt_heading = math.atan2(magYcomp, magXcomp) * RAD_TO_DEG
        if tilt_heading < 0:
            tilt_heading += 360

        stamp = self.get_clock().now().to_msg()

        acc_msg = Vector3Stamped()
        acc_msg.header.stamp = stamp
        acc_msg.vector.x = ACCx
        acc_msg.vector.y = ACCy
        acc_msg.vector.z = ACCz
        self.pub_accel.publish(acc_msg)

        mag_msg = Vector3Stamped()
        mag_msg.header.stamp = stamp
        mag_msg.vector.x = MAGx
        mag_msg.vector.y = MAGy
        mag_msg.vector.z = MAGz
        self.pub_mag.publish(mag_msg)

        self.pub_heading.publish(Float64(data=float(tilt_heading)))


def main(args=None):
    rclpy.init(args=args)
    node = BerryIMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
