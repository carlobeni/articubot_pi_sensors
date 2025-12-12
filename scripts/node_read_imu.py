#!/usr/bin/env python3
#
# ROS2 Node: node_read_imu
#
# This node reads accelerometer, gyroscope and magnetometer data
# from the BerryIMU and publishes:
#
#   /sensor/imu/accel   → geometry_msgs/Vector3Stamped
#   /sensor/imu/mag     → geometry_msgs/Vector3Stamped
#   /sensor/imu/compass → std_msgs/Float64  (tilt compensated heading)
#
# Fully based on the official OzzMaker IMU example with:
#  - Low pass filters
#  - Median filters
#  - Complementary filter
#  - Kalman filter
#
# Compatible with BerryIMUv1/2/3/320.
#

import sys
import math
import time
import datetime
import rclpy
from rclpy.node import Node

from std_msgs.msg import z
from geometry_msgs.msg import Vector3Stamped

from articubot_pi_sensors.utils.BerryIMU import IMU
import hw_config as cfg


#############################
# CONSTANTES ORIGINALES
#############################
RAD_TO_DEG = 57.29578
M_PI = math.pi
G_GAIN = 0.070
AA = 0.40    

MAG_LPF_FACTOR = 0.4
ACC_LPF_FACTOR = 0.4

ACC_MEDIANTABLESIZE = 9
MAG_MEDIANTABLESIZE = 9


############# CALIBRACIÓN DEL COMPÁS (puedes modificar aquí) ###############
magXmin = 0
magYmin = 0
magZmin = 0
magXmax = 0
magYmax = 0
magZmax = 0
###########################################################################


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
#  FUNCIONES DEL KALMAN ORIGINAL
#########################################
def kalmanFilterY(accAngle, gyroRate, DT):
    global KFangleY, Q_angle, Q_gyro, y_bias
    global YP_00, YP_01, YP_10, YP_11, R_angle

    KFangleY = KFangleY + DT * (gyroRate - y_bias)

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
    global KFangleX, Q_angle, Q_gyro, x_bias
    global XP_00, XP_01, XP_10, XP_11, R_angle

    KFangleX = KFangleX + DT * (gyroRate - x_bias)

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
#   NODO ROS2
#########################################
class BerryIMUNode(Node):
    def __init__(self):
        super().__init__("node_read_imu")

        cfg.check_domain_id(self.get_logger())

        # Publicadores
        self.pub_accel = self.create_publisher(
            Vector3Stamped, cfg.TOPIC_IMU_ACCEL, 10)
        self.pub_mag = self.create_publisher(
            Vector3Stamped, cfg.TOPIC_IMU_MAG, 10)
        self.pub_heading = self.create_publisher(
            Float64, cfg.TOPIC_IMU_COMPASS, 10)

        self.get_logger().info("Inicializando IMU BerryIMU…")
        IMU.detectIMU()
        if IMU.BerryIMUversion == 99:
            self.get_logger().error("No se detectó BerryIMU")
            sys.exit(1)
        IMU.initIMU()
        self.get_logger().info("BerryIMU detectada y configurada")

        # Variables del filtro
        self.oldAccX = self.oldAccY = self.oldAccZ = 0
        self.oldMagX = self.oldMagY = self.oldMagZ = 0

        # tablas de mediana
        self.acc1X = [1] * ACC_MEDIANTABLESIZE
        self.acc1Y = [1] * ACC_MEDIANTABLESIZE
        self.acc1Z = [1] * ACC_MEDIANTABLESIZE

        self.mag1X = [1] * MAG_MEDIANTABLESIZE
        self.mag1Y = [1] * MAG_MEDIANTABLESIZE
        self.mag1Z = [1] * MAG_MEDIANTABLESIZE

        self.last_time = datetime.datetime.now()

        # Timer 33 ms
        self.timer = self.create_timer(0.033, self.loop)

    #########################################
    # LOOP PRINCIPAL
    #########################################
    def loop(self):

        global KFangleX, KFangleY

        # Leer sensores
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        GYRx = IMU.readGYRx()
        GYRy = IMU.readGYRy()
        GYRz = IMU.readGYRz()
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()

        # Aplicar calibración del compás
        MAGx -= (magXmin + magXmax)/2
        MAGy -= (magYmin + magYmax)/2
        MAGz -= (magZmin + magZmax)/2

        # Tiempo delta
        now = datetime.datetime.now()
        LP = (now - self.last_time).microseconds / 1_000_000.0
        self.last_time = now

        #################################
        #  LOW PASS FILTERS
        #################################
        ACCx = ACCx * ACC_LPF_FACTOR + self.oldAccX * (1 - ACC_LPF_FACTOR)
        ACCy = ACCy * ACC_LPF_FACTOR + self.oldAccY * (1 - ACC_LPF_FACTOR)
        ACCz = ACCz * ACC_LPF_FACTOR + self.oldAccZ * (1 - ACC_LPF_FACTOR)

        MAGx = MAGx * MAG_LPF_FACTOR + self.oldMagX * (1 - MAG_LPF_FACTOR)
        MAGy = MAGy * MAG_LPF_FACTOR + self.oldMagY * (1 - MAG_LPF_FACTOR)
        MAGz = MAGz * MAG_LPF_FACTOR + self.oldMagZ * (1 - MAG_LPF_FACTOR)

        self.oldAccX, self.oldAccY, self.oldAccZ = ACCx, ACCy, ACCz
        self.oldMagX, self.oldMagY, self.oldMagZ = MAGx, MAGy, MAGz

        #################################
        #  MEDIAN FILTERS
        #################################
        def median_update(table, value):
            table.pop()
            table.insert(0, value)
            sorted_table = sorted(table)
            return sorted_table[len(table)//2]

        ACCx = median_update(self.acc1X, ACCx)
        ACCy = median_update(self.acc1Y, ACCy)
        ACCz = median_update(self.acc1Z, ACCz)

        MAGx = median_update(self.mag1X, MAGx)
        MAGy = median_update(self.mag1Y, MAGy)
        MAGz = median_update(self.mag1Z, MAGz)

        #################################
        #   ÁNGULOS A PARTIR DE ACCEL
        #################################
        AccXangle = math.atan2(ACCy, ACCz) * RAD_TO_DEG
        AccYangle = (math.atan2(ACCz, ACCx) + M_PI) * RAD_TO_DEG
        if AccYangle > 90:
            AccYangle -= 270
        else:
            AccYangle += 90

        #################################
        #   GYRO → deg/s
        #################################
        rateX = GYRx * G_GAIN
        rateY = GYRy * G_GAIN

        #################################
        #  FILTRO COMPLEMENTARIO
        #################################
        CFangleX = AA * (KFangleX + rateX * LP) + (1 - AA) * AccXangle
        CFangleY = AA * (KFangleY + rateY * LP) + (1 - AA) * AccYangle

        #################################
        #  KALMAN FILTER
        #################################
        KFangleX = kalmanFilterX(AccXangle, rateX, LP)
        KFangleY = kalmanFilterY(AccYangle, rateY, LP)

        #################################
        #  HEADING PURO
        #################################
        heading = math.atan2(MAGy, MAGx) * RAD_TO_DEG
        if heading < 0:
            heading += 360

        #################################
        # TILT COMPENSATION
        #################################
        normAcc = math.sqrt(ACCx*ACCx + ACCy*ACCy + ACCz*ACCz)
        accXnorm = ACCx / normAcc
        accYnorm = ACCy / normAcc

        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm / math.cos(pitch))

        if IMU.BerryIMUversion in (1,3,320):
            magXcomp = MAGx*math.cos(pitch) + MAGz*math.sin(pitch)
        else:
            magXcomp = MAGx*math.cos(pitch) - MAGz*math.sin(pitch)

        if IMU.BerryIMUversion in (1,3,320):
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch) + MAGy*math.cos(roll) - MAGz*math.sin(roll)*math.cos(pitch)
        else:
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch) + MAGy*math.cos(roll) + MAGz*math.sin(roll)*math.cos(pitch)

        tiltHeading = math.atan2(magYcomp, magXcomp) * RAD_TO_DEG
        if tiltHeading < 0:
            tiltHeading += 360

        #################################
        # PUBLICAR EN ROS2
        #################################

        # 1. Acelerómetro
        acc_msg = Vector3Stamped()
        acc_msg.header.stamp = self.get_clock().now().to_msg()
        acc_msg.vector.x = ACCx
        acc_msg.vector.y = ACCy
        acc_msg.vector.z = ACCz
        self.pub_accel.publish(acc_msg)

        # 2. Magnetómetro
        mag_msg = Vector3Stamped()
        mag_msg.header.stamp = acc_msg.header.stamp
        mag_msg.vector.x = MAGx
        mag_msg.vector.y = MAGy
        mag_msg.vector.z = MAGz
        self.pub_mag.publish(mag_msg)

        # 3. Brújula compensada
        self.pub_heading.publish(Float64(data=float(tiltHeading)))



def main(args=None):
    rclpy.init(args=args)
    node = BerryIMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
