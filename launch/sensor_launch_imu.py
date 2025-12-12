from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ===== IMU (accel + mag + compass) =====
        Node(
            package="articubot_pi_sensors",
            executable="node_read_imu.py",
            name="node_read_imu",
            output="screen",
            parameters=[{
                "qos_reliability": "best_effort",
                "qos_history": "keep_last",
                "qos_depth": 10,
            }],
        ),

        # ===== SERIAL → MEGA (SUBSCRIBER) =====
        Node(
            package="articubot_pi_sensors",
            executable="node_move.py",
            name="node_move",
            output="screen",
            parameters=[{
                # Control: confiabilidad alta
                "qos_reliability": "reliable",
                "qos_history": "keep_last",
                "qos_depth": 10,
            }],
        ),

        # ===== MONITOR (IMU + SERIAL) =====
        Node(
            package="articubot_pi_sensors",
            executable="monitor_node.py",
            name="monitor_node",
            output="screen",
            prefix=["xterm -hold -e"],
            parameters=[{
                # QoS del monitor (debe empatar con IMU)
                "qos_reliability": "best_effort",
                "qos_history": "keep_last",
                "qos_depth": 10,

                # Sensores
                "monitor_ir": False,
                "monitor_ultrasonic": False,
                "monitor_gps": False,
                "monitor_camera": False,
                "monitor_imu_accel": True,
                "monitor_imu_mag": True,
                "monitor_imu_compass": True,

                # Serial subscriber
                "monitor_cmd_serial": True,
            }],
        ),
    ])
