from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ===== IR =====
        Node(
            package="articubot_pi_sensors",
            executable="node_read_ir.py",
            name="node_read_ir",
            output="screen",
            parameters=[{
                "qos_reliability": "best_effort",
                "qos_history": "keep_last",
                "qos_depth": 5,
            }],
        ),

        # ===== ULTRASONIC =====
        Node(
            package="articubot_pi_sensors",
            executable="node_read_ultrasonic.py",
            name="node_read_ultrasonic",
            output="screen",
            parameters=[{
                "qos_reliability": "best_effort",
                "qos_history": "keep_last",
                "qos_depth": 5,
            }],
        ),

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

        # ===== GPS =====
        Node(
            package="articubot_pi_sensors",
            executable="node_read_gps.py",
            name="node_read_gps",
            output="screen",
            parameters=[{
                "qos_reliability": "reliable",
                "qos_history": "keep_last",
                "qos_depth": 5,
            }],
        ),

        # ===== CAMERA =====
        Node(
            package="articubot_pi_sensors",
            executable="node_camera.py",
            name="node_camera",
            output="screen",
            parameters=[{
                "qos_reliability": "best_effort",
                "qos_history": "keep_last",
                "qos_depth": 2,
            }],
        ),

        # ===== MOVE / SERIAL BRIDGE =====
        # Suscriptor a /cmd_serial → reenvía a Arduino Mega
        Node(
            package="articubot_pi_sensors",
            executable="node_move.py",
            name="node_move",
            output="screen",
            parameters=[{
                "qos_reliability": "reliable",
                "qos_history": "keep_last",
                "qos_depth": 10,
            }],
        ),

        # ===== MONITOR (PUBLISHERS + SUBSCRIBERS, CON ID) =====
        Node(
            package="articubot_pi_sensors",
            executable="monitor_node.py",
            name="monitor_node",
            output="screen",
            prefix=["xterm -hold -e"],
            parameters=[{
                # QoS del monitor (sensores = best_effort)
                "qos_reliability": "best_effort",
                "qos_history": "keep_last",
                "qos_depth": 10,

                # Sensores
                "monitor_ir": True,
                "monitor_ultrasonic": True,
                "monitor_gps": True,
                "monitor_camera": True,
                "monitor_imu_accel": True,
                "monitor_imu_mag": True,
                "monitor_imu_compass": True,

                # Subscriber serial
                "monitor_cmd_serial": True,
            }],
        ),
    ])
