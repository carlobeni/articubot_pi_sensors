from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():

    # ===== LIMPIEZA PREVIA (EVITA NODOS DUPLICADOS) =====
    cleanup = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "pkill -f node_camera.py || true; ",
            "pkill -f node_read_gps.py || true; ",
            "pkill -f node_read_imu.py || true; ",
            "pkill -f node_read_ir.py || true; ",
            "pkill -f node_read_ultrasonic.py || true; ",
            "pkill -f command_listener_node.py || true; "
            "pkill -f monitor_node.py || true"
        ],
        output="screen"
    )

    return LaunchDescription([

        # ---------- CLEANUP ----------
        cleanup,

        # Espera corta para asegurar que los procesos mueran
        TimerAction(
            period=1.0,
            actions=[

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

                # ===== COMMAND LISTENER → ARDUINO MEGA =====
                Node(
                    package="articubot_pi_sensors",
                    executable="command_listener_node.py",
                    name="command_listener",
                    output="screen",
                    parameters=[{
                        # Comandos = críticos → confiable
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
            ]
        ),
    ])
