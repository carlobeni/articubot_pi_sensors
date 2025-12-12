from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package="articubot_pi_sensors",
            executable="node_read_ir.py",
            name="node_read_ir",
            output="screen",
        ),

        Node(
            package="articubot_pi_sensors",
            executable="node_read_ultrasonic.py",
            name="node_read_ultrasonic",
            output="screen",
        ),

        Node(
            package="articubot_pi_sensors",
            executable="node_read_imu.py",
            name="node_read_imu",
            output="screen",
        ),

        Node(
            package="articubot_pi_sensors",
            executable="node_read_gps.py",
            name="node_read_gps",
            output="screen",
        ),

        Node(
            package="articubot_pi_sensors",
            executable="node_camera.py",
            name="node_camera",
            output="screen",
        ),

        Node(
            package="articubot_pi_sensors",
            executable="node_move.py",
            name="node_move",
            output="screen",
        ),

        Node(
            package="articubot_pi_sensors",
            executable="monitor_node.py",
            name="monitor_node",
            output="screen",
        ),
    ])
