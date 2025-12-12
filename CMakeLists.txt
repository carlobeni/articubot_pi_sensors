cmake_minimum_required(VERSION 3.5)
project(articubot_pi_sensors)

# Build system
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# ------------------------------------------------------------------------------
# Instalar articubot_pi_sensors Python como ejecutables ROS2
# (todos están directamente en articubot_pi_sensors/)
# ------------------------------------------------------------------------------
install(
  PROGRAMS
    articubot_pi_sensors/node_read_ir.py
    articubot_pi_sensors/node_read_ultrasonic.py
    articubot_pi_sensors/node_read_gps.py
    articubot_pi_sensors/node_read_imu.py
    articubot_pi_sensors/node_camera.py
    articubot_pi_sensors/node_move.py
    articubot_pi_sensors/monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Instalar utilidades (BerryIMU, helpers, etc.)
# ------------------------------------------------------------------------------
install(
  DIRECTORY articubot_pi_sensors/utils
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Instalar archivo de configuración global
# ------------------------------------------------------------------------------
install(
  FILES articubot_pi_sensors/hw_config.py
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Instalar launch files
# ------------------------------------------------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
ament_package()
