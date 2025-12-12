cmake_minimum_required(VERSION 3.5)
project(articubot_pi_sensors)

# Build system
find_package(ament_cmake REQUIRED)

# ------------------------------------------------------------------------------
# Instalar scripts Python como ejecutables ROS2
# (todos están directamente en scripts/)
# ------------------------------------------------------------------------------
install(
  PROGRAMS
    scripts/node_read_ir.py
    scripts/node_read_ultrasonic.py
    scripts/node_read_gps.py
    scripts/node_read_imu.py
    scripts/node_camera.py
    scripts/node_move.py
    scripts/monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Instalar utilidades (BerryIMU, helpers, etc.)
# ------------------------------------------------------------------------------
install(
  DIRECTORY scripts/utils
  DESTINATION lib/${PROJECT_NAME}/utils
)

# ------------------------------------------------------------------------------
# Instalar archivo de configuración global
# ------------------------------------------------------------------------------
install(
  FILES scripts/hw_config.py
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
