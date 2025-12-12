# articubot_pi_client
`articubot_pi_sensors` es un paquete ROS2 diseñado para Raspberry Pi 4 que provee nodos para leer múltiples sensores conectados por GPIO, I2C, UART y cámara.  
Las mediciones se publican en topics ROS2, permitiendo integrar los datos en sistemas de navegación, control y monitoreo del robot.

El paquete incluye nodos para:
- IMU BerryIMU (acelerómetro, magnetómetro, heading)
- Sensores IR digitales
- Sensor ultrasónico HC-SR04
- GPS (BerryGPS) por UART
- JS40F digital
- Cámara (imagen comprimida)
- Control serial del robot vía Arduino Mega
- Un monitor en terminal con tabla dinámica

La configuración de pines, topics y ROS_DOMAIN_ID se realiza desde `hw_config.py`.


##  Descripción de nodos

### **node_read_imu**
Lee IMU BerryIMU aplicando:
- Low Pass Filter  
- Median Filter  
- Complementary Filter  
- Kalman Filter  

Publica:
- `/sensor/imu/accel`
- `/sensor/imu/mag`
- `/sensor/imu/compass`


### **node_read_ir**
Lee dos sensores IR digitales por GPIO.  
Publica: `/sensor/IR_measure`


### **node_read_ultrasonic**
Mide distancia con HC-SR04.  
Publica: `/sensor/ultrasonic_read`


### **node_read_gps**
Lectura del GPS desde BerryGPS por UART.  
Publica latitud, longitud y tiempo.


### **node_camera**
Captura y publica imagen comprimida.  
Topic: `/camera/image_raw/compressed`


### **node_move**
Convierte `/cmd_vel_master` a comandos seriales para Arduino Mega.


### **monitor_node**
Muestra una tabla dinámica en terminal con todos los sensores.


## Instalacion

### 1. Clonar repositorio
```bash
git clone https://github.com/carlobeni/articubot_pi_sensors.git
```

### 2. Instalar dependencias

```bash
sudo apt update
sudo apt install python3-smbus
sudo apt install python3-smbus2
sudo apt install python3-rpi.gpio
sudo apt install python3-serial
sudo apt install python3-opencv
```

### 3. Compilar
```bash
cd articubot_pi_sensors
colcon build --symlink-install
```

### 4. Correr launch
```bash
ros2 launch articubot_pi_sensors sensor_launch.py
```