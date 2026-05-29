Para lograr que el Pickasso esquive obstáculos de forma fluida y mantenga la estabilidad del manipulador xArm5, debemos dar el salto definitivo del control por segmentos (que bloquea el código con `time.sleep`) a un control cinemático continuo para tus llantas mecanum.

Aquí tienes la guía completa y definitiva con los tres pasos: la actualización de la Raspberry, los ajustes exactos de ROS 2 y los comandos de ejecución.

### 1. El Nuevo Cerebro Físico (Script de Raspberry Pi)

Tu código original leía distancias y se quedaba atrapado en bucles `while` esperando a que los encoders llegaran a la meta. Para escuchar a `cmd_vel_udp_bridge_node`, el script debe calcular las ecuaciones cinemáticas de tus 4 ruedas holonómicas en tiempo real y tener un "freno de emergencia" (watchdog) por si se pierde la conexión WiFi.

Reemplaza todo tu script de la Raspberry con esta versión. He respetado la inicialización de tus ESP32, tu IMU y la lógica de direcciones (`1` y `2`) que ya tenías mapeada para tus motores:

```python
import serial
import time
import math
import socket
import board
import busio
import adafruit_bno055

# =========================
# CONFIGURACIÓN
# =========================

UDP_IP = "0.0.0.0"
UDP_PORT = 5005  # AHORA ESCUCHA EN EL PUERTO DEL CMD_VEL

# Escalamiento: Velocidad máxima de ROS (0.25 m/s) a RPM de tus motores
VEL_MAX_ROS = 0.25
RPM_MAX_MOTORES = 25.0
FACTOR_CONVERSION = RPM_MAX_MOTORES / VEL_MAX_ROS

# =========================
# IMU Y SERIAL ESP32
# =========================

i2c = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)

def obtener_yaw():
    try:
        yaw = sensor_imu.euler[0]
        if yaw is not None:
            return yaw
    except Exception:
        pass
    return 0.0

try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
    time.sleep(2)
    print("ESP32 conectadas.")
except Exception as e:
    print(f"Error serial: {e}")
    exit()

def mandar_orden(placa, motor, direccion, rpm):
    comando = f"{motor},{direccion},{round(abs(rpm), 1)}\n"
    placa.write(comando.encode("utf-8"))

def frenar_y_limpiar():
    mandar_orden(esp_1, "A", 0, 0)
    mandar_orden(esp_1, "B", 0, 0)
    mandar_orden(esp_2, "A", 0, 0)
    mandar_orden(esp_2, "B", 0, 0)
    esp_1.reset_input_buffer()
    esp_2.reset_input_buffer()

# =========================
# UDP Y CONTROL CONTINUO MECANUM
# =========================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
# Watchdog: Frena si no recibe comandos de ROS en 0.5 segundos
sock.settimeout(0.5) 

print(f"Escuchando velocidades continuas en puerto {UDP_PORT}")
print("Esperando conexión de ROS 2...")

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode("utf-8").strip()
            
            partes = msg.split(",")
            if len(partes) != 3:
                continue

            # 1. Extraer velocidades (vx, vy, omega)
            vx_ros = float(partes[0])
            vy_ros = float(partes[1])
            omega_ros = float(partes[2])

            # 2. Cinemática Inversa Mecanum
            # FL: Front Left, FR: Front Right, BL: Back Left, BR: Back Right
            v_fl = vy_ros + vx_ros - omega_ros
            v_fr = vy_ros - vx_ros + omega_ros
            v_bl = vy_ros - vx_ros - omega_ros
            v_br = vy_ros + vx_ros + omega_ros

            # 3. Escalar a RPM
            rpm_fl = v_fl * FACTOR_CONVERSION
            rpm_fr = v_fr * FACTOR_CONVERSION
            rpm_bl = v_bl * FACTOR_CONVERSION
            rpm_br = v_br * FACTOR_CONVERSION

            # 4. Asignar direcciones respetando tu electrónica
            # ESP1 (Frontales): Adelante = 2
            dir_fl = 2 if rpm_fl >= 0 else 1
            dir_fr = 2 if rpm_fr >= 0 else 1
            
            # ESP2 (Traseras): Adelante = 1
            dir_bl = 1 if rpm_bl >= 0 else 2
            dir_br = 1 if rpm_br >= 0 else 2

            # 5. Enviar instrucciones a las ESP32
            mandar_orden(esp_1, "A", dir_fl, abs(rpm_fl))
            mandar_orden(esp_1, "B", dir_fr, abs(rpm_fr))
            mandar_orden(esp_2, "A", dir_bl, abs(rpm_bl))
            mandar_orden(esp_2, "B", dir_br, abs(rpm_br))

        except socket.timeout:
            # ROS dejó de transmitir (ej. evasión fallida o meta alcanzada)
            frenar_y_limpiar()

except KeyboardInterrupt:
    print("Apagando...")
    frenar_y_limpiar()
finally:
    frenar_y_limpiar()
    esp_1.close()
    esp_2.close()
    sock.close()

```

---

### 2. Los Ajustes Clave en ROS 2 (Computadora)

Asegúrate de que en tu computadora tienes guardados estos tres detalles vitales para que el mapa y el planificador no se atasquen:

**En `config.py`:**
Verifica que las medidas y márgenes estén configurados así para darle espacio de maniobra al chasis holonómico sin hacer los obstáculos gigantes:

```python
ROBOT_LENGTH = 0.70
ROBOT_WIDTH = 0.68
SAFETY_MARGIN = 0.01
TABLE_SAFETY_MARGIN = 0.15
TABLE_SIZE = (0.60, 1.30)

```

**En `planner_node.py`:**
Asegúrate de que los bordes del mapa sean solo de 1 celda (hacia afuera) y que el sistema envíe una ruta vacía (para activar el frenado) si se bloquea el paso:

```python
    # En create_grid(): Bordes delgados
    for y in range(self.height_cells):
        for x in range(self.width_cells):
            if x == 0 or x == self.width_cells - 1 or y == 0 or y == self.height_cells - 1:
                grid[y][x] = 1

    # En plan_and_publish(): Frenado de emergencia
    grid_path = self.astar(start, goal)
    if len(grid_path) == 0:
        self.get_logger().warn("¡Ruta bloqueada! Cancelando.")
        empty_path = Path()
        empty_path.header.frame_id = "map"
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)
        return

```

**En `path_follower_node.py`:**
Garantiza que acepta rutas nuevas al instante (quitando el seguro de longitud) y que frena al recibir rutas vacías:

```python
    # Al inicio de path_callback()
    if len(new_path) == 0:
        self.get_logger().warn("¡Ruta vacía! Frenando motores.")
        self.path_points = []
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        return

    self.path_points = new_path
    self.current_index = 0

```

*Nota: Después de verificar estos tres archivos, recuerda siempre hacer `colcon build --packages-select pickasso_amr_2d`.*

---

### 3. Comandos de Ejecución (Para Pruebas Reales)

Levanta este sistema en 4 terminales. Ya no usaremos el publicador de segmentos discretos.

**Terminal 1 (Visualización en RViz y Simulador):**

```bash
cd ~/x_arm/
source install/setup.bash
ros2 launch pickasso_amr_2d simulation.launch.py

```

**Terminal 2 (El Cerebro de Seguimiento Continuo):**

```bash
cd ~/x_arm/
source install/setup.bash
ros2 run pickasso_amr_2d path_follower_node

```

**Terminal 3 (El Enlace Inalámbrico al Carrito):**

```bash
cd ~/x_arm/
source install/setup.bash
ros2 run pickasso_amr_2d cmd_vel_udp_bridge_node

```

*(Al iniciar este comando, tu Raspberry Pi debería indicar que comenzó a recibir datos).*

**Terminal 4 (Enviar la Orden):**

```bash
cd ~/x_arm/
source install/setup.bash
ros2 run pickasso_amr_2d test_goal_node

```

¡Ahora sí! Mientras el carrito avance por la planta buscando el punto de "Pickup", usa la herramienta de "Publish Point" en RViz para hacer clic frente a la línea verde. Verás al AMR recalcular su rumbo mediante movimientos holonómicos suaves en tiempo real, esquivando todo a su paso sin comprometer su mecánica.
