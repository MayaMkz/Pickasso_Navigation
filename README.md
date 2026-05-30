Buena pregunta. Tienes razón en cuestionarlo. Déjame explicar:

## ¿Por qué A* si es un programa directo en la Raspberry?

A* resuelve un problema real aunque no uses ROS: **el robot no puede ir en línea recta de HOME a PICKUP porque la mesa está en medio**. Sin planificador, el robot chocaría con la mesa o las estaciones.

```
HOME (2.40, 0.50) → PICKUP (2.55, 2.86)

Sin A*:  va directo → choca con la mesa (1.35, 1.90)
Con A*:  rodea la mesa automáticamente → llega seguro
```

El mapa y el A* están **dentro del mismo archivo** `pickasso_amr.py`, no es un programa separado. No necesitas instalar nada extra, es solo Python puro con `heapq` que ya viene en la librería estándar.

---

## Comandos detallados para ejecutar

**En la Raspberry Pi — instalar dependencias (solo la primera vez):**
```bash
pip3 install pyserial adafruit-circuitpython-bno055
```

**Verificar que los ESP32 están conectados:**
```bash
ls /dev/ttyUSB*
# Debes ver /dev/ttyUSB0 y /dev/ttyUSB1
```

**Si no aparecen, dar permisos:**
```bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```



**Conectarse a la Raspberry por SSH:**
```bash
ssh merg@Xarm5.loca1
```
Para esto tienes que estar conectada via ethernet si aplicas local, o desde la raspberry pi connect tambien se puede. O igual este comando sirve si la compu y la raspberry se encuentran en la misma red. EN TODO CASO DE QUE NO 

ssh merg@10.42.0.234

**Correr el programa:**
```bash
cd /Carrito
python3 pickasso_amr.py
```

Deberías ver esto:
```
[OK] IMU BNO055 conectado.
[OK] ESP32 conectados.
============================================
  Pickasso AMR listo. Escuchando UDP :5006
  Posición inicial: HOME (2.40, 0.50)
  Goals disponibles:
    PICKUP         → 2.550,2.527
    CLASSIFICATION → 0.830,0.700
    HOME           → 2.400,0.500
  Enviar: echo "x,y" | nc -u <IP_RASPI> 5006
============================================

Esperando goal...
```

---

**Desde tu PC — enviar goals (en otra terminal):**

```

Luego desde la PC:
```bash
# Opción 1: netcat (Linux)
echo "pickup" | nc -u -w1 192.168.X.X 5006
```
# Opción 2: si nc no está disponible, instalar
sudo apt install netcat-openbsd      # Ubuntu/Debian
brew install netcat                  # Mac



---

**Secuencia completa de una misión (HOME → PICKUP → CLASSIFICATION → HOME):**

```bash
# Terminal 1: Raspberry corriendo el programa
python3 pickasso_amr.py
```
# Terminal 2: PC enviando goals uno por uno,
```bash
# Opción 1: netcat (Linux)
echo "pickup" | nc -u -w1 192.168.X.X 5006

# esperar "OK: llegue" antes de enviar el siguiente

```

**Si quieres que el programa arranque automáticamente al encender la Raspberry:**
```bash
# Editar crontab
crontab -e

# Agregar esta línea al final
@reboot sleep 10 && python3 /home/Carrito/pickasso_amr.py >> /home/Carrito/amr_log.txt 2>&1
```


```
```



Use command line (xArm):

cd ~/dev_ws/
# launch xarm_driver_node:
ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.117

# enable all joints:
ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"

# set proper mode (0) and state (0)
ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"

# Cartesian linear motion: (unit: mm, rad)
ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian "{pose: [300, 0, 250, 3.14, 0, 0], speed: 50, acc: 500, mvtime: 0}"   

# joint motion for xArm6: (unit: rad)
ros2 service call /xarm/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [-0.58, 0, 0, 0, 0, 0], speed: 0.35, acc: 10, mvtime: 0}"
Use command line (lite6):
