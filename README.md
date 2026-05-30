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

**Copiar el archivo a la Raspberry (desde tu PC):**
```bash
scp pickasso_amr.py pi@192.168.X.X:/home/pi/
```

**Conectarse a la Raspberry por SSH:**
```bash
ssh pi@192.168.X.X
```

**Correr el programa:**
```bash
cd /home/pi
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

Primero saber la IP de la Raspberry:
```bash
# En la Raspberry
hostname -I
```

Luego desde la PC:
```bash
# Opción 1: netcat (Linux/Mac)
echo "pickup" | nc -u -w1 192.168.X.X 5006

# Opción 2: si nc no está disponible, instalar
sudo apt install netcat-openbsd      # Ubuntu/Debian
brew install netcat                  # Mac

# Opción 3: desde Python en la PC (más confiable)
python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(60)
s.sendto(b'pickup', ('192.168.X.X', 5006))
resp, _ = s.recvfrom(256)
print(resp.decode())
"
```

---

**Secuencia completa de una misión (HOME → PICKUP → CLASSIFICATION → HOME):**
```bash
# Terminal 1: Raspberry corriendo el programa
python3 pickasso_amr.py

# Terminal 2: PC enviando goals uno por uno,
# esperar "OK: llegue" antes de enviar el siguiente

python3 -c "
import socket, time

IP_RASPI = '192.168.X.X'   # cambia por tu IP real
PORT     = 5006

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(120)   # 2 min máximo por movimiento

mision = ['pickup', 'classification', 'home']

for goal in mision:
    print(f'Enviando goal: {goal}')
    s.sendto(goal.encode(), (IP_RASPI, PORT))

    # Confirmación de que empezó a moverse
    resp, _ = s.recvfrom(256)
    print(f'  Raspberry: {resp.decode()}')

    # Esperar confirmación de llegada
    resp, _ = s.recvfrom(256)
    print(f'  Raspberry: {resp.decode()}')
    print()

print('Misión completa.')
"
```

---

**Si quieres que el programa arranque automáticamente al encender la Raspberry:**
```bash
# Editar crontab
crontab -e

# Agregar esta línea al final
@reboot sleep 10 && python3 /home/pi/pickasso_amr.py >> /home/pi/amr_log.txt 2>&1
```
