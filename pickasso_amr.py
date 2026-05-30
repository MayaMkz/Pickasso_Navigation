"""
pickasso_amr.py — Pickasso AMR | Raspberry Pi
==============================================
Programa autónomo completo. No necesita ROS 2.

Flujo:
  1. Espera un goal (x, y) por UDP desde la PC
  2. Planifica la ruta con A* ortogonal sobre el mapa de config
  3. Ejecuta cada segmento F/B/R/L con encoders + corrección IMU
  4. Confirma llegada de vuelta a la PC

Enviar goal desde PC:
  echo "2.55,2.86" | nc -u 192.168.X.X 5006
  o desde Python:
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(b"2.55,2.86", ("192.168.X.X", 5006))
"""

import serial
import time
import math
import socket
import heapq
import sys

# ══════════════════════════════════════════════════════════════════════════════
#  MAPA — igual que config.py de ROS 2
# ══════════════════════════════════════════════════════════════════════════════
AREA_WIDTH  = 3.0
AREA_HEIGHT = 3.8
RESOLUTION  = 0.05

ROBOT_LENGTH = 0.66
ROBOT_WIDTH  = 0.70

SAFETY_MARGIN       = 0.01
TABLE_SAFETY_MARGIN = 0.15

HOME_POSE           = (2.40, 0.50)

PICKUP_BOX_CENTER   = (2.55, 3.20)
PICKUP_BOX_SIZE     = (0.34, 0.27)

CLASSIFICATION_BOX_CENTER = (0.45, 0.70)
CLASSIFICATION_BOX_SIZE   = (0.40, 0.39)

TABLE_CENTER = (1.35, 1.90)
TABLE_SIZE   = (0.60, 0.70)

STATIONS = [
    {"name": "pickup_box",         "center": PICKUP_BOX_CENTER,        "size": PICKUP_BOX_SIZE},
    {"name": "classification_box", "center": CLASSIFICATION_BOX_CENTER, "size": CLASSIFICATION_BOX_SIZE},
    {"name": "table",              "center": TABLE_CENTER,              "size": TABLE_SIZE},
]

APPROACH_GAP             = 0.15
FRONT_APPROACH_DISTANCE  = (ROBOT_LENGTH / 2.0) + APPROACH_GAP

PICKUP_POSE = (
    PICKUP_BOX_CENTER[0],
    PICKUP_BOX_CENTER[1] - (PICKUP_BOX_SIZE[1] / 2.0) - FRONT_APPROACH_DISTANCE,
)

CLASSIFICATION_POSE = (
    CLASSIFICATION_BOX_CENTER[0] + (CLASSIFICATION_BOX_SIZE[0] / 2.0) + FRONT_APPROACH_DISTANCE,
    CLASSIFICATION_BOX_CENTER[1],
)

# ══════════════════════════════════════════════════════════════════════════════
#  HARDWARE
# ══════════════════════════════════════════════════════════════════════════════
DIAMETRO_LLANTA_MM = 152.0
CIRCUNFERENCIA_M   = (DIAMETRO_LLANTA_MM * math.pi) / 1000.0
PULSOS_POR_REV     = 20000.0
PULSOS_POR_METRO   = PULSOS_POR_REV / CIRCUNFERENCIA_M

RPM_BASE           = 15
KP_ESTABILIZACION  = 0.2      # igual que Holo_cuadrado.py
MIN_SEGMENTO_M     = 0.05     # segmentos menores a esto se ignoran

# ══════════════════════════════════════════════════════════════════════════════
#  IMU — igual que Holo_cuadrado.py
# ══════════════════════════════════════════════════════════════════════════════
try:
    import board, busio, adafruit_bno055
    i2c        = busio.I2C(board.SCL, board.SDA)
    sensor_imu = adafruit_bno055.BNO055_I2C(i2c)
    IMU_OK     = True
    print("[OK] IMU BNO055 conectado.")
except Exception as e:
    print(f"[WARN] IMU no disponible: {e}")
    IMU_OK = False

class FiltroIMU_AntiBrincos:
    def __init__(self, umbral=10.0, max_rechazos=5):
        self.yaw_ant  = None
        self.umbral   = umbral
        self.rechazos = 0
        self.max_r    = max_rechazos

    def filtrar(self, yaw):
        if self.yaw_ant is None:
            self.yaw_ant = yaw; return yaw
        d = yaw - self.yaw_ant
        if d >  180: d -= 360
        if d < -180: d += 360
        if abs(d) > self.umbral:
            self.rechazos += 1
            if self.rechazos < self.max_r:
                return self.yaw_ant
            self.yaw_ant = yaw; self.rechazos = 0; return yaw
        self.rechazos = 0; self.yaw_ant = yaw; return yaw

filtro_yaw = FiltroIMU_AntiBrincos()

def obtener_yaw():
    if not IMU_OK:
        return 0.0
    try:
        y = sensor_imu.euler[0]
        if y is not None:
            return filtro_yaw.filtrar(y)
    except:
        pass
    return filtro_yaw.yaw_ant or 0.0

# ══════════════════════════════════════════════════════════════════════════════
#  SERIAL ESP32
# ══════════════════════════════════════════════════════════════════════════════
try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.05)
    time.sleep(2)
    print("[OK] ESP32 conectados.")
except Exception as e:
    print(f"[ERROR] ESP32: {e}"); sys.exit(1)

def _cmd(placa, motor, dir_, rpm):
    rpm = max(0.0, min(80.0, abs(rpm)))
    try:
        placa.write(f"{motor},{dir_},{rpm:.1f}\n".encode())
    except:
        pass

def frenar_y_limpiar():
    for p in (esp_1, esp_2):
        _cmd(p, "A", 0, 0); _cmd(p, "B", 0, 0)
    try:
        esp_1.reset_input_buffer(); esp_2.reset_input_buffer()
    except:
        pass

def leer_encoders(placa):
    ultima = ""
    while placa.in_waiting > 0:
        try:
            ultima = placa.readline().decode("utf-8", errors="ignore").strip()
        except:
            pass
    if "A:" in ultima:
        try:
            p = ultima.split(",")
            return int(p[0].split(":")[1]), int(p[1].split(":")[1])
        except:
            pass
    return None, None

def _enc_inicial(placa):
    enc, _ = leer_encoders(placa)
    while enc is None:
        enc, _ = leer_encoders(placa)
    return enc

# ══════════════════════════════════════════════════════════════════════════════
#  MOVIMIENTOS con encoders + IMU — igual que Holo_cuadrado.py
# ══════════════════════════════════════════════════════════════════════════════

def mover_adelante(metros, rpm=RPM_BASE):
    if metros < MIN_SEGMENTO_M:
        return
    print(f"  [F] {metros:.3f} m")
    pulsos_obj = int(metros * PULSOS_POR_METRO)
    yaw_obj    = obtener_yaw()
    enc_ini    = _enc_inicial(esp_1)
    while True:
        enc, _ = leer_encoders(esp_1)
        if enc is not None and abs(enc - enc_ini) >= pulsos_obj:
            break
        err = obtener_yaw() - yaw_obj
        if err >  180: err -= 360
        if err < -180: err += 360
        a = err * KP_ESTABILIZACION
        _cmd(esp_1, "A", 2, rpm - a); _cmd(esp_1, "B", 2, rpm - a)
        _cmd(esp_2, "A", 1, rpm + a); _cmd(esp_2, "B", 1, rpm + a)
        time.sleep(0.02)
    frenar_y_limpiar()

def mover_atras(metros, rpm=RPM_BASE):
    if metros < MIN_SEGMENTO_M:
        return
    print(f"  [B] {metros:.3f} m")
    pulsos_obj = int(metros * PULSOS_POR_METRO)
    yaw_obj    = obtener_yaw()
    enc_ini    = _enc_inicial(esp_1)
    while True:
        enc, _ = leer_encoders(esp_1)
        if enc is not None and abs(enc - enc_ini) >= pulsos_obj:
            break
        err = obtener_yaw() - yaw_obj
        if err >  180: err -= 360
        if err < -180: err += 360
        a = err * KP_ESTABILIZACION
        _cmd(esp_1, "A", 1, rpm + a); _cmd(esp_1, "B", 1, rpm + a)
        _cmd(esp_2, "A", 2, rpm - a); _cmd(esp_2, "B", 2, rpm - a)
        time.sleep(0.02)
    frenar_y_limpiar()

def mover_derecha(metros, rpm=RPM_BASE):
    if metros < MIN_SEGMENTO_M:
        return
    print(f"  [R] {metros:.3f} m")
    pulsos_obj = int(metros * PULSOS_POR_METRO * 1.10)
    yaw_obj    = obtener_yaw()
    enc_ini    = _enc_inicial(esp_1)
    while True:
        enc, _ = leer_encoders(esp_1)
        if enc is not None and abs(enc - enc_ini) >= pulsos_obj:
            break
        err = obtener_yaw() - yaw_obj
        if err >  180: err -= 360
        if err < -180: err += 360
        a = err * KP_ESTABILIZACION
        _cmd(esp_1, "A", 2, rpm - a); _cmd(esp_1, "B", 1, rpm - a)
        _cmd(esp_2, "A", 2, rpm + a); _cmd(esp_2, "B", 1, rpm + a)
        time.sleep(0.02)
    frenar_y_limpiar()

def mover_izquierda(metros, rpm=RPM_BASE):
    if metros < MIN_SEGMENTO_M:
        return
    print(f"  [L] {metros:.3f} m")
    pulsos_obj = int(metros * PULSOS_POR_METRO * 1.10)
    yaw_obj    = obtener_yaw()
    enc_ini    = _enc_inicial(esp_1)
    while True:
        enc, _ = leer_encoders(esp_1)
        if enc is not None and abs(enc - enc_ini) >= pulsos_obj:
            break
        err = obtener_yaw() - yaw_obj
        if err >  180: err -= 360
        if err < -180: err += 360
        a = err * KP_ESTABILIZACION
        _cmd(esp_1, "A", 1, rpm + a); _cmd(esp_1, "B", 2, rpm + a)
        _cmd(esp_2, "A", 1, rpm - a); _cmd(esp_2, "B", 2, rpm - a)
        time.sleep(0.02)
    frenar_y_limpiar()

# ══════════════════════════════════════════════════════════════════════════════
#  PLANIFICADOR A* ORTOGONAL — igual que planner_node.py
# ══════════════════════════════════════════════════════════════════════════════
WIDTH_CELLS  = int(AREA_WIDTH  / RESOLUTION)
HEIGHT_CELLS = int(AREA_HEIGHT / RESOLUTION)

def crear_grid():
    grid = [[0] * WIDTH_CELLS for _ in range(HEIGHT_CELLS)]

    # Borde exterior
    for y in range(HEIGHT_CELLS):
        for x in range(WIDTH_CELLS):
            if x == 0 or x == WIDTH_CELLS - 1 or y == 0 or y == HEIGHT_CELLS - 1:
                grid[y][x] = 1

    # Estaciones y mesa
    for station in STATIONS:
        cx, cy = station["center"]
        sx, sy = station["size"]
        margin = TABLE_SAFETY_MARGIN if station["name"] == "table" else SAFETY_MARGIN
        sx_inf = sx + ROBOT_LENGTH + 2.0 * margin
        sy_inf = sy + ROBOT_WIDTH  + 2.0 * margin
        min_x  = int((cx - sx_inf / 2.0) / RESOLUTION)
        max_x  = int((cx + sx_inf / 2.0) / RESOLUTION)
        min_y  = int((cy - sy_inf / 2.0) / RESOLUTION)
        max_y  = int((cy + sy_inf / 2.0) / RESOLUTION)
        for y in range(max(0, min_y), min(HEIGHT_CELLS, max_y + 1)):
            for x in range(max(0, min_x), min(WIDTH_CELLS, max_x + 1)):
                grid[y][x] = 1

    return grid

def world_to_grid(x, y):
    return int(x / RESOLUTION), int(y / RESOLUTION)

def grid_to_world(gx, gy):
    return gx * RESOLUTION, gy * RESOLUTION

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    neighbors = [(1,0),(-1,0),(0,1),(0,-1)]
    open_set  = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score   = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for dx, dy in neighbors:
            nx, ny = current[0] + dx, current[1] + dy
            if nx < 0 or ny < 0 or nx >= WIDTH_CELLS or ny >= HEIGHT_CELLS:
                continue
            if grid[ny][nx] == 1:
                continue
            neighbor = (nx, ny)
            tg = g_score[current] + 1
            if neighbor not in g_score or tg < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor]   = tg
                heapq.heappush(open_set, (tg + heuristic(neighbor, goal), neighbor))
    return []

def find_free(grid, gx, gy, max_r=15):
    visited = set()
    queue   = [(gx, gy)]
    visited.add((gx, gy))
    while queue:
        cx, cy = queue.pop(0)
        if grid[cy][cx] == 0:
            return (cx, cy)
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = cx+dx, cy+dy
            if (nx,ny) not in visited and 0<=nx<WIDTH_CELLS and 0<=ny<HEIGHT_CELLS:
                if abs(nx-gx)<=max_r and abs(ny-gy)<=max_r:
                    visited.add((nx,ny)); queue.append((nx,ny))
    return None

def comprimir_path(path):
    """Conserva solo los vértices donde cambia la dirección."""
    if len(path) <= 2:
        return path
    compressed = [path[0]]
    prev_dir   = None
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        curr_dir = (dx, dy)
        if curr_dir != prev_dir:
            if i > 1:
                compressed.append(path[i-1])
            prev_dir = curr_dir
    compressed.append(path[-1])
    return compressed

def planificar(grid, start_world, goal_world):
    sx, sy = world_to_grid(*start_world)
    gx, gy = world_to_grid(*goal_world)

    if grid[sy][sx] == 1:
        libre = find_free(grid, sx, sy)
        if libre is None:
            print("[ERROR] Start bloqueado y sin celda libre cercana.")
            return []
        sx, sy = libre

    if grid[gy][gx] == 1:
        libre = find_free(grid, gx, gy)
        if libre is None:
            print("[ERROR] Goal bloqueado y sin celda libre cercana.")
            return []
        gx, gy = libre

    raw  = astar(grid, (sx, sy), (gx, gy))
    if not raw:
        print("[ERROR] A* no encontró ruta.")
        return []

    return comprimir_path(raw)

# ══════════════════════════════════════════════════════════════════════════════
#  EJECUCIÓN DEL PATH
# ══════════════════════════════════════════════════════════════════════════════

def ejecutar_path(path_grid):
    """
    Recorre el path comprimido segmento a segmento.
    Cada segmento es puro F/B o puro R/L (ortogonal).
    """
    if len(path_grid) < 2:
        print("[WARN] Path vacío o de un solo punto.")
        return

    for i in range(len(path_grid) - 1):
        x1, y1 = grid_to_world(*path_grid[i])
        x2, y2 = grid_to_world(*path_grid[i+1])

        dx = round(x2 - x1, 6)
        dy = round(y2 - y1, 6)

        # Eje Y del mapa → F/B
        if abs(dy) >= MIN_SEGMENTO_M:
            if dy > 0:
                mover_adelante(abs(dy))
            else:
                mover_atras(abs(dy))

        # Eje X del mapa → R/L
        if abs(dx) >= MIN_SEGMENTO_M:
            if dx > 0:
                mover_derecha(abs(dx))
            else:
                mover_izquierda(abs(dx))

        time.sleep(0.15)   # pequeña pausa entre segmentos

    frenar_y_limpiar()
    print("[✓] Ruta completada.")

# ══════════════════════════════════════════════════════════════════════════════
#  SERVIDOR UDP — escucha goals desde la PC
# ══════════════════════════════════════════════════════════════════════════════
UDP_PORT = 5006

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
print(f"\n{'='*44}")
print(f"  Pickasso AMR listo. Escuchando UDP :{UDP_PORT}")
print(f"  Posición inicial: HOME {HOME_POSE}")
print(f"  Goals disponibles:")
print(f"    PICKUP         → {PICKUP_POSE[0]:.3f},{PICKUP_POSE[1]:.3f}")
print(f"    CLASSIFICATION → {CLASSIFICATION_POSE[0]:.3f},{CLASSIFICATION_POSE[1]:.3f}")
print(f"    HOME           → {HOME_POSE[0]:.3f},{HOME_POSE[1]:.3f}")
print(f"  Enviar: echo \"x,y\" | nc -u <IP_RASPI> {UDP_PORT}")
print(f"{'='*44}\n")

# Posición actual estimada (se actualiza tras cada movimiento)
pos_actual = list(HOME_POSE)

# Grid estático (sin obstáculos dinámicos por ahora)
grid = crear_grid()

try:
    while True:
        print("Esperando goal...")
        data, addr = sock.recvfrom(1024)
        msg = data.decode("utf-8").strip()
        print(f"← Goal recibido de {addr}: '{msg}'")

        # Formato esperado: "x,y"  ej: "2.55,2.86"
        # También acepta nombres: "pickup", "classification", "home"
        msg_lower = msg.lower()
        if msg_lower == "pickup":
            goal = PICKUP_POSE
        elif msg_lower in ("classification", "class"):
            goal = CLASSIFICATION_POSE
        elif msg_lower == "home":
            goal = HOME_POSE
        else:
            try:
                partes = msg.split(",")
                goal   = (float(partes[0]), float(partes[1]))
            except Exception as e:
                print(f"[ERROR] Formato inválido: {e}. Usa 'x,y' o 'pickup/classification/home'")
                sock.sendto(b"ERROR: formato invalido", addr)
                continue

        print(f"[→] De {pos_actual} a {goal}")

        path = planificar(grid, tuple(pos_actual), goal)

        if not path:
            sock.sendto(b"ERROR: sin ruta", addr)
            continue

        print(f"[✓] Ruta: {len(path)} segmentos")
        sock.sendto(f"OK: ejecutando {len(path)} segmentos".encode(), addr)

        ejecutar_path(path)

        # Actualizar posición actual al goal alcanzado
        pos_actual = list(goal)

        sock.sendto(b"OK: llegue", addr)
        print(f"[✓] En posición: {pos_actual}\n")

except KeyboardInterrupt:
    print("\n[!] Detenido por el usuario.")
finally:
    frenar_y_limpiar()
    esp_1.close()
    esp_2.close()
    sock.close()
