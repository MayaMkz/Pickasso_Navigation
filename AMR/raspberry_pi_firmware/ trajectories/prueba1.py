import time
import math
import sys
import board
import busio
import adafruit_bno055
import socket
import threading
import serial

# ==========================================
# 1. CONFIGURACIÓN FÍSICA Y RED
# ==========================================
DIAMETRO_LLANTA_MM = 127.7
PULSOS_POR_REV = 20000.0
CIRCUNFERENCIA_M = (DIAMETRO_LLANTA_MM * math.pi) / 1000.0
PULSOS_POR_METRO = PULSOS_POR_REV / CIRCUNFERENCIA_M

KP_RUMBO = 0.5
RPM_BASE = 25.0

PUERTO_UDP = 5005
datos_vision = {"dx": 0.0, "dy": 0.0, "dist": 0.0, "activo": False, "ultima_vez": time.time()}

# ==========================================
# 2. CLASE FILTRO IMU (Anti-Brincos)
# ==========================================
class FiltroIMU_AntiBrincos:
    def __init__(self, umbral_salto_grados=10.0, max_rechazos=5):
        self.yaw_anterior = None
        self.umbral_salto = umbral_salto_grados
        self.rechazos_consecutivos = 0
        self.max_rechazos = max_rechazos

    def filtrar(self, yaw_nuevo):
        if self.yaw_anterior is None:
            self.yaw_anterior = yaw_nuevo
            return yaw_nuevo
        diferencia = yaw_nuevo - self.yaw_anterior
        if diferencia > 180: diferencia -= 360
        elif diferencia < -180: diferencia += 360
        if abs(diferencia) > self.umbral_salto:
            self.rechazos_consecutivos += 1
            if self.rechazos_consecutivos < self.max_rechazos:
                return self.yaw_anterior
            else:
                self.yaw_anterior = yaw_nuevo
                self.rechazos_consecutivos = 0
                return yaw_nuevo
        self.rechazos_consecutivos = 0
        self.yaw_anterior = yaw_nuevo
        return yaw_nuevo

filtro_yaw = FiltroIMU_AntiBrincos(umbral_salto_grados=10.0, max_rechazos=5)

# ==========================================
# 3. HILO DE RED (Escucha a la PC)
# ==========================================
def escuchar_vision():
    global datos_vision
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PUERTO_UDP))
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            partes = data.decode('utf-8').split(',')
            if len(partes) == 3:
                datos_vision["dx"] = float(partes[0])
                datos_vision["dy"] = float(partes[1])
                datos_vision["dist"] = float(partes[2])
                datos_vision["activo"] = True
                datos_vision["ultima_vez"] = time.time()
        except:
            pass

threading.Thread(target=escuchar_vision, daemon=True).start()

# ==========================================
# 4. INICIALIZACIÓN HARDWARE
# ==========================================
print("Iniciando El Cerebro, Sensores y Encoders...")
i2c = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)

try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
    time.sleep(2)
    print("[OK] ESP32 en línea.")
except Exception as e:
    print(f"Error de conexión USB: {e}")
    sys.exit()

def obtener_yaw():
    try:
        yaw_crudo = sensor_imu.euler[0]
        if yaw_crudo is not None:
            return filtro_yaw.filtrar(yaw_crudo)
    except:
        pass
    return filtro_yaw.yaw_anterior if filtro_yaw.yaw_anterior else 0.0

def leer_encoders(placa):
    ultima_linea = ""
    while placa.in_waiting > 0:
        try:
            ultima_linea = placa.readline().decode('utf-8').rstrip()
        except:
            pass
    if "A:" in ultima_linea and "B:" in ultima_linea:
        try:
            partes = ultima_linea.split(',')
            return int(partes[0].split(':')[1]), int(partes[1].split(':')[1])
        except:
            pass
    return None, None

def mandar_orden(placa, motor, direccion, rpm):
    rpm_seguro = max(0, min(abs(rpm), 100))
    comando = f"{motor},{direccion},{round(rpm_seguro, 1)}\n".encode('utf-8')
    try:
        placa.write(comando)
    except:
        pass

def frenar_y_limpiar():
    mandar_orden(esp_1, "A
