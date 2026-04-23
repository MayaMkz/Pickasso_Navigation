import serial
import time
import math
import sys
import board
import busio
import adafruit_bno055
import socket
import threading

# ==========================================
# 1. CONFIGURACIÓN FÍSICA Y RED
# ==========================================
DIAMETRO_LLANTA_MM = 127.7   
PULSOS_POR_REV = 20000.0     
CIRCUNFERENCIA_M = (DIAMETRO_LLANTA_MM * math.pi) / 1000.0
PULSOS_POR_METRO = PULSOS_POR_REV / CIRCUNFERENCIA_M

KP_RUMBO = 0.5 
RPM_BASE = 25.0  # Velocidad de crucero

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
            # Espera formato: "dx,dy,dist"
            partes = data.decode('utf-8').split(',')
            if len(partes) == 3:
                datos_vision["dx"] = float(partes[0])
                datos_vision["dy"] = float(partes[1])
                datos_vision["dist"] = float(partes[2])
                datos_vision["activo"] = True
                datos_vision["ultima_vez"] = time.time()
        except: pass

threading.Thread(target=escuchar_vision, daemon=True).start()

# ==========================================
# 4. INICIALIZACIÓN HARDWARE
# ==========================================
print("Iniciando El Cerebro y Sensores...")
i2c = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)

try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1) 
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1) 
    time.sleep(2)
    print("[OK] ESP32 en línea.")
except Exception as e:
    print(f"Error de conexión USB: {e}"); sys.exit()

def obtener_yaw():
    try:
        yaw_crudo = sensor_imu.euler[0]
        if yaw_crudo is not None:
            return filtro_yaw.filtrar(yaw_crudo)
    except: pass
    return filtro_yaw.yaw_anterior if filtro_yaw.yaw_anterior else 0.0

def mandar_orden(placa, motor, direccion, rpm):
    rpm_seguro = max(0, min(abs(rpm), 100))
    placa.write(f"{motor},{direccion},{round(rpm_seguro, 1)}\n".encode('utf-8'))

def frenar_y_limpiar():
    mandar_orden(esp_1, "A", 0, 0); mandar_orden(esp_1, "B", 0, 0)
    mandar_orden(esp_2, "A", 0, 0); mandar_orden(esp_2, "B", 0, 0)
    esp_1.reset_input_buffer(); esp_2.reset_input_buffer()

def leer_encoders(placa):
    ultima_linea = ""
    while placa.in_waiting > 0:
        try: ultima_linea = placa.readline().decode('utf-8').rstrip()
        except: pass
    if "A:" in ultima_linea and "B:" in ultima_linea:
        try:
            partes = ultima_linea.split(',')
            return int(partes[0].split(':')[1]), int(partes[1].split(':')[1])
        except: pass
    return None, None

# ==========================================
# 5. CONTROLADOR DINÁMICO (LA MAGIA)
# ==========================================
print("\n" + "="*40 + "\n  ESPERANDO ÓRDENES DE VISIÓN (WIFI)\n" + "="*40)
print("Asegúrate de alinear el robot con la cámara antes de arrancar.")

try:
    while True:
        tiempo_sin_datos = time.time() - datos_vision["ultima_vez"]
        
        # Paro de seguridad si se pierde la conexión WiFi o la cámara lo pierde de vista por más de 1 segundo
        if tiempo_sin_datos > 1.0 or not datos_vision["activo"]:
            frenar_y_limpiar()
            sys.stdout.write("\r[ESPERA] Sin datos de visión o ruta inactiva...       ")
            sys.stdout.flush()
            time.sleep(0.1)
            continue

        # Extraer datos de la memoria compartida
        dx = datos_vision["dx"]
        dy = datos_vision["dy"]
        dist_meta = datos_vision["dist"]

        # 1. CÁLCULO DE ÁNGULO HACIA LA META (Trigonometría)
        # OJO: Dependiendo de cómo montaste la cámara, puede que tengas que cambiar el signo de dy
        angulo_meta = math.degrees(math.atan2(dy, dx))
        
        # 2. CÁLCULO DE ERROR DE RUMBO
        yaw_actual = obtener_yaw()
        error_yaw = angulo_meta - yaw_actual
        
        # Normalizar entre -180 y +180
        error_yaw = (error_yaw + 180) % 360 - 180

        # 3. DECISIÓN INTELIGENTE: ¿Frente o Reversa?
        sentido_marcha = 1 # 1 = Adelante, 2 = Reversa
        if abs(error_yaw) > 90:
            sentido_marcha = 2
            # Engañamos al control para que maneje hacia atrás invirtiendo la referencia 180 grados
            error_yaw = (error_yaw + 180) % 360 - 180

        # 4. CÁLCULO DE VELOCIDAD DE LLANTAS (PID Proporcional)
        # Reducir velocidad si ya estamos muy cerca
        rpm_actual = RPM_BASE if dist_meta > 0.15 else RPM_BASE * 0.6
        
        ajuste = error_yaw * KP_RUMBO
        rpm_izq = rpm_actual - ajuste
        rpm_der = rpm_actual + ajuste

        # 5. ENVIAR A LOS MOTORES
        mandar_orden(esp_1, "A", sentido_marcha, rpm_izq) 
        mandar_orden(esp_1, "B", sentido_marcha, rpm_izq) 
        mandar_orden(esp_2, "A", sentido_marcha, rpm_der) 
        mandar_orden(esp_2, "B", sentido_marcha, rpm_der) 

        # 6. LEER ENCODERS (Opcional, para telemetría)
        enc_a, _ = leer_encoders(esp_1)
        dist_rueda = (enc_a / PULSOS_POR_METRO) if enc_a else 0.0

        # HUD en Terminal
        accion = "ADELANTE" if sentido_marcha == 1 else "REVERSA "
        sys.stdout.write(f"\r[RUN] Dist: {dist_meta:.2f}m | Meta: {angulo_meta:+.0f}° | Yaw: {yaw_actual:+.0f}° | Error: {error_yaw:+.0f}° | {accion}   ")
        sys.stdout.flush()
        
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\n[!] Detenido por el usuario.")
finally:
    frenar_y_limpiar()
    esp_1.close(); esp_2.close()
    print("Sistemas apagados.")
