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
            if self.rechazos_consecutivos < self.max_rechazos: return self.yaw_anterior 
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
    comando = f"{motor},{direccion},{round(rpm_seguro, 1)}\n".encode('utf-8')
    try: placa.write(comando)
    except: pass 

def frenar_y_limpiar():
    mandar_orden(esp_1, "A", 0, 0); mandar_orden(esp_1, "B", 0, 0)
    mandar_orden(esp_2, "A", 0, 0); mandar_orden(esp_2, "B", 0, 0)
    try: esp_1.reset_input_buffer(); esp_2.reset_input_buffer()
    except: pass

# ==========================================
# 5. TRACCIÓN (A:1, B:2 = ADELANTE PARA AMBOS)
# ==========================================
def aplicar_velocidades(v_izq, v_der):
    """
    Motor Izquierdo (ESP_1): Adelante = A:1, B:2
    Motor Derecho   (ESP_2): Adelante = A:1, B:2
    """
    v_izq = max(-100, min(100, v_izq))
    v_der = max(-100, min(100, v_der))
    
    dir_1A = 1 if v_izq >= 0 else 2
    dir_1B = 2 if v_izq >= 0 else 1
    
    dir_2A = 1 if v_der >= 0 else 2
    dir_2B = 2 if v_der >= 0 else 1

    mandar_orden(esp_1, "A", dir_1A, abs(v_izq))
    mandar_orden(esp_1, "B", dir_1B, abs(v_izq))
    mandar_orden(esp_2, "A", dir_2A, abs(v_der))
    mandar_orden(esp_2, "B", dir_2B, abs(v_der))


# ==========================================
# 6. CONTROLADOR DINÁMICO (Con Giro Bloqueado)
# ==========================================
print("\n" + "="*40 + "\n  ESPERANDO ÓRDENES DE VISIÓN (WIFI)\n" + "="*40)

modo_giro_bloqueado = False

try:
    while True:
        tiempo_sin_datos = time.time() - datos_vision["ultima_vez"]
        
        if tiempo_sin_datos > 1.0 or not datos_vision["activo"]:
            frenar_y_limpiar()
            modo_giro_bloqueado = False
            sys.stdout.write("\r[ESPERA] Sin datos de visión o misión inactiva...       ")
            sys.stdout.flush()
            time.sleep(0.1)
            continue

        dx = datos_vision["dx"]
        dy = datos_vision["dy"]
        dist_meta = datos_vision["dist"]

        # Trigonometría
        angulo_meta = math.degrees(math.atan2(dy, dx))
        
        # Lectura de IMU y cálculo de error
        yaw_actual = obtener_yaw()
        error_yaw = angulo_meta - yaw_actual
        error_yaw = (error_yaw + 180) % 360 - 180

        # Si el error es mayor a 15 grados, forzamos el modo de giro estático
        if abs(error_yaw) > 15 and not modo_giro_bloqueado:
            frenar_y_limpiar()
            time.sleep(0.2) # Pausa dramática para asentar inercia
            modo_giro_bloqueado = True

        if modo_giro_bloqueado:
            # ----------------------------------------
            # ESTADO: GIRANDO (Prioridad IMU)
            # ----------------------------------------
            # Calculamos la fuerza de giro (min 20 para no atascarse, max 35)
            fuerza_giro = max(20, min(35, abs(error_yaw) * 0.8))
            
            if error_yaw < 0: 
                # Girar a la Derecha (Izquierda adelante, Derecha atrás)
                aplicar_velocidades(fuerza_giro, -fuerza_giro)
            else: 
                # Girar a la Izquierda (Izquierda atrás, Derecha adelante)
                aplicar_velocidades(-fuerza_giro, fuerza_giro)
                
            sys.stdout.write(f"\r-> Girando... IMU: {yaw_actual:+.1f}° / Meta: {angulo_meta:+.1f}° (Faltan {abs(error_yaw):.1f}°)   ")
            sys.stdout.flush()

            # Tolerancia de 3.5 grados para dar el giro por terminado
            if abs(error_yaw) <= 3.5:
                frenar_y_limpiar()
                modo_giro_bloqueado = False
                time.sleep(0.2) # Pausa para asentar antes de avanzar
                
        else:
            # ----------------------------------------
            # ESTADO: AVANZANDO RECTO
            # ----------------------------------------
            rpm_base = RPM_BASE if dist_meta > 0.15 else RPM_BASE * 0.6
            ajuste_recto = error_yaw * (KP_RUMBO * 0.5) 
            
            aplicar_velocidades(rpm_base - ajuste_recto, rpm_base + ajuste_recto)
            
            sys.stdout.write(f"\r-> Avanzando... Faltan: {dist_meta:.2f}m | IMU: {yaw_actual:+.1f}° | Err: {error_yaw:+.1f}°   ")
            sys.stdout.flush()
        
        # Dormimos muy poco (0.02s) para leer el IMU tan rápido como tu código viejo (50Hz)
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\n\n[!] Detenido por el usuario.")
finally:
    frenar_y_limpiar()
    esp_1.close(); esp_2.close()
    print("Sistemas apagados.")
