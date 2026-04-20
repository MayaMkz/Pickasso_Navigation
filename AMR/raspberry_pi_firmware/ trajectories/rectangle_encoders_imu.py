import serial
import time
import math
import sys
import board
import busio
import adafruit_bno055

# ==========================================
# 1. CONFIGURACIÓN FÍSICA Y CINEMÁTICA
# ==========================================
DIAMETRO_LLANTA_MM = 127.7   
PULSOS_POR_REV = 20000.0     
CIRCUNFERENCIA_M = (DIAMETRO_LLANTA_MM * math.pi) / 1000.0
PULSOS_POR_METRO = PULSOS_POR_REV / CIRCUNFERENCIA_M

KP_RUMBO = 0.5 # Ajusta entre 0.3 y 0.7 según estabilidad

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
# 3. INICIALIZACIÓN
# ==========================================
print("Iniciando El Cerebro y Sensores...")
i2c = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)

try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1) 
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1) 
    time.sleep(2)
    print("Sistemas en línea para navegación rectangular.")
except Exception as e:
    print(f"Error de conexión USB: {e}"); exit()

def obtener_yaw():
    while True:
        try:
            yaw_crudo = sensor_imu.euler[0]
            if yaw_crudo is not None:
                return filtro_yaw.filtrar(yaw_crudo)
        except: pass
        time.sleep(0.01)

def mandar_orden(placa, motor, direccion, rpm):
    placa.write(f"{motor},{direccion},{round(rpm, 1)}\n".encode('utf-8'))

def frenar_y_limpiar():
    mandar_orden(esp_1, "A", 0, 0); mandar_orden(esp_1, "B", 0, 0)
    mandar_orden(esp_2, "A", 0, 0); mandar_orden(esp_2, "B", 0, 0)
    time.sleep(0.1) 
    esp_1.reset_input_buffer(); esp_1.reset_output_buffer()
    esp_2.reset_input_buffer(); esp_2.reset_output_buffer()

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
# 4. FUNCIONES DE TRAYECTORIA
# ==========================================
def mover_recto(metros, direccion, rpm_base=25):
    print(f"\n-> Tramo: {metros}m a {rpm_base} RPM...")
    esp_1.reset_input_buffer()
    pulsos_objetivo = int(metros * PULSOS_POR_METRO)
    enc_a_ini, _ = leer_encoders(esp_1)
    while enc_a_ini is None: enc_a_ini, _ = leer_encoders(esp_1)
    yaw_objetivo = obtener_yaw()

    while True:
        enc_a_act, _ = leer_encoders(esp_1)
        if enc_a_act is not None:
            distancia_recorrida = abs(enc_a_act - enc_a_ini)
            sys.stdout.write(f"\r   Progreso: {(distancia_recorrida/pulsos_objetivo)*100:.1f}% ")
            if distancia_recorrida >= pulsos_objetivo:
                frenar_y_limpiar(); break
        
        yaw_actual = obtener_yaw()
        error_yaw = yaw_actual - yaw_objetivo
        if error_yaw > 180: error_yaw -= 360
        elif error_yaw < -180: error_yaw += 360
        ajuste = error_yaw * KP_RUMBO

        # Aplicar corrección diferencial
        rpm_izq = max(10, min(rpm_base - ajuste, 100))
        rpm_der = max(10, min(rpm_base + ajuste, 100))
        mandar_orden(esp_1, "A", 1, rpm_izq); mandar_orden(esp_1, "B", 2, rpm_izq) 
        mandar_orden(esp_2, "A", 1, rpm_der); mandar_orden(esp_2, "B", 2, rpm_der) 
        time.sleep(0.02)

def girar_grados(grados, rpm_giro=25):
    print(f"\n-> Girando {grados} grados...")
    yaw_ini = obtener_yaw()
    if grados > 0: # Derecha
        mandar_orden(esp_1, "A", 1, rpm_giro); mandar_orden(esp_1, "B", 2, rpm_giro) 
        mandar_orden(esp_2, "A", 2, rpm_giro); mandar_orden(esp_2, "B", 1, rpm_giro) 
    else: # Izquierda
        mandar_orden(esp_1, "A", 2, rpm_giro); mandar_orden(esp_1, "B", 1, rpm_giro) 
        mandar_orden(esp_2, "A", 1, rpm_giro); mandar_orden(esp_2, "B", 2, rpm_giro) 
    while True:
        yaw_act = obtener_yaw()
        giro = yaw_act - yaw_ini
        if giro > 180: giro -= 360
        elif giro < -180: giro += 360
        sys.stdout.write(f"\r   IMU: {abs(giro):.1f}° / {abs(grados)}° ")
        if abs(giro) >= (abs(grados) - 1.5):
            frenar_y_limpiar(); break
        time.sleep(0.01)

# ==========================================
# 5. MENÚ DE OPERACIÓN
# ==========================================
try:
    while True:
        print("\n" + "="*40 + "\n  CONTROLADOR DE TRAYECTORIA RECTANGULAR\n" + "="*40)
        opcion = input("Escribe 'y' (Rectángulo 1.3x0.6m), o 's' (Salir): ").strip().lower()
        if opcion == 's': frenar_y_limpiar(); break
        elif opcion == 'y':
            print("\nIniciando en 3, 2, 1..."); time.sleep(3)
            # Lado 1 (Largo)
            mover_recto(1.3, 2); time.sleep(1); girar_grados(90); time.sleep(1)
            # Lado 2 (Ancho)
            mover_recto(0.6, 2); time.sleep(1); girar_grados(90); time.sleep(1)
            # Lado 3 (Largo)
            mover_recto(1.3, 2); time.sleep(1); girar_grados(90); time.sleep(1)
            # Lado 4 (Ancho)
            mover_recto(0.6, 2); time.sleep(1); girar_grados(90)
            print("\n¡Rectángulo completado!")
except KeyboardInterrupt:
    frenar_y_limpiar()
finally:
    esp_1.close(); esp_2.close(); print("Sistema cerrado.")
