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
DIAMETRO_LLANTA_MM = 152.0   
PULSOS_POR_REV = 20000.0     
CIRCUNFERENCIA_M = (DIAMETRO_LLANTA_MM * math.pi) / 1000.0
PULSOS_POR_METRO = PULSOS_POR_REV / CIRCUNFERENCIA_M

# Ganancia para mantener el frente (Yaw) siempre en el mismo ángulo
KP_ESTABILIZACION = 0.2

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

filtro_yaw = FiltroIMU_AntiBrincos()

# ==========================================
# 3. INICIALIZACIÓN Y COMUNICACIÓN
# ==========================================
i2c = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)

try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1) 
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1) 
    time.sleep(2)
    print("Sistemas Mecanum en línea.")
except Exception as e:
    print(f"Error: {e}"); exit()

def obtener_yaw():
    while True:
        try:
            yaw_crudo = sensor_imu.euler[0]
            if yaw_crudo is not None: return filtro_yaw.filtrar(yaw_crudo)
        except: pass
        time.sleep(0.01)

def mandar_orden(placa, motor, direccion, rpm):
    comando = f"{motor},{direccion},{round(rpm, 1)}\n"
    placa.write(comando.encode('utf-8'))

def frenar_y_limpiar():
    mandar_orden(esp_1, "A", 0, 0); mandar_orden(esp_1, "B", 0, 0)
    mandar_orden(esp_2, "A", 0, 0); mandar_orden(esp_2, "B", 0, 0)
    esp_1.reset_input_buffer(); esp_2.reset_input_buffer()

def leer_encoders(placa):
    ultima_linea = ""
    while placa.in_waiting > 0:
        try: ultima_linea = placa.readline().decode('utf-8').rstrip()
        except: pass
    if "A:" in ultima_linea:
        try:
            partes = ultima_linea.split(',')
            return int(partes[0].split(':')[1]), int(partes[1].split(':')[1])
        except: pass
    return None, None

# ==========================================
# 4. NAVEGACIÓN MECANUM (Rectángulo sin rotar)
# ==========================================

def mover_recto(metros, direccion, rpm_base=15):
    """ Mueve adelante (dir 1) o atrás (dir 2) manteniendo el ángulo """
    print(f"-> Moviendo {metros}m (Adelante/Atrás) con estabilización IMU...")
    yaw_objetivo = obtener_yaw()
    pulsos_objetivo = int(metros * PULSOS_POR_METRO)
    
    # Usamos esp_1 para medir distancia
    enc_a_ini, _ = leer_encoders(esp_1)
    while enc_a_ini is None: enc_a_ini, _ = leer_encoders(esp_1)

    while True:
        enc_a_act, _ = leer_encoders(esp_1)
        if enc_a_act is not None and abs(enc_a_act - enc_a_ini) >= pulsos_objetivo:
            break
        
        yaw_actual = obtener_yaw()
        error_yaw = yaw_actual - yaw_objetivo
        if error_yaw > 180: error_yaw -= 360
        elif error_yaw < -180: error_yaw += 360
        
        ajuste = error_yaw * KP_ESTABILIZACION
        
        if direccion == 1: # ADELANTE
            mandar_orden(esp_1, "A", 2, rpm_base - ajuste); mandar_orden(esp_1, "B", 2, rpm_base - ajuste)
            mandar_orden(esp_2, "A", 1, rpm_base + ajuste); mandar_orden(esp_2, "B", 1, rpm_base + ajuste)
        else: # ATRÁS (direccion == 2)
            mandar_orden(esp_1, "A", 1, rpm_base + ajuste); mandar_orden(esp_1, "B", 1, rpm_base + ajuste)
            mandar_orden(esp_2, "A", 2, rpm_base - ajuste); mandar_orden(esp_2, "B", 2, rpm_base - ajuste)
        time.sleep(0.02)
    frenar_y_limpiar()

def desplazar_lateral(metros, lado, rpm_base=15):
    """ 
    Mueve a la DERECHA o IZQUIERDA sin rotar. 
    Lado: 'D' (Derecha), 'I' (Izquierda)
    """
    print(f"-> Desplazando lateralmente {metros}m hacia {lado}...")
    yaw_objetivo = obtener_yaw()
    # Las llantas Mecanum patinan un poco más de lado, aumentamos pulsos un 10%
    pulsos_objetivo = int(metros * PULSOS_POR_METRO * 1.1)
    
    enc_a_ini, _ = leer_encoders(esp_1)
    while enc_a_ini is None: enc_a_ini, _ = leer_encoders(esp_1)

    while True:
        enc_a_act, _ = leer_encoders(esp_1)
        if enc_a_act is not None and abs(enc_a_act - enc_a_ini) >= pulsos_objetivo:
            break

        error_yaw = obtener_yaw() - yaw_objetivo
        if error_yaw > 180: error_yaw -= 360
        elif error_yaw < -180: error_yaw += 360
        
        ajuste = error_yaw * KP_ESTABILIZACION

        if lado == 'D':
            # Mecanum Derecha: FrontIzq(2), BackIzq(1), FrontDer(2), BackDer(1)
            mandar_orden(esp_1, "A", 2, rpm_base - ajuste) # M1
            mandar_orden(esp_1, "B", 1, rpm_base - ajuste) # M2
            mandar_orden(esp_2, "A", 2, rpm_base + ajuste) # M3
            mandar_orden(esp_2, "B", 1, rpm_base + ajuste) # M4
        else: # IZQUIERDA
            mandar_orden(esp_1, "A", 1, rpm_base + ajuste)
            mandar_orden(esp_1, "B", 2, rpm_base + ajuste)
            mandar_orden(esp_2, "A", 1, rpm_base - ajuste)
            mandar_orden(esp_2, "B", 2, rpm_base - ajuste)
        time.sleep(0.02)
    frenar_y_limpiar()

# ==========================================
# 5. BUCLE PRINCIPAL (Rectángulo Mecanum)
# ==========================================
try:
    while True:
        print("\n RUTINA RECTÁNGULO MECANUM (Sin girar frente)")
        opcion = input("Escribe 'y' (Ejecutar), o 's' (Salir): ").strip().lower()
        
        if opcion == 'y':
            # Lado 1: Adelante
            mover_recto(metros=2.0, direccion=1)
            time.sleep(0.5)
            # Lado 2: Derecha (Desplazamiento lateral)
            desplazar_lateral(metros=1.5, lado='I')
            time.sleep(0.5)
            # Lado 3: Atrás
            mover_recto(metros=2.0, direccion=2)
            time.sleep(0.5)
            # Lado 4: Izquierda (Regresar al origen)
            desplazar_lateral(metros=1.5, lado='D')
            print("\n¡Trayectoria completada!")
        elif opcion == 's': break

except KeyboardInterrupt:
    frenar_y_limpiar()
finally:
    esp_1.close(); esp_2.close()
