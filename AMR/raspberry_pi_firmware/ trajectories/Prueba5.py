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
# Medidas del robot diferencial (Ajustar L_WHEELBASE_M físicamente)
L_WHEELBASE_M = 0.30       # Distancia en metros entre el centro de la llanta izq y der
DIAMETRO_LLANTA_MM = 127.7
CIRCUNFERENCIA_M = (DIAMETRO_LLANTA_MM * math.pi) / 1000.0

# PID de Velocidad Angular (Giroscopio)
# Aumentar si no toma la curva lo suficientemente fuerte, bajar si oscila.
KP_OMEGA = 0.8  

PUERTO_UDP = 5005

# ==========================================
# ESTADO COMPARTIDO ENTRE THREADS
# ==========================================
vision_lock = threading.Lock()
nuevo_dato_event = threading.Event()

datos_vision = {
    "v_lineal": 0.0, 
    "omega_ref": 0.0, 
    "dist": 1.0,
    "estado_mision": 0,
    "activo": False, 
    "ultima_vez": 0.0
}

# ==========================================
# 2. INICIALIZACIÓN DE HARDWARE
# ==========================================
print("Iniciando hardware...")
i2c = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)

try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.05)
    time.sleep(2)
    print("[OK] ESP32 en línea.")
except Exception as e:
    print(f"Error USB: {e}"); sys.exit()

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

def aplicar_velocidades_recto(rpm_izq, rpm_der):
    # Limitar RPMs físicos por seguridad
    rpm_izq = max(-80, min(80, rpm_izq))
    rpm_der = max(-80, min(80, rpm_der))
    
    # 1: Adelante, 2: Atrás
    dir_1A, dir_1B = (1, 2) if rpm_izq >= 0 else (2, 1)
    dir_2A, dir_2B = (1, 2) if rpm_der >= 0 else (2, 1)
    
    mandar_orden(esp_1, "A", dir_1A, abs(rpm_izq))
    mandar_orden(esp_1, "B", dir_1B, abs(rpm_izq))
    mandar_orden(esp_2, "A", dir_2A, abs(rpm_der))
    mandar_orden(esp_2, "B", dir_2B, abs(rpm_der))

def leer_encoders_seguro(placa):
    placa.reset_input_buffer()
    time.sleep(0.01)
    ultima_linea = ""
    for _ in range(5):
        while placa.in_waiting > 0:
            try: ultima_linea = placa.readline().decode('utf-8').rstrip()
            except: pass
        if "A:" in ultima_linea and "B:" in ultima_linea:
            try:
                partes = ultima_linea.split(',')
                return int(partes[0].split(':')[1]), int(partes[1].split(':')[1])
            except: pass
        time.sleep(0.01)
    return 0, 0

# ==========================================
# 3. DIAGNÓSTICO
# ==========================================
def diagnostico_inicial_motores():
    print("\n" + "="*40)
    print(" DIAGNÓSTICO DE HARDWARE...")
    print("="*40)
    while True:
        frenar_y_limpiar()
        time.sleep(0.2)
        e1A_ini, e1B_ini = leer_encoders_seguro(esp_1)
        e2A_ini, e2B_ini = leer_encoders_seguro(esp_2)
        print("[!] Pulso de prueba (15 RPM)...")
        aplicar_velocidades_recto(15, 15)
        time.sleep(0.5)
        frenar_y_limpiar()
        time.sleep(0.2)
        e1A_fin, e1B_fin = leer_encoders_seguro(esp_1)
        e2A_fin, e2B_fin = leer_encoders_seguro(esp_2)
        dif_1A = abs(e1A_fin - e1A_ini); dif_1B = abs(e1B_fin - e1B_ini)
        dif_2A = abs(e2A_fin - e2A_ini); dif_2B = abs(e2B_fin - e2B_ini)
        if dif_1A < 50 or dif_1B < 50 or dif_2A < 50 or dif_2B < 50:
            print("\n[ERROR] Falla en motores/encoders.")
            input(">>> Presiona ENTER cuando revises las conexiones...")
        else:
            print("[OK] 4 motores y encoders responden correctamente.")
            break

diagnostico_inicial_motores()

# ==========================================
# 4. THREAD A — RECEPTOR UDP (Productor)
# ==========================================
def hilo_escuchar_vision():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PUERTO_UDP))
    sock.setblocking(False)
    print(f"[UDP] Escuchando en puerto {PUERTO_UDP}...")

    while True:
        dato_fresco = None
        try:
            while True:
                data, _ = sock.recvfrom(1024)
                dato_fresco = data 
        except BlockingIOError:
            pass 

        if dato_fresco is not None:
            try:
                partes = dato_fresco.decode('utf-8').split(',')
                if len(partes) >= 4:
                    with vision_lock:
                        datos_vision["v_lineal"]      = float(partes[0])
                        datos_vision["omega_ref"]     = float(partes[1])
                        datos_vision["dist"]          = float(partes[2])
                        datos_vision["estado_mision"] = int(partes[3])
                        datos_vision["activo"]        = True
                        datos_vision["ultima_vez"]    = time.time()
                    nuevo_dato_event.set()
            except Exception:
                pass
        time.sleep(0.005)

# ==========================================
# 5. THREAD B — CONTROL PID CINEMÁTICO (50 Hz)
# ==========================================
pид_activo = threading.Event()
pид_activo.set()

def hilo_pid_imu():
    periodo = 0.02  # 50 Hz para lectura de giroscopio

    while True:
        if not pид_activo.is_set():
            time.sleep(periodo)
            continue

        with vision_lock:
            v_ref = datos_vision.get("v_lineal", 0.0)
            omega_ref = datos_vision.get("omega_ref", 0.0)
            activo_local = datos_vision.get("activo", False)
            ultima = datos_vision.get("ultima_vez", 0.0)

        # Si perdemos visión o nos piden detenernos
        if not activo_local or (time.time() - ultima > 1.0) or v_ref == 0.0:
            frenar_y_limpiar()
            time.sleep(periodo)
            continue

        # Leer velocidad angular real desde el giroscopio (Eje Z) en rad/s
        try:
            # Si el sensor se montó con Z hacia arriba, una rotación a la izq es Z positivo.
            gyro_data = sensor_imu.gyro
            omega_real = gyro_data[2] if (gyro_data and gyro_data[2] is not None) else 0.0
        except:
            omega_real = 0.0

        # Calcular el error de giro
        error_omega = omega_ref - omega_real
        ajuste_pid = error_omega * KP_OMEGA

        # Cinemática Diferencial (Salida en m/s)
        # V_R = V + (W*L)/2 y V_L = V - (W*L)/2
        v_izq_ms = v_ref - ((omega_ref * L_WHEELBASE_M) / 2.0) - ajuste_pid
        v_der_ms = v_ref + ((omega_ref * L_WHEELBASE_M) / 2.0) + ajuste_pid

        # Convertir m/s a RPM para enviar a los ESP32
        rpm_izq = (v_izq_ms * 60.0) / CIRCUNFERENCIA_M
        rpm_der = (v_der_ms * 60.0) / CIRCUNFERENCIA_M

        aplicar_velocidades_recto(rpm_izq, rpm_der)
        time.sleep(periodo)

# Arrancar threads
threading.Thread(target=hilo_escuchar_vision, daemon=True).start()
threading.Thread(target=hilo_pid_imu,         daemon=True).start()

# ==========================================
# 6. MAIN THREAD — LÓGICA TÁCTICA
# ==========================================
print("\n" + "="*40 + "\n  ESPERANDO TRAYECTORIAS PURE PURSUIT\n" + "="*40)

estado_mision_anterior = 0

try:
    while True:
        nuevo_dato_event.wait(timeout=0.1)
        nuevo_dato_event.clear()

        with vision_lock:
            dist_meta_local     = datos_vision.get("dist", 1.0)
            estado_mision_local = datos_vision.get("estado_mision", 0)
            activo_local        = datos_vision.get("activo", False)
            ultima_vez_local    = datos_vision.get("ultima_vez", 0.0)
            v_lineal_local      = datos_vision.get("v_lineal", 0.0)
            omega_ref_local     = datos_vision.get("omega_ref", 0.0)

        # Timeout de seguridad
        if not activo_local or (time.time() - ultima_vez_local > 1.0):
            pид_activo.clear()
            frenar_y_limpiar()
            sys.stdout.write("\r[ESPERA] Sin datos de visión...        ")
            sys.stdout.flush()
            continue

        pид_activo.set()

        # Detección de paradas logísticas (Almacén)
        if estado_mision_local != estado_mision_anterior:
            if estado_mision_local in (2, 4):
                pид_activo.clear()
                frenar_y_limpiar()
                print(f"\n[■] OPERACIÓN EN ESTACIÓN (Estado {estado_mision_local}) — esperando 10 s...")
                time.sleep(10)
                print("[►] Continuando ruta.")
                pид_activo.set()
            estado_mision_anterior = estado_mision_local

        sys.stdout.write(f"\r[NAV] Estado: {estado_mision_local} | Dist: {dist_meta_local:.2f}m | V: {v_lineal_local:.2f} | W: {omega_ref_local:+.2f}   ")
        sys.stdout.flush()

except KeyboardInterrupt:
    print("\n\n[!] Detenido por el usuario.")
finally:
    pид_activo.clear()
    frenar_y_limpiar()
    if 'esp_1' in locals(): esp_1.close()
    if 'esp_2' in locals(): esp_2.close()
    print("Sistemas apagados.")
