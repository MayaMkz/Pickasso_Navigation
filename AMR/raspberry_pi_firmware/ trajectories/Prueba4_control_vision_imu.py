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

# Ganancias de Control PID
KP_RUMBO = 0.3       
RPM_BASE = 20.0  
KP_VISION_METROS = 500

INVERTIR_CORRECCION_VISION = True 
INVERTIR_PID_IMU = False 

PUERTO_UDP = 5005

# ==========================================
# VARIABLES COMPARTIDAS, LOCKS Y EVENTS
# ==========================================
datos_vision = {"dx": 0.0, "dy": 0.0, "dist": 0.0, "sentido": 1.0, "error_angular": 0.0, "activo": False, "ultima_vez": time.time()}

vision_lock = threading.Lock()
nuevo_dato_event = threading.Event()

# ==========================================
# 2. CLASE FILTRO IMU (Anti-Brincos)
# ==========================================
class FiltroIMU_AntiBrincos:
    def __init__(self, umbral_salto_grados=10.0, max_rechazos=10):
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
# 3. HILO DE VISIÓN (PRODUCER)
# ==========================================
def hilo_escuchar_vision():
    global datos_vision
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PUERTO_UDP))
    sock.setblocking(False) 
    
    while True:
        try:
            while True:
                data, _ = sock.recvfrom(1024)
                partes = data.decode('utf-8').split(',')
                
                if len(partes) >= 4:
                    with vision_lock:
                        datos_vision["dx"] = float(partes[0])
                        datos_vision["dy"] = float(partes[1])
                        datos_vision["dist"] = float(partes[2])
                        datos_vision["sentido"] = float(partes[3])
                        if len(partes) >= 5:
                            datos_vision["error_angular"] = float(partes[4])
                        datos_vision["activo"] = True
                        datos_vision["ultima_vez"] = time.time()
                    
                    nuevo_dato_event.set()
                    
        except BlockingIOError:
            time.sleep(0.01)
        except Exception:
            pass

threading.Thread(target=hilo_escuchar_vision, daemon=True).start()

# ==========================================
# 4. INICIALIZACIÓN DE HARDWARE
# ==========================================
print("Iniciando El Cerebro (Threading con Locks & Events)...")
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
        if yaw_crudo is not None: return filtro_yaw.filtrar(yaw_crudo)
    except: pass
    return filtro_yaw.yaw_anterior if filtro_yaw.yaw_anterior else 0.0

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

def aplicar_velocidades_recto(v_izq, v_der):
    v_izq = max(-100, min(100, v_izq))
    v_der = max(-100, min(100, v_der))
    dir_1A, dir_1B = (1, 2) if v_izq >= 0 else (2, 1)
    dir_2A, dir_2B = (1, 2) if v_der >= 0 else (2, 1)
    mandar_orden(esp_1, "A", dir_1A, abs(v_izq)); mandar_orden(esp_1, "B", dir_1B, abs(v_izq))
    mandar_orden(esp_2, "A", dir_2A, abs(v_der)); mandar_orden(esp_2, "B", dir_2B, abs(v_der))

# ==========================================
# 5. DIAGNÓSTICO DE ARRANQUE
# ==========================================
def diagnostico_inicial_motores():
    print("\n" + "="*40)
    print(" EJECUTANDO DIAGNÓSTICO DE HARDWARE...")
    print("="*40)
    while True:
        frenar_y_limpiar()
        time.sleep(0.2)
        e1A_ini, e1B_ini = leer_encoders_seguro(esp_1)
        e2A_ini, e2B_ini = leer_encoders_seguro(esp_2)
        print("[!] Aplicando pulso de prueba (15 RPM)...")
        aplicar_velocidades_recto(15, 15)
        time.sleep(0.5) 
        frenar_y_limpiar()
        time.sleep(0.2)
        e1A_fin, e1B_fin = leer_encoders_seguro(esp_1)
        e2A_fin, e2B_fin = leer_encoders_seguro(esp_2)
        dif_1A = abs(e1A_fin - e1A_ini); dif_1B = abs(e1B_fin - e1B_ini)
        dif_2A = abs(e2A_fin - e2A_ini); dif_2B = abs(e2B_fin - e2B_ini)
        umbral_fallo = 50 
        if dif_1A < umbral_fallo or dif_1B < umbral_fallo or dif_2A < umbral_fallo or dif_2B < umbral_fallo:
            print("\n[ERROR CRÍTICO] Se detectó falla en motores o encoders:")
            input("\n>>> Presiona ENTER cuando revises las conexiones...")
        else:
            print("[OK] Los 4 motores y encoders responden correctamente.")
            break
diagnostico_inicial_motores()

# ==========================================
# 6. RUTINAS DE GIRO
# ==========================================
def girar_grados(grados, rpm_giro=20):
    print(f"\n-> Girando {grados:.1f} grados...")
    yaw_ini = obtener_yaw()
    if grados > 0: 
        mandar_orden(esp_1, "A", 1, rpm_giro); mandar_orden(esp_1, "B", 2, rpm_giro) 
        mandar_orden(esp_2, "A", 2, rpm_giro); mandar_orden(esp_2, "B", 1, rpm_giro) 
    else: 
        mandar_orden(esp_1, "A", 2, rpm_giro); mandar_orden(esp_1, "B", 1, rpm_giro) 
        mandar_orden(esp_2, "A", 1, rpm_giro); mandar_orden(esp_2, "B", 2, rpm_giro) 
    while True:
        yaw_act = obtener_yaw()
        giro = yaw_act - yaw_ini
        if giro > 180: giro -= 360
        elif giro < -180: giro += 360
        if abs(giro) >= (abs(grados) - 1.5):
            frenar_y_limpiar()
            print("\n[!] Giro completado.")
            time.sleep(0.5)
            break
        time.sleep(0.01)

# ==========================================
# 7. MÁQUINA DE ESTADOS PRINCIPAL (CONSUMER)
# ==========================================
print("\n" + "="*40 + "\n  ESPERANDO ÓRDENES DE VISIÓN (WIFI)\n" + "="*40)

angulo_vision_anterior = None
rumbo_vision_ideal = None 
yaw_recta_base = None
yaw_objetivo_local = None
contador_esquinas = 0 

try:
    while True:
        
        with vision_lock:
            dx_local = datos_vision["dx"]
            dy_local = datos_vision["dy"]
            dist_meta_local = datos_vision["dist"]
            sentido_local = datos_vision["sentido"]
            error_angular_local = datos_vision["error_angular"]
            activo_local = datos_vision["activo"]
            ultima_vez_local = datos_vision["ultima_vez"]
        
        if nuevo_dato_event.is_set():
            nuevo_dato_event.clear()
            
        if not activo_local or (time.time() - ultima_vez_local > 1.0):
            frenar_y_limpiar()
            angulo_vision_anterior = None
            rumbo_vision_ideal = None
            yaw_recta_base = None
            contador_esquinas = 0 
            sys.stdout.write("\r[ESPERA] Sin datos de visión...        ")
            sys.stdout.flush()
            time.sleep(0.1)
            continue

        angulo_vision_actual = math.degrees(math.atan2(dy_local, dx_local))

        if angulo_vision_anterior is None:
            angulo_vision_anterior = angulo_vision_actual
            rumbo_vision_ideal = angulo_vision_actual 
            yaw_recta_base = obtener_yaw() 
            yaw_objetivo_local = yaw_recta_base
            print("\n[>>>] INICIANDO RUTEO RECTO [>>>]")

        dif_vision = angulo_vision_actual - angulo_vision_anterior
        dif_vision = (dif_vision + 180) % 360 - 180

        # --- LÓGICA DE ESQUINAS CORREGIDA ---
        # Como el rectángulo de la PC ya tiene el offset de 35cm incluido,
        # el carrito simplemente debe girar 90 grados sobre su propio eje sin moverse de lugar.
        if abs(dif_vision) > 45 and dist_meta_local > 0.20: 
            frenar_y_limpiar()
            contador_esquinas += 1 
            print(f"\n[ESTACIÓN ALCANZADA] Esquina {contador_esquinas} procesada.")
            time.sleep(0.2) 
            
            if contador_esquinas == 1 or contador_esquinas == 3:
                print("\n[■] OPERACIÓN EN ESTACIÓN: Esperando 10 segundos...")
                time.sleep(10)
                print("\n[►] Operación terminada. Girando para continuar.")
            else:
                time.sleep(0.3) 
            
            # Giro puro en el mismo lugar
            girar_grados(90.0 * sentido_local, rpm_giro=20)
            
            # Reset de ángulos para que empiece a seguir la nueva línea recta
            angulo_vision_anterior = angulo_vision_actual
            rumbo_vision_ideal = angulo_vision_actual 
            yaw_recta_base = obtener_yaw() 
            yaw_objetivo_local = yaw_recta_base
            continue
            
        elif abs(dif_vision) > 90 and dist_meta_local <= 0.15:
            frenar_y_limpiar()
            sys.stdout.write("\r[ESPERA] Overshoot detectado. Esperando actualización PC...")
            sys.stdout.flush()
            continue

        # --- FUSIÓN SENSORIAL ---
        if error_angular_local != 0.0:
            correccion_vision = error_angular_local
        else:
            dif_angulo_vision = angulo_vision_actual - rumbo_vision_ideal
            dif_angulo_vision = (dif_angulo_vision + 180) % 360 - 180
            error_lateral_metros = dist_meta_local * math.sin(math.radians(dif_angulo_vision))
            correccion_vision = error_lateral_metros * KP_VISION_METROS
            
        correccion_vision = max(-140.0, min(140.0, correccion_vision))
        
        if INVERTIR_CORRECCION_VISION:
            yaw_objetivo_local = yaw_recta_base - correccion_vision
        else:
            yaw_objetivo_local = yaw_recta_base + correccion_vision

        # --- LECTURA IMU Y PID MOTORES ---
        yaw_actual = obtener_yaw()
        error_yaw = yaw_actual - yaw_objetivo_local
        error_yaw = (error_yaw + 180) % 360 - 180
        
        ajuste = error_yaw * KP_RUMBO
        if INVERTIR_PID_IMU: ajuste = -ajuste
        
        rpm_base = RPM_BASE if dist_meta_local > 0.20 else RPM_BASE * 0.6
        rpm_izq = rpm_base - ajuste
        rpm_der = rpm_base + ajuste

        aplicar_velocidades_recto(rpm_izq, rpm_der)

        sys.stdout.write(f"\r[NAVEGANDO] Meta: {dist_meta_local:.2f}m | Volantazo: {correccion_vision:+.1f}°   ")
        sys.stdout.flush()
        
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\n\n[!] Detenido por el usuario.")
finally:
    frenar_y_limpiar()
    if 'esp_1' in locals(): esp_1.close()
    if 'esp_2' in locals(): esp_2.close()
    print("Sistemas apagados.")
