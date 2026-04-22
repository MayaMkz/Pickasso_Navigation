import serial, time, math, sys, socket, threading, board, busio, adafruit_bno055

# ==========================================
# 1. PARÁMETROS DE NAVEGACIÓN
# ==========================================
KP_RUMBO = 0.5    # Corrección IMU (Giro)
KP_VISION = 25.0  # Corrección de desviación lateral
RPM_BASE = 35.0   
PUERTO_UDP = 5005

tags_vistos = {}
tecla_usuario = ""

print("\n" + "="*50)
print("[*] INICIANDO SISTEMA DE NAVEGACIÓN PICKASSO")
print("="*50)

# Inicialización de Sensores I2C
i2c = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)

# Inicialización de ESP32 (Motores)
try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1) 
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1) 
    print("[OK] Enlace con Médula Espinal (ESP32) establecido.")
except Exception as e:
    sys.exit(f"[!] Error crítico USB: {e}")

# ==========================================
# 2. HILOS DE BACKGROUND (ROUTER Y TECLADO)
# ==========================================
def escuchar_wifi():
    """ Escucha el UDP a altísima velocidad y actualiza la memoria compartida """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PUERTO_UDP))
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            paquetes = data.decode('utf-8').split('|')
            tags_temp = {}
            for p in paquetes:
                if not p: continue
                partes = p.split(':')
                if len(partes) == 2:
                    id_tag = int(partes[0])
                    coords = partes[1].split(',')
                    tags_temp[id_tag] = {"x": float(coords[0]), "y": float(coords[1])}
            
            global tags_vistos
            tags_vistos = tags_temp
        except: pass

def vigilante_teclado():
    """ Hilo de máxima prioridad para el Paro de Emergencia """
    global tecla_usuario
    while True: 
        tecla_usuario = input().strip().lower()

threading.Thread(target=escuchar_wifi, daemon=True).start()
threading.Thread(target=vigilante_teclado, daemon=True).start()

# ==========================================
# 3. CONTROL CINEMÁTICO
# ==========================================
def mandar_orden(placa, motor, direccion, rpm):
    placa.write(f"{motor},{direccion},{round(rpm, 1)}\n".encode('utf-8'))

def frenar_y_limpiar():
    mandar_orden(esp_1, "A", 0, 0); mandar_orden(esp_1, "B", 0, 0)
    mandar_orden(esp_2, "A", 0, 0); mandar_orden(esp_2, "B", 0, 0)
    time.sleep(0.1) 
    esp_1.reset_input_buffer(); esp_2.reset_input_buffer()

def girar_90_grados():
    print("\n[Rotación] Ejecutando giro de 90 grados...")
    # Leer IMU inicial (con protección si falla la lectura)
    try: yaw_ini = sensor_imu.euler[0]
    except: yaw_ini = 0
    if yaw_ini is None: yaw_ini = 0
    
    # Mandar comando de giro sobre su propio eje (Derecha)
    mandar_orden(esp_1, "A", 1, 30); mandar_orden(esp_1, "B", 2, 30) 
    mandar_orden(esp_2, "A", 2, 30); mandar_orden(esp_2, "B", 1, 30) 
    
    while True:
        global tecla_usuario
        if tecla_usuario == 's':
            frenar_y_limpiar()
            sys.exit("\n[!!!] PARO DE EMERGENCIA ACTIVADO [!!!]")
            
        try: yaw_act = sensor_imu.euler[0]
        except: yaw_act = yaw_ini
        if yaw_act is None: yaw_act = yaw_ini
        
        giro = yaw_act - yaw_ini
        if giro > 180: giro -= 360
        elif giro < -180: giro += 360
        
        sys.stdout.write(f"\r   [IMU] Angulo: {abs(giro):.1f}° / 90.0°   ")
        sys.stdout.flush()
        
        # Frenar 1.5 grados antes para compensar la inercia de las llantas
        if abs(giro) >= 88.5:
            frenar_y_limpiar()
            break
        time.sleep(0.01)

def calcular_error_lateral(sx, sy, tx, ty, rx, ry):
    """ Distancia perpendicular del robot a la línea recta ideal """
    num = (ty - sy)*(rx - sx) - (tx - sx)*(ry - sy)
    den = math.hypot(tx - sx, ty - sy)
    return num / den if den != 0 else 0

def ir_a_estacion(id_destino):
    global tecla_usuario
    tecla_usuario = "" 
    
    print(f"\n\n=== NAVEGANDO HACIA WAYPOINT {id_destino} ===")
    print(">> INSTRUCCIÓN: Presiona 'y' para arrancar | 's' para Paro Total")
    
    # Espera segura de confirmación
    while tecla_usuario != 'y':
        if tecla_usuario == 's':
            frenar_y_limpiar()
            sys.exit("\n[!!!] MISIÓN CANCELADA [!!!]")
        time.sleep(0.1)
        
    tecla_usuario = "" # Armar el botón de emergencia
    
    if 0 not in tags_vistos or id_destino not in tags_vistos:
        print(f"[!] Error: La cámara no ve el Tag 0 o el destino {id_destino}. Abortando tramo.")
        return

    # Congelar punto de partida para trazar la línea virtual
    start_x, start_y = tags_vistos[0]["x"], tags_vistos[0]["y"]
    
    # Congelar el rumbo para no irse de lado
    try: yaw_objetivo = sensor_imu.euler[0]
    except: yaw_objetivo = 0
    if yaw_objetivo is None: yaw_objetivo = 0

    while True:
        # 1. Monitoreo de Paro
        if tecla_usuario == 's':
            frenar_y_limpiar()
            sys.exit("\n\n[!!!] PARO DE EMERGENCIA ACTIVADO [!!!]")

        # 2. Control Principal
        if 0 in tags_vistos and id_destino in tags_vistos:
            rx, ry = tags_vistos[0]["x"], tags_vistos[0]["y"]
            tx, ty = tags_vistos[id_destino]["x"], tags_vistos[id_destino]["y"]
            
            # ¿Ya llegó al objetivo? (Umbral de 10 cm)
            distancia = math.hypot(tx - rx, ty - ry)
            if distancia < 0.10:
                frenar_y_limpiar()
                print(f"\n[✔] WAYPOINT {id_destino} ALCANZADO.")
                break

            # Error Lateral (Visión)
            error_lateral = calcular_error_lateral(start_x, start_y, tx, ty, rx, ry)
            
            # Error de Rumbo (IMU)
            try: yaw_actual = sensor_imu.euler[0]
            except: yaw_actual = yaw_objetivo
            if yaw_actual is None: yaw_actual = yaw_objetivo
            
            error_yaw = yaw_actual - yaw_objetivo
            if error_yaw > 180: error_yaw -= 360
            elif error_yaw < -180: error_yaw += 360

            # Fusión Matemática y Límite de RPM
            ajuste_total = (error_yaw * KP_RUMBO) + (error_lateral * KP_VISION)
            rpm_izq = max(10, min(RPM_BASE - ajuste_total, 100))
            rpm_der = max(10, min(RPM_BASE + ajuste_total, 100))

            mandar_orden(esp_1, "A", 1, rpm_izq); mandar_orden(esp_1, "B", 2, rpm_izq) 
            mandar_orden(esp_2, "A", 1, rpm_der); mandar_orden(esp_2, "B", 2, rpm_der) 

            # HUD de Telemetría (Se imprime limpiamente en la misma línea)
            sys.stdout.write(f"\r[TELEMETRÍA] Dist: {distancia:.2f}m | Desvío: {error_lateral*100:.1f}cm | RPM: {rpm_izq:.0f}/{rpm_der:.0f}   ")
            sys.stdout.flush()
            
        time.sleep(0.05)

# ==========================================
# 4. RUTINA LOGÍSTICA DEL ALMACÉN
# ==========================================
try:
    print("\nEsperando enlace de visión con la computadora central...")
    while 0 not in tags_vistos or 1 not in tags_vistos or 2 not in tags_vistos:
        # Espera hasta ver el robot (0) y las dos esquinas físicas (1 y 2)
        time.sleep(0.5)
    print("[OK] Coordenadas recibidas. Pista mapeada.")

    # TRAMO 1: Robot -> Esquina 1 FÍSICA
    ir_a_estacion(1)
    
    # TRAMO 2: Giro -> Esquina 3 VIRTUAL
    girar_90_grados()
    ir_a_estacion(3) 
    
    # TRAMO 3: Giro -> Esquina 2 FÍSICA (La opuesta inicial)
    girar_90_grados()
    ir_a_estacion(2)
    
    # TRAMO 4: Giro -> Esquina 4 VIRTUAL (Cierra el rectángulo)
    girar_90_grados()
    ir_a_estacion(4) 
    
    print("\n\n[★★★] RUTA DE ALMACÉN COMPLETADA EXITOSAMENTE [★★★]")

except KeyboardInterrupt:
    print("\n[!] Interrupción por teclado detectada.")
finally:
    frenar_y_limpiar()
    esp_1.close()
    esp_2.close()
    print("[OK] Motores desenergizados y puertos cerrados.")
