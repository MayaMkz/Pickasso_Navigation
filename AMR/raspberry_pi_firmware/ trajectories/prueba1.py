import serial, time, math, sys, socket, threading, board, busio, adafruit_bno055

# ==========================================
# 1. PARÁMETROS
# ==========================================
KP_RUMBO = 0.5    # Qué tan rápido corrige su ángulo
KP_VISION = 25.0  # Qué tan fuerte se aferra a la línea virtual de la cámara
RPM_BASE = 35.0   
PUERTO_UDP = 5005

tags_vistos = {}
tecla_usuario = ""

print("Iniciando Médula Espinal y Sensores...")
i2c = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)

try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1) 
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1) 
    print("[OK] ESP32 Conectadas.")
except Exception as e:
    print(f"Error USB: {e}"); sys.exit()

# ==========================================
# 2. PROCESOS EN SEGUNDO PLANO
# ==========================================
def escuchar_wifi():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PUERTO_UDP))
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            paquetes = data.decode('utf-8').split('|')
            tags_temp = {}
            for p in paquetes:
                id_tag, coords = p.split(':')
                x, y, z = map(float, coords.split(','))
                tags_temp[int(id_tag)] = {"x": x, "y": y}
            global tags_vistos; tags_vistos = tags_temp
        except: pass

def vigilante_teclado():
    global tecla_usuario
    while True: 
        tecla_usuario = input().strip().lower()

threading.Thread(target=escuchar_wifi, daemon=True).start()
threading.Thread(target=vigilante_teclado, daemon=True).start()

# ==========================================
# 3. CONTROL DE HARDWARE
# ==========================================
def mandar_orden(placa, motor, direccion, rpm):
    placa.write(f"{motor},{direccion},{round(rpm, 1)}\n".encode('utf-8'))

def frenar_y_limpiar():
    mandar_orden(esp_1, "A", 0, 0); mandar_orden(esp_1, "B", 0, 0)
    mandar_orden(esp_2, "A", 0, 0); mandar_orden(esp_2, "B", 0, 0)
    time.sleep(0.1) 
    esp_1.reset_input_buffer(); esp_2.reset_input_buffer()

def girar_grados(grados, rpm_giro=25):
    print(f"\n[Rotación] Girando {grados} grados...")
    try: yaw_ini = sensor_imu.euler[0]
    except: yaw_ini = 0
    
    if grados > 0: 
        mandar_orden(esp_1, "A", 1, rpm_giro); mandar_orden(esp_1, "B", 2, rpm_giro) 
        mandar_orden(esp_2, "A", 2, rpm_giro); mandar_orden(esp_2, "B", 1, rpm_giro) 
    else: 
        mandar_orden(esp_1, "A", 2, rpm_giro); mandar_orden(esp_1, "B", 1, rpm_giro) 
        mandar_orden(esp_2, "A", 1, rpm_giro); mandar_orden(esp_2, "B", 2, rpm_giro) 
        
    while True:
        global tecla_usuario
        if tecla_usuario == 's':
            frenar_y_limpiar(); print("\n[!!!] EMERGENCIA [!!!]"); sys.exit()
            
        try: yaw_act = sensor_imu.euler[0]
        except: yaw_act = yaw_ini
        
        giro = yaw_act - yaw_ini
        if giro > 180: giro -= 360
        elif giro < -180: giro += 360
        
        sys.stdout.write(f"\r   IMU: {abs(giro):.1f}° / {abs(grados)}° ")
        if abs(giro) >= (abs(grados) - 1.5):
            frenar_y_limpiar(); break
        time.sleep(0.01)

def calcular_error_lateral(sx, sy, tx, ty, rx, ry):
    num = (ty - sy)*(rx - sx) - (tx - sx)*(ry - sy)
    den = math.hypot(tx - sx, ty - sy)
    return num / den if den != 0 else 0

def ir_a_estacion(id_destino):
    global tecla_usuario
    tecla_usuario = "" 
    
    print(f"\n\n=== RUTA HACIA ARUCO {id_destino} ===")
    print(">> 'y' = Arrancar | 's' = Emergencia")
    
    while tecla_usuario != 'y':
        if tecla_usuario == 's':
            frenar_y_limpiar(); print("\n[!!!] ABORTADO [!!!]"); sys.exit()
        time.sleep(0.1)
        
    tecla_usuario = "" # Reseteamos para estar listos para la 's'
    
    if 0 not in tags_vistos or id_destino not in tags_vistos:
        print("[!] No veo los tags en la cámara. Misión cancelada.")
        return

    start_x, start_y = tags_vistos[0]["x"], tags_vistos[0]["y"]
    try: yaw_objetivo = sensor_imu.euler[0]
    except: yaw_objetivo = 0

    while True:
        if tecla_usuario == 's':
            frenar_y_limpiar(); print("\n\n[!!!] PARO DE EMERGENCIA [!!!]"); sys.exit()

        if 0 in tags_vistos and id_destino in tags_vistos:
            rx, ry = tags_vistos[0]["x"], tags_vistos[0]["y"]
            tx, ty = tags_vistos[id_destino]["x"], tags_vistos[id_destino]["y"]
            
            # 1. ¿Ya llegamos a la coordenada?
            distancia = math.hypot(tx - rx, ty - ry)
            if distancia < 0.10:
                frenar_y_limpiar()
                print(f"\n[✔] Estación {id_destino} Alcanzada.")
                break

            # 2. ¿Nos salimos de la línea?
            error_lateral = calcular_error_lateral(start_x, start_y, tx, ty, rx, ry)
            
            # 3. ¿El carrito se rotó? (IMU)
            try: yaw_actual = sensor_imu.euler[0]
            except: yaw_actual = yaw_objetivo
            
            error_yaw = yaw_actual - yaw_objetivo
            if error_yaw > 180: error_yaw -= 360
            elif error_yaw < -180: error_yaw += 360

            # 4. Inyectar correcciones a los Encoders (Vía ESP32)
            ajuste_total = (error_yaw * KP_RUMBO) + (error_lateral * KP_VISION)
            rpm_izq = max(10, min(RPM_BASE - ajuste_total, 100))
            rpm_der = max(10, min(RPM_BASE + ajuste_total, 100))

            mandar_orden(esp_1, "A", 1, rpm_izq); mandar_orden(esp_1, "B", 2, rpm_izq) 
            mandar_orden(esp_2, "A", 1, rpm_der); mandar_orden(esp_2, "B", 2, rpm_der) 

            # Impresión de telemetría limpia
            sys.stdout.write(f"\r[HUD] Meta: {distancia:.2f}m | Desvío: {error_lateral*100:.1f}cm | RPM: {rpm_izq:.0f}/{rpm_der:.0f}   ")
            sys.stdout.flush()
        time.sleep(0.05)

# ==========================================
# 4. CEREBRO PRINCIPAL
# ==========================================
try:
    print("\nEsperando enlace con cámara...")
    while 0 not in tags_vistos: time.sleep(0.5)

    # SECUENCIA DEL RECTÁNGULO
    ir_a_estacion(1)
    
    girar_grados(90)
    ir_a_estacion(2) # Usando la esquina virtual
    
    girar_grados(90)
    ir_a_estacion(3)
    
    girar_grados(90)
    ir_a_estacion(4) # Usando la esquina virtual
    
    print("\n[★★★] OPERACIÓN EXITOSA [★★★]")

except KeyboardInterrupt:
    frenar_y_limpiar()
finally:
    frenar_y_limpiar(); esp_1.close(); esp_2.close()
