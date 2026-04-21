import serial, time, math, sys, socket, threading, board, busio, adafruit_bno055

# ==========================================
# 1. CONFIGURACIÓN Y PARÁMETROS
# ==========================================
KP_RUMBO = 0.7    # Corrección IMU (Giro)
KP_VISION = 30.0  # Corrección de desviación lateral (Línea virtual)
RPM_BASE = 25.0   
PUERTO_UDP = 5005

tags_vistos = {}
tecla_emergencia = ""

print("Iniciando Sistemas a Bordo...")
i2c = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)

try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1) 
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1) 
    print("[OK] Motores Listos.")
except Exception as e:
    print(f"Error USB: {e}"); sys.exit()

# ==========================================
# 2. HILOS DE BACKGROUND (WIFI Y TECLADO)
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
            global tags_vistos
            tags_vistos = tags_temp
        except: pass

def vigilante_teclado():
    global tecla_emergencia
    while True:
        # Esto atrapa lo que escribas en cualquier momento
        tecla_emergencia = input().strip().lower()

threading.Thread(target=escuchar_wifi, daemon=True).start()
threading.Thread(target=vigilante_teclado, daemon=True).start()

# ==========================================
# 3. CONTROL DE MOVIMIENTO
# ==========================================
def mandar_orden(placa, motor, direccion, rpm):
    placa.write(f"{motor},{direccion},{round(rpm, 1)}\n".encode('utf-8'))

def frenar_y_limpiar():
    mandar_orden(esp_1, "A", 0, 0); mandar_orden(esp_1, "B", 0, 0)
    mandar_orden(esp_2, "A", 0, 0); mandar_orden(esp_2, "B", 0, 0)
    time.sleep(0.1) 
    esp_1.reset_input_buffer(); esp_2.reset_input_buffer()

def calcular_error_lateral(start_x, start_y, target_x, target_y, robot_x, robot_y):
    """ Calcula la desviación del robot respecto a la línea ideal de avance """
    numerador = (target_y - start_y)*(robot_x - start_x) - (target_x - start_x)*(robot_y - start_y)
    denominador = math.hypot(target_x - start_x, target_y - start_y)
    if denominador == 0: return 0
    return numerador / denominador

def ir_a_estacion(id_destino):
    global tecla_emergencia
    tecla_emergencia = "" # Limpiamos eventos pasados
    
    print(f"\n======================================")
    print(f" OBJETIVO DETECTADO: ESTACIÓN {id_destino}")
    print(f"======================================")
    print(">> Presiona 'y' + Enter para arrancar")
    print(">> Presiona 's' + Enter en CUALQUIER MOMENTO para PARO DE EMERGENCIA")
    
    # Esperar confirmación
    while tecla_emergencia != 'y':
        if tecla_emergencia == 's':
            print("\n[!!!] MISIÓN ABORTADA [!!!]"); frenar_y_limpiar(); sys.exit()
        time.sleep(0.1)
        
    tecla_emergencia = "" # Limpiar después del 'y'
    
    # Guardar las coordenadas de arranque para crear la Línea Virtual
    if 0 not in tags_vistos or id_destino not in tags_vistos:
        print("[!] Error: No veo la cámara. Abortando.")
        return

    start_x = tags_vistos[0]["x"]
    start_y = tags_vistos[0]["y"]
    
    # Obtener yaw inicial del IMU para mantenerlo derecho
    try: yaw_objetivo = sensor_imu.euler[0]
    except: yaw_objetivo = 0

    print(f"\n[>>>] Avanzando hacia Tag {id_destino}...")

    # Bucle de navegación
    while True:
        # 1. PARO DE EMERGENCIA INMEDIATO
        if tecla_emergencia == 's':
            print("\n\n[!!!] PARO DE EMERGENCIA ACTIVADO POR OPERADOR [!!!]")
            frenar_y_limpiar()
            sys.exit()

        # 2. Control de Trayectoria
        if 0 in tags_vistos and id_destino in tags_vistos:
            rx, ry = tags_vistos[0]["x"], tags_vistos[0]["y"]
            tx, ty = tags_vistos[id_destino]["x"], tags_vistos[id_destino]["y"]
            
            # Distancia a la meta
            distancia = math.hypot(tx - rx, ty - ry)
            
            if distancia < 0.10: # ¡LLEGAMOS!
                frenar_y_limpiar()
                print(f"\n[★★★] LLEGADA CONFIRMADA A ESTACIÓN {id_destino} [★★★]")
                break

            # Cálculo de errores
            error_lateral = calcular_error_lateral(start_x, start_y, tx, ty, rx, ry)
            
            try: yaw_actual = sensor_imu.euler[0]
            except: yaw_actual = yaw_objetivo
            
            error_yaw = yaw_actual - yaw_objetivo
            if error_yaw > 180: error_yaw -= 360
            elif error_yaw < -180: error_yaw += 360

            # Fusión de Control (Visión + IMU)
            ajuste_total = (error_yaw * KP_RUMBO) + (error_lateral * KP_VISION)
            
            rpm_izq = max(10, min(RPM_BASE - ajuste_total, 100))
            rpm_der = max(10, min(RPM_BASE + ajuste_total, 100))

            mandar_orden(esp_1, "A", 1, rpm_izq); mandar_orden(esp_1, "B", 2, rpm_izq) 
            mandar_orden(esp_2, "A", 1, rpm_der); mandar_orden(esp_2, "B", 2, rpm_der) 

            sys.stdout.write(f"\r   Distancia: {distancia:.2f}m | Error Lateral: {error_lateral:.3f}m    ")
            sys.stdout.flush()

        time.sleep(0.05)

# ==========================================
# 4. EJECUCIÓN DE LA MISIÓN RECTÁNGULO
# ==========================================
try:
    print("\nEsperando enlace con la cámara...")
    while 0 not in tags_vistos:
        time.sleep(0.5)

    # La ruta de tu almacén: Va al 1, luego al 2, etc.
    ir_a_estacion(1)
    # Aquí puedes llamar a tu función girar_grados(90) si el carrito necesita voltearse
    
    ir_a_estacion(2)
    
    print("\n[✔] RUTA DEL ALMACÉN COMPLETADA CON ÉXITO.")

except KeyboardInterrupt:
    frenar_y_limpiar()
finally:
    frenar_y_limpiar()
    esp_1.close(); esp_2.close()
