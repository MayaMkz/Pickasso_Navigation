import serial, time, math, sys, socket, threading, board, busio, adafruit_bno055

# ==========================================
# PARÁMETROS DE NAVEGACIÓN VECTORIAL
# ==========================================
KP_ANGULO = 0.8    # Qué tan agresivo corrige la dirección
KP_DIST = 40.0     # Qué tan rápido acelera basado en la distancia
RPM_MAX = 45.0     # Límite de velocidad
PUERTO_UDP = 5005

tags_vistos = {}
tecla_usuario = ""

i2c = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)

try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1) 
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1) 
except: sys.exit("[!] Error USB ESP32")

def escuchar_wifi():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PUERTO_UDP))
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            tags_temp = {}
            for p in data.decode('utf-8').split('|'):
                if ':' in p:
                    id_tag, coords = p.split(':')
                    x, y = map(float, coords.split(','))
                    tags_temp[int(id_tag)] = {"x": x, "y": y}
            global tags_vistos; tags_vistos = tags_temp
        except: pass

def vigilante_teclado():
    global tecla_usuario
    while True: tecla_usuario = input().strip().lower()

threading.Thread(target=escuchar_wifi, daemon=True).start()
threading.Thread(target=vigilante_teclado, daemon=True).start()

def mandar_orden(placa, motor, dir_num, rpm):
    # Asegurar que RPM no sea negativo y mandarlo
    rpm_limpio = max(0, min(abs(rpm), 100))
    placa.write(f"{motor},{dir_num},{round(rpm_limpio, 1)}\n".encode('utf-8'))

def frenar():
    mandar_orden(esp_1, "A", 0, 0); mandar_orden(esp_1, "B", 0, 0)
    mandar_orden(esp_2, "A", 0, 0); mandar_orden(esp_2, "B", 0, 0)

def ir_a_objetivo(id_meta):
    global tecla_usuario
    tecla_usuario = ""
    print(f"\n>> Esperando inicio hacia TAG {id_meta} ('y'=Arrancar, 's'=Paro)")
    
    while tecla_usuario != 'y':
        if tecla_usuario == 's': frenar(); sys.exit("\n[!] ABORTADO")
        time.sleep(0.1)
    tecla_usuario = ""

    while True:
        if tecla_usuario == 's': frenar(); sys.exit("\n[!!!] PARO DE EMERGENCIA")

        if 0 in tags_vistos and id_meta in tags_vistos:
            # 1. Calcular Diferencias (Vectores)
            dx = tags_vistos[id_meta]["x"] - tags_vistos[0]["x"]
            dy = tags_vistos[id_meta]["y"] - tags_vistos[0]["y"]
            dist = math.hypot(dx, dy)

            # ¿Llegamos?
            if dist < 0.10:
                frenar()
                print(f"\n[✔] Objetivo {id_meta} alcanzado.")
                break

            # 2. Calcular Ángulos
            # atan2 nos da el ángulo absoluto hacia la meta (en grados)
            angulo_meta = math.degrees(math.atan2(dy, dx))
            
            try: yaw_actual = sensor_imu.euler[0]
            except: yaw_actual = 0
            if yaw_actual is None: yaw_actual = 0

            # Error de orientación
            error_angulo = angulo_meta - yaw_actual
            # Normalizar entre -180 y 180
            error_angulo = (error_angulo + 180) % 360 - 180

            # 3. Lógica de Dirección (Adelante o Reversa)
            sentido_marcha = 1 # 1 = Adelante, 2 = Reversa
            
            if abs(error_angulo) > 90:
                # El objetivo está detrás del robot. Marcha atrás.
                sentido_marcha = 2
                # Ajustamos el error de ángulo como si la parte trasera fuera el frente
                error_angulo = (error_angulo + 180) % 360 - 180 

            # 4. Cálculo de Motores (Control Proporcional)
            ajuste_giro = error_angulo * KP_ANGULO
            rpm_base = min(RPM_MAX, dist * KP_DIST)

            rpm_izq = rpm_base - ajuste_giro
            rpm_der = rpm_base + ajuste_giro

            # 5. Mandar señales a ESP32
            # (Si es sentido_marcha=1 va adelante, si es 2 va hacia atrás)
            mandar_orden(esp_1, "A", sentido_marcha, rpm_izq) 
            mandar_orden(esp_1, "B", sentido_marcha, rpm_izq) 
            mandar_orden(esp_2, "A", sentido_marcha, rpm_der) 
            mandar_orden(esp_2, "B", sentido_marcha, rpm_der) 

            # Telemetría
            accion = "ADELANTE" if sentido_marcha == 1 else "REVERSA"
            sys.stdout.write(f"\r[HUD] Dist: {dist:.2f}m | Error Ang: {error_angulo:.1f}° | Acción: {accion}   ")
            sys.stdout.flush()

        time.sleep(0.05)

# ==========================================
# RUTINA DE PRUEBA SIMPLIFICADA
# ==========================================
try:
    print("\nEsperando datos de la cámara...")
    while 0 not in tags_vistos: time.sleep(0.5)

    # Elige a dónde quieres ir
    ir_a_objetivo(1)
    
    # ir_a_objetivo(2) # Descomenta esta si quieres que vaya al 2 después

except KeyboardInterrupt:
    pass
finally:
    frenar()
    esp_1.close(); esp_2.close()
