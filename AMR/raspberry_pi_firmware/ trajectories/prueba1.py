import serial, time, math, socket, threading, board, busio, adafruit_bno055

# --- CONFIGURACIÓN ---
KP_RUMBO = 0.6
KP_VISION = 25.0
rpm_base = 30.0
PUERTO_UDP = 5005

# Diccionario para almacenar posiciones de todos los tags vistos
tags_vistos = {} 

def escuchar_wifi():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PUERTO_UDP))
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            paquetes = data.decode('utf-8').split('|')
            for p in paquetes:
                id_tag, coords = p.split(':')
                x, y, z = map(float, coords.split(','))
                tags_vistos[int(id_tag)] = {"x": x, "y": y, "dist": math.sqrt(x**2 + y**2)}
        except: pass

threading.Thread(target=escuchar_wifi, daemon=True).start()

# --- INICIALIZACIÓN HARDWARE ---
i2c = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)
esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)

def mandar_orden(motor, direccion, rpm):
    cmd = f"{motor},{direccion},{round(rpm, 1)}\n".encode()
    if motor in ['A', 'B']: esp_1.write(cmd) # Ajustar según tu mapeo
    # Agregar lógica para esp_2 si es necesario

def navegar_a_target(target_id):
    print(f"Buscando objetivo: Tag {target_id}...")
    while True:
        if 0 in tags_vistos and target_id in tags_vistos:
            dx = tags_vistos[target_id]["x"] - tags_vistos[0]["x"]
            dist = math.sqrt(dx**2 + (tags_vistos[target_id]["y"] - tags_vistos[0]["y"])**2)
            
            if dist < 0.10: # Umbral de llegada
                print(f"Llegamos al Tag {target_id}")
                # Frenar
                break
            
            ajuste = dx * KP_VISION
            mandar_orden('A', 1, rpm_base - ajuste) # Simplificado para el ejemplo
            # Aquí incluirías el resto de tus motores y la corrección del IMU
        time.sleep(0.05)

# --- MISION ---
try:
    navegar_a_target(1) # Ir al primer punto (1.3m)
    # Aquí podrías meter un giro de 90 grados con el IMU
    navegar_a_target(2) # Ir al segundo punto (0.6m)
finally:
    # Frenar todo
    pass
