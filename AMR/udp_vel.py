import time
import math
import socket
import board
import busio
import adafruit_bno055

# =========================
# CONFIGURACIÓN
# =========================

UDP_IP = "0.0.0.0"
UDP_PORT = 5006

DIAMETRO_LLANTA_MM = 160.0
PULSOS_POR_REV = 20000.0

CIRCUNFERENCIA_M = (DIAMETRO_LLANTA_MM * math.pi) / 1000.0
PULSOS_POR_METRO = PULSOS_POR_REV / CIRCUNFERENCIA_M

RPM_BASE = 15
KP_ESTABILIZACION = 0.0

# =========================
# IMU
# =========================

i2c = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)

def obtener_yaw():
    try:
        yaw = sensor_imu.euler[0]
        if yaw is not None:
            return yaw
    except Exception:
        pass
    return 0.0

# =========================
# SERIAL ESP32
# =========================

try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
    time.sleep(2)
    print("ESP32 conectadas.")
except Exception as e:
    print(f"Error serial: {e}")
    exit()

def mandar_orden(placa, motor, direccion, rpm):
    comando = f"{motor},{direccion},{round(abs(rpm), 1)}\n"
    placa.write(comando.encode("utf-8"))

def frenar_y_limpiar():
    mandar_orden(esp_1, "A", 0, 0)
    mandar_orden(esp_1, "B", 0, 0)
    mandar_orden(esp_2, "A", 0, 0)
    mandar_orden(esp_2, "B", 0, 0)
    esp_1.reset_input_buffer()
    esp_2.reset_input_buffer()

def leer_encoders(placa):
    ultima_linea = ""

    while placa.in_waiting > 0:
        try:
            ultima_linea = placa.readline().decode("utf-8").rstrip()
        except Exception:
            pass

    if "A:" in ultima_linea:
        try:
            partes = ultima_linea.split(",")
            enc_a = int(partes[0].split(":")[1])
            enc_b = int(partes[1].split(":")[1])
            return enc_a, enc_b
        except Exception:
            pass

    return None, None

# =========================
# MOVIMIENTOS
# =========================

def mover_adelante(metros, rpm=RPM_BASE):
    print(f"Moviendo ADELANTE {metros:.2f} m")

    pulsos_objetivo = int(metros * PULSOS_POR_METRO)

    enc_ini, _ = leer_encoders(esp_1)
    while enc_ini is None:
        enc_ini, _ = leer_encoders(esp_1)

    while True:
        enc_act, _ = leer_encoders(esp_1)

        if enc_act is not None:
            if abs(enc_act - enc_ini) >= pulsos_objetivo:
                break

        mandar_orden(esp_1, "A", 2, rpm)
        mandar_orden(esp_1, "B", 2, rpm)
        mandar_orden(esp_2, "A", 1, rpm)
        mandar_orden(esp_2, "B", 1, rpm)

        time.sleep(0.02)

    frenar_y_limpiar()

def mover_atras(metros, rpm=RPM_BASE):
    print(f"Moviendo ATRAS {metros:.2f} m")

    pulsos_objetivo = int(metros * PULSOS_POR_METRO)

    enc_ini, _ = leer_encoders(esp_1)
    while enc_ini is None:
        enc_ini, _ = leer_encoders(esp_1)

    while True:
        enc_act, _ = leer_encoders(esp_1)

        if enc_act is not None:
            if abs(enc_act - enc_ini) >= pulsos_objetivo:
                break

        mandar_orden(esp_1, "A", 1, rpm)
        mandar_orden(esp_1, "B", 1, rpm)
        mandar_orden(esp_2, "A", 2, rpm)
        mandar_orden(esp_2, "B", 2, rpm)

        time.sleep(0.02)

    frenar_y_limpiar()

def mover_derecha(metros, rpm=RPM_BASE):
    print(f"Moviendo DERECHA {metros:.2f} m")

    pulsos_objetivo = int(metros * PULSOS_POR_METRO * 1.1)

    enc_ini, _ = leer_encoders(esp_1)
    while enc_ini is None:
        enc_ini, _ = leer_encoders(esp_1)

    while True:
        enc_act, _ = leer_encoders(esp_1)

        if enc_act is not None:
            if abs(enc_act - enc_ini) >= pulsos_objetivo:
                break

        mandar_orden(esp_1, "A", 2, rpm)
        mandar_orden(esp_1, "B", 1, rpm)
        mandar_orden(esp_2, "A", 2, rpm)
        mandar_orden(esp_2, "B", 1, rpm)

        time.sleep(0.02)

    frenar_y_limpiar()

def mover_izquierda(metros, rpm=RPM_BASE):
    print(f"Moviendo IZQUIERDA {metros:.2f} m")

    pulsos_objetivo = int(metros * PULSOS_POR_METRO * 1.1)

    enc_ini, _ = leer_encoders(esp_1)
    while enc_ini is None:
        enc_ini, _ = leer_encoders(esp_1)

    while True:
        enc_act, _ = leer_encoders(esp_1)

        if enc_act is not None:
            if abs(enc_act - enc_ini) >= pulsos_objetivo:
                break

        mandar_orden(esp_1, "A", 1, rpm)
        mandar_orden(esp_1, "B", 2, rpm)
        mandar_orden(esp_2, "A", 1, rpm)
        mandar_orden(esp_2, "B", 2, rpm)

        time.sleep(0.02)

    frenar_y_limpiar()

# =========================
# UDP
# =========================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Escuchando comandos de segmento en puerto {UDP_PORT}")
print("Formato: F,0.30 | B,0.30 | R,0.30 | L,0.30 | S,0")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        msg = data.decode("utf-8").strip()

        print(f"Recibido: {msg}")

        partes = msg.split(",")

        if len(partes) != 2:
            print("Formato inválido")
            continue

        comando = partes[0].upper()
        distancia = float(partes[1])

        if comando == "F":
            mover_adelante(distancia)

        elif comando == "B":
            mover_atras(distancia)

        elif comando == "R":
            mover_derecha(distancia)

        elif comando == "L":
            mover_izquierda(distancia)

        elif comando == "S":
            frenar_y_limpiar()

        else:
            print("Comando no reconocido")

except KeyboardInterrupt:
    print("Deteniendo...")
    frenar_y_limpiar()

finally:
    frenar_y_limpiar()
    esp_1.close()
    esp_2.close()
    sock.close()'
