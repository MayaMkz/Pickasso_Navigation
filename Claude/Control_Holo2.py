"""
Control_Holo2.py — Pickasso AGV Holonómico | Raspberry Pi — Mecanum
==============================================================
[CORRECCIÓN CRÍTICA] CASCADA VISIÓN + IMU
El IMU integra suavemente la orden de visión (omega_heading)
para mantener el rumbo dictado por la PC en todo momento,
rechazando derrapes sin "pelear" contra la cámara.
"""

import serial
import time
import math
import sys
import board
import busio
import adafruit_bno055
import socket
import threading

DIAMETRO_LLANTA_MM = 152.0
CIRCUNFERENCIA_M   = (DIAMETRO_LLANTA_MM * math.pi) / 1000.0   

K_IMU = 0.085   # rad/s / deg
OMEGA_IMU_MAX = 0.20   # rad/s
INVERTIR_IMU = False

RPM_MAX_FISICO = 80.0   # rpm
ESPERA_ESTACION_S = 10.0   # s
TIMEOUT_VISION_S = 0.75    # s
PUERTO_UDP = 5005

vision_lock      = threading.Lock()
nuevo_dato_event = threading.Event()

datos_vision = {
    "vx":             0.0,
    "vy":             0.0,
    "dist":           9.9,
    "estado_mision":  0,
    "omega_heading":  0.0,   
    "activo":         False,
    "ultima_vez":     0.0,
}

cmd_lock = threading.Lock()
cmd = {"vx": 0.0, "vy": 0.0, "omega_heading": 0.0, "habilitado": False}
ctrl_activo = threading.Event()

class FiltroIMU:
    def __init__(self, umbral=10.0, max_rechazos=5):
        self.yaw_ant  = None
        self.umbral   = umbral
        self.rechazos = 0
        self.max_r    = max_rechazos

    def filtrar(self, yaw):
        if self.yaw_ant is None:
            self.yaw_ant = yaw; return yaw
        d = yaw - self.yaw_ant
        if d > 180: d -= 360
        if d < -180: d += 360
        if abs(d) > self.umbral:
            self.rechazos += 1
            if self.rechazos < self.max_r: return self.yaw_ant
            self.yaw_ant = yaw; self.rechazos = 0; return yaw
        self.rechazos = 0; self.yaw_ant = yaw; return yaw

filtro_yaw = FiltroIMU(umbral=10.0, max_rechazos=5)

print("Iniciando hardware Mecanum...")
i2c        = busio.I2C(board.SCL, board.SDA)
sensor_imu = adafruit_bno055.BNO055_I2C(i2c)

try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.05)
    time.sleep(2)
    print("[OK] ESP32 conectados.")
except Exception as e:
    print(f"[ERROR USB] {e}"); sys.exit(1)

def obtener_yaw():
    try:
        y = sensor_imu.euler[0]
        if y is not None: return filtro_yaw.filtrar(y)
    except: pass
    return filtro_yaw.yaw_ant or 0.0

def _cmd(placa, motor, dir_, rpm):
    rpm = max(0.0, min(RPM_MAX_FISICO, abs(rpm)))
    try: placa.write(f"{motor},{dir_},{rpm:.1f}\n".encode())
    except: pass

def frenar():
    for p in (esp_1, esp_2):
        _cmd(p, "A", 0, 0); _cmd(p, "B", 0, 0)
    try: esp_1.reset_input_buffer(); esp_2.reset_input_buffer()
    except: pass

def ms_a_rpm(v_ms):
    return (v_ms / CIRCUNFERENCIA_M) * 60.0

def aplicar_mecanum(vx_ms, vy_ms, omega_imu_rads=0.0):
    omega_ms = omega_imu_rads * 0.15   
    v_FL =  vx_ms + vy_ms + omega_ms
    v_BL =  vx_ms - vy_ms + omega_ms
    v_FR =  vx_ms - vy_ms - omega_ms
    v_BR =  vx_ms + vy_ms - omega_ms

    rpm_FL = ms_a_rpm(v_FL)
    rpm_BL = ms_a_rpm(v_BL)
    rpm_FR = ms_a_rpm(v_FR)
    rpm_BR = ms_a_rpm(v_BR)

    dir_FL = 2 if rpm_FL >= 0 else 1
    _cmd(esp_1, "A", dir_FL, abs(rpm_FL))
    dir_BL = 2 if rpm_BL >= 0 else 1
    _cmd(esp_1, "B", dir_BL, abs(rpm_BL))
    dir_FR = 1 if rpm_FR >= 0 else 2
    _cmd(esp_2, "A", dir_FR, abs(rpm_FR))
    dir_BR = 1 if rpm_BR >= 0 else 2
    _cmd(esp_2, "B", dir_BR, abs(rpm_BR))

def thread_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PUERTO_UDP))
    sock.setblocking(False)
    print(f"[UDP] Escuchando :{PUERTO_UDP}")

    while True:
        fresco = None
        try:
            while True:
                data, _ = sock.recvfrom(256)
                fresco = data   
        except BlockingIOError: pass

        if fresco is not None:
            try:
                p = fresco.decode().split(',')
                if len(p) >= 4:
                    vx_r   = float(p[0])
                    vy_r   = float(p[1])
                    dist_r = float(p[2])
                    est_r  = int(float(p[3]))
                    ohd_r  = float(p[4]) if len(p) >= 5 else 0.0  

                    with vision_lock:
                        datos_vision["vx"]            = vx_r
                        datos_vision["vy"]            = vy_r
                        datos_vision["dist"]          = dist_r
                        datos_vision["estado_mision"] = est_r
                        datos_vision["omega_heading"] = ohd_r
                        datos_vision["activo"]        = True
                        datos_vision["ultima_vez"]    = time.time()

                    with cmd_lock:
                        cmd["vx"]            = vx_r
                        cmd["vy"]            = vy_r
                        cmd["omega_heading"] = ohd_r
                        cmd["habilitado"]    = True

                    nuevo_dato_event.set()
            except Exception: pass
        time.sleep(0.005)

def thread_ctrl():
    DT            = 0.01    
    yaw_ref       = None
    yaw_init      = False

    while True:
        if not ctrl_activo.is_set():
            time.sleep(DT); continue

        with cmd_lock:
            vx_c   = cmd["vx"]
            vy_c   = cmd["vy"]
            ohd_c  = cmd["omega_heading"]
            habili = cmd["habilitado"]

        with vision_lock:
            ultima = datos_vision["ultima_vez"]

        if not habili or (time.time() - ultima > TIMEOUT_VISION_S):
            time.sleep(DT); continue

        yaw_actual = obtener_yaw()
        if not yaw_init:
            yaw_ref  = yaw_actual
            yaw_init = True

        # El IMU ahora sigue ciegamente las correcciones calculadas
        # por la computadora (basadas en el yaw_bloqueado de la pista)
        yaw_ref += math.degrees(ohd_c) * DT
        yaw_ref  = yaw_ref % 360  

        err_yaw = yaw_actual - yaw_ref
        if err_yaw >  180: err_yaw -= 360
        if err_yaw < -180: err_yaw += 360
        
        omega_imu = -K_IMU * err_yaw
        if INVERTIR_IMU: omega_imu = -omega_imu
        omega_imu = max(-OMEGA_IMU_MAX, min(OMEGA_IMU_MAX, omega_imu))

        omega_total = ohd_c + omega_imu
        aplicar_mecanum(vx_c, vy_c, omega_total)

        sys.stdout.write(f"\r[CTRL] vx={vx_c:+.2f} vy={vy_c:+.2f} ω_vis={math.degrees(ohd_c):+.1f}°/s yaw={yaw_actual:.1f}° ")
        sys.stdout.flush()
        time.sleep(DT)

threading.Thread(target=thread_udp,  daemon=True).start()
threading.Thread(target=thread_ctrl, daemon=True).start()
ctrl_activo.set()

print("\n" + "="*44 + "\n  ESPERANDO DATOS DE VISIÓN (UDP)\n" + "="*44)
estado_anterior = 0

try:
    while True:
        nuevo_dato_event.wait(timeout=0.1)
        nuevo_dato_event.clear()

        with vision_lock:
            est_act  = datos_vision["estado_mision"]
            activo   = datos_vision["activo"]
            ultima   = datos_vision["ultima_vez"]

        if not activo or (time.time() - ultima > TIMEOUT_VISION_S):
            ctrl_activo.clear()
            frenar()
            with cmd_lock: cmd["habilitado"] = False
            estado_anterior = 0
            sys.stdout.write("\r[EMERGENCIA] Sin visión.      ")
            sys.stdout.flush()
            continue

        ctrl_activo.set()

        if est_act != estado_anterior and estado_anterior != 0:
            print(f"\n[TRANSICIÓN] estado {estado_anterior} → {est_act}")
            if estado_anterior in (1, 3):
                num_est = 1 if estado_anterior == 1 else 2
                ctrl_activo.clear(); frenar()
                print(f"[■] ESTACIÓN {num_est} — esperando xArm6 ({ESPERA_ESTACION_S:.0f} s)...")
                time.sleep(ESPERA_ESTACION_S)
                ctrl_activo.set()

        if est_act > 4:
            ctrl_activo.clear(); frenar()
            print("\n\n[✓✓] MISIÓN COMPLETADA. Carro detenido."); break

        estado_anterior = est_act
        time.sleep(0.03)

except KeyboardInterrupt:
    print("\n\n[!] Detenido por el usuario.")
finally:
    ctrl_activo.clear(); frenar()
    try: esp_1.close(); esp_2.close()
    except: pass
