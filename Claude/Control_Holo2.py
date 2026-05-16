"""
Prueba7.py — Pickasso AGV Holonómico | Raspberry Pi — Mecanum
==============================================================
CAMBIOS respecto a Prueba6.py
------------------------------
[1] CINEMÁTICA MECANUM — reemplaza el modelo diferencial.
    Recibe (vx, vy) del robot y calcula la velocidad de cada rueda:
        FL = vx + vy       (esp1 Motor A)
        BL = vx - vy       (esp1 Motor B)
        FR = vx - vy       (esp2 Motor A)
        BR = vx + vy       (esp2 Motor B)
    Las direcciones de motor se extraen del programa Holo_cuadrado.py.

[2] IMU = SOLO ESTABILIZACIÓN DE HEADING
    El robot holonómico NO gira en la trayectoria.
    El IMU detecta si el carro rotó (por inercia, piso, golpe) y agrega
    una corrección de omega pequeña que se inyecta en el mixing Mecanum:
        FL -= omega_imu    BL -= omega_imu
        FR += omega_imu    BR += omega_imu

[3] SE ELIMINA girar_grados — el carro ya no rota en esquinas.

[4] DETECCIÓN DE ESTACIONES — el main thread vigila cambios de
    estado_mision para 1→2 (estación 1) y 3→4 (estación 2),
    pausa el control y espera al xArm6.

[5] DIÁMETRO DE LLANTA ACTUALIZADO a 152 mm (ruedas Mecanum).

PAQUETE UDP RECIBIDO: "vx,vy,dist,estado_mision"
  vx           : velocidad frontal del robot [m/s]
  vy           : velocidad lateral del robot [m/s, + = derecha]
  dist         : distancia al waypoint [m]
  estado_mision: int 1-4

PARÁMETROS QUE PUEDES TOCAR — ver sección [TUNING] abajo.
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

# ══════════════════════════════════════════════════════════════════════
#  [TUNING A] PARÁMETROS FÍSICOS — medir, no estimar
# ══════════════════════════════════════════════════════════════════════
# Diámetro de las ruedas Mecanum (mm).
DIAMETRO_LLANTA_MM = 152.0
PULSOS_POR_REV     = 20000.0
CIRCUNFERENCIA_M   = (DIAMETRO_LLANTA_MM * math.pi) / 1000.0   # ~0.4775 m

# ══════════════════════════════════════════════════════════════════════
#  [TUNING B] ESTABILIZACIÓN DE HEADING (IMU)
# ══════════════════════════════════════════════════════════════════════
# El robot holonómico debe mantener su orientación constante.
# El IMU detecta desviaciones y las corrige vía omega_imu.
#
# K_IMU : ganancia de corrección de heading [rad/s por grado de error].
#   Sube si el carro rota cuando debería ir recto.
#   Baja si el IMU oscila o "pelea" contra la trayectoria.
#   Rango útil: 0.02 – 0.10
K_IMU = 0.04   # rad/s / deg

# OMEGA_IMU_MAX : máxima corrección del IMU (rad/s).
#   Evita que un pico magnético domine el control.
OMEGA_IMU_MAX = 0.20   # rad/s

# INVERTIR_IMU : pon True si la corrección gira el carro en la
#   dirección equivocada al hacer la primera prueba.
INVERTIR_IMU = False

# ══════════════════════════════════════════════════════════════════════
#  [TUNING C] CONVERSIÓN m/s → RPM
# ══════════════════════════════════════════════════════════════════════
# RPM_MAX_FISICO : límite máximo de RPM que aceptan los ESP32.
#   Ajusta si los motores hacen ruido o se sobrecalientan a 100 RPM.
RPM_MAX_FISICO = 80.0   # rpm

# ══════════════════════════════════════════════════════════════════════
#  [TUNING D] TIMING DE ESTACIONES
# ══════════════════════════════════════════════════════════════════════
# ESPERA_ESTACION_S : segundos que el carro espera al brazo xArm6.
#   Ajusta según el ciclo real del brazo.
ESPERA_ESTACION_S = 10.0   # s

# TIMEOUT_VISION_S : segundos sin UDP antes de parada de emergencia.
TIMEOUT_VISION_S = 1.0    # s

# ══════════════════════════════════════════════════════════════════════
#  RED
# ══════════════════════════════════════════════════════════════════════
PUERTO_UDP = 5005

# ══════════════════════════════════════════════════════════════════════
#  ESTADO COMPARTIDO ENTRE THREADS
# ══════════════════════════════════════════════════════════════════════
vision_lock      = threading.Lock()
nuevo_dato_event = threading.Event()

datos_vision = {
    "vx":             0.0,
    "vy":             0.0,
    "dist":           9.9,
    "estado_mision":  0,
    "omega_heading":  0.0,   # corrección de orientación [rad/s] desde visión
    "activo":         False,
    "ultima_vez":     0.0,
}

cmd_lock = threading.Lock()
cmd = {"vx": 0.0, "vy": 0.0, "omega_heading": 0.0, "habilitado": False}

ctrl_activo = threading.Event()

# ══════════════════════════════════════════════════════════════════════
#  FILTRO IMU ANTI-BRINCOS — sin cambios
# ══════════════════════════════════════════════════════════════════════
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

# ══════════════════════════════════════════════════════════════════════
#  HARDWARE
# ══════════════════════════════════════════════════════════════════════
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
    """Velocidad lineal de llanta (m/s) → RPM."""
    return (v_ms / CIRCUNFERENCIA_M) * 60.0

# ══════════════════════════════════════════════════════════════════════
#  CINEMÁTICA MECANUM
#  Basada en Holo_cuadrado.py (orden y direcciones verificadas).
#
#  Distribución de motores:
#    esp_1 Motor A = Frontal Izquierdo (FL)
#    esp_1 Motor B = Trasero  Izquierdo (BL)
#    esp_2 Motor A = Frontal  Derecho   (FR)
#    esp_2 Motor B = Trasero  Derecho   (BR)
#
#  Convención de dirección:
#    Lado izquierdo (esp_1): dir2 = adelante (+), dir1 = atrás (-)
#    Lado derecho   (esp_2): dir1 = adelante (+), dir2 = atrás (-)
#
#  Mixing (velocidades normalizadas, + = adelante del robot):
#    FL = vx + vy + omega_imu   (omega empuja hacia un lado para corregir rotación)
#    BL = vx - vy + omega_imu
#    FR = vx - vy - omega_imu
#    BR = vx + vy - omega_imu
#
#  Donde:
#    vx > 0  →  robot avanza
#    vy > 0  →  robot se desplaza a la DERECHA
#    omega_imu > 0  →  corrige rotación antihoraria (CCW)
# ══════════════════════════════════════════════════════════════════════
def aplicar_mecanum(vx_ms, vy_ms, omega_imu_rads=0.0):
    """
    vx_ms      : velocidad frontal en m/s
    vy_ms      : velocidad lateral en m/s (+ = derecha)
    omega_imu_rads : corrección de heading en rad/s
    """
    # Convertir omega de rad/s a m/s lineal de llanta
    # Para Mecanum, la corrección de rotación afecta a todas las ruedas igual
    # que en un diferencial. Usamos un factor de escala razonable.
    omega_ms = omega_imu_rads * 0.15   # 0.15 m es aprox. radio de giro del carro

    # Velocidades de cada rueda en m/s
    v_FL =  vx_ms + vy_ms + omega_ms
    v_BL =  vx_ms - vy_ms + omega_ms
    v_FR =  vx_ms - vy_ms - omega_ms
    v_BR =  vx_ms + vy_ms - omega_ms

    # Convertir a RPM
    rpm_FL = ms_a_rpm(v_FL)
    rpm_BL = ms_a_rpm(v_BL)
    rpm_FR = ms_a_rpm(v_FR)
    rpm_BR = ms_a_rpm(v_BR)

    # Aplicar a los motores con las direcciones correctas de Holo_cuadrado.py
    # FL (esp1 A): positivo = dir2
    dir_FL = 2 if rpm_FL >= 0 else 1
    _cmd(esp_1, "A", dir_FL, abs(rpm_FL))

    # BL (esp1 B): positivo = dir2
    dir_BL = 2 if rpm_BL >= 0 else 1
    _cmd(esp_1, "B", dir_BL, abs(rpm_BL))

    # FR (esp2 A): positivo = dir1
    dir_FR = 1 if rpm_FR >= 0 else 2
    _cmd(esp_2, "A", dir_FR, abs(rpm_FR))

    # BR (esp2 B): positivo = dir1
    dir_BR = 1 if rpm_BR >= 0 else 2
    _cmd(esp_2, "B", dir_BR, abs(rpm_BR))

# ══════════════════════════════════════════════════════════════════════
#  DIAGNÓSTICO DE ARRANQUE
# ══════════════════════════════════════════════════════════════════════
def leer_encoders(placa):
    placa.reset_input_buffer(); time.sleep(0.01)
    ultima = ""
    for _ in range(5):
        while placa.in_waiting:
            try: ultima = placa.readline().decode().rstrip()
            except: pass
        if "A:" in ultima and "B:" in ultima:
            try:
                p = ultima.split(',')
                return int(p[0].split(':')[1]), int(p[1].split(':')[1])
            except: pass
        time.sleep(0.01)
    return 0, 0

def diagnostico():
    print("\n" + "="*44 + "\n  DIAGNÓSTICO MECANUM (4 ruedas)\n" + "="*44)
    while True:
        frenar(); time.sleep(0.2)
        i1A,i1B = leer_encoders(esp_1)
        i2A,i2B = leer_encoders(esp_2)
        print("[!] Pulso adelante 15 RPM...")
        # Test adelante
        _cmd(esp_1,"A",2,15); _cmd(esp_1,"B",2,15)
        _cmd(esp_2,"A",1,15); _cmd(esp_2,"B",1,15)
        time.sleep(0.5); frenar(); time.sleep(0.2)
        f1A,f1B = leer_encoders(esp_1)
        f2A,f2B = leer_encoders(esp_2)
        nombres = ["FL(esp1A)","BL(esp1B)","FR(esp2A)","BR(esp2B)"]
        deltas  = [abs(f1A-i1A),abs(f1B-i1B),abs(f2A-i2A),abs(f2B-i2B)]
        ok = True
        for n,d in zip(nombres,deltas):
            if d < 50: print(f"  [FALLA] {n} — delta={d}"); ok = False
            else:      print(f"  [OK]    {n} — delta={d}")
        if ok:
            print("[OK] Las 4 ruedas Mecanum responden."); break
        input(">>> Revisa conexiones y presiona ENTER...")

diagnostico()

# ══════════════════════════════════════════════════════════════════════
#  THREAD A — RECEPTOR UDP (200 Hz efectivo)
# ══════════════════════════════════════════════════════════════════════
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
                fresco = data   # solo el más reciente
        except BlockingIOError:
            pass

        if fresco is not None:
            try:
                p = fresco.decode().split(',')
                if len(p) >= 4:
                    vx_r   = float(p[0])
                    vy_r   = float(p[1])
                    dist_r = float(p[2])
                    est_r  = int(float(p[3]))
                    ohd_r  = float(p[4]) if len(p) >= 5 else 0.0  # omega_heading

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
            except Exception:
                pass

        time.sleep(0.005)

# ══════════════════════════════════════════════════════════════════════
#  THREAD B — CONTROL MECANUM A 100 Hz
# ══════════════════════════════════════════════════════════════════════
def thread_ctrl():
    """
    Recibe (vx, vy, omega_heading) del cmd compartido.

    omega_heading viene de la PC (visión sabe el ángulo del segmento
    y la orientación real del robot). Es la corrección principal.

    El IMU agrega rechazo de perturbaciones de alta frecuencia
    (vibraciones, derrapes, inercia) encima del comando de visión.

    omega_total = omega_heading (visión) + omega_imu (perturbaciones)
    """
    DT       = 0.01   # 100 Hz
    yaw_ref  = None
    yaw_init = False

    while True:
        if not ctrl_activo.is_set():
            time.sleep(DT); continue

        with cmd_lock:
            vx_c   = cmd["vx"]
            vy_c   = cmd["vy"]
            ohd_c  = cmd["omega_heading"]   # corrección principal de orientación
            habili = cmd["habilitado"]

        with vision_lock:
            ultima = datos_vision["ultima_vez"]

        if not habili or (time.time() - ultima > TIMEOUT_VISION_S):
            time.sleep(DT); continue

        yaw_actual = obtener_yaw()

        # Inicializar referencia de yaw al arrancar
        # (solo para el rechazo de perturbaciones del IMU)
        if not yaw_init:
            yaw_ref  = yaw_actual
            yaw_init = True

        # Actualizar yaw_ref integrando omega_heading (en deg)
        # Así el IMU no "pelea" contra la corrección de orientación de visión
        yaw_ref += math.degrees(ohd_c) * DT
        yaw_ref  = yaw_ref % 360

        # Error de heading residual (perturbaciones no capturadas por visión)
        err_yaw = yaw_actual - yaw_ref
        if err_yaw >  180: err_yaw -= 360
        if err_yaw < -180: err_yaw += 360

        # Corrección IMU: solo perturbaciones residuales
        omega_imu = -K_IMU * err_yaw
        if INVERTIR_IMU: omega_imu = -omega_imu
        omega_imu = max(-OMEGA_IMU_MAX, min(OMEGA_IMU_MAX, omega_imu))

        # Omega total: visión manda la orientación, IMU rechaza perturbaciones
        omega_total = ohd_c + omega_imu

        aplicar_mecanum(vx_c, vy_c, omega_total)

        sys.stdout.write(
            f"\r[CTRL 100Hz] vx={vx_c:+.3f} vy={vy_c:+.3f}  "
            f"ω_vis={math.degrees(ohd_c):+.1f}°/s  "
            f"ω_imu={math.degrees(omega_imu):+.1f}°/s  "
            f"err_yaw={err_yaw:+.1f}°    "
        )
        sys.stdout.flush()
        time.sleep(DT)

# ══════════════════════════════════════════════════════════════════════
#  ARRANCAR THREADS
# ══════════════════════════════════════════════════════════════════════
threading.Thread(target=thread_udp,  daemon=True).start()
threading.Thread(target=thread_ctrl, daemon=True).start()
ctrl_activo.set()

# ══════════════════════════════════════════════════════════════════════
#  MAIN THREAD — LÓGICA DE ESTACIONES
#  El carro holonómico NO gira en esquinas. Solo pausa en estaciones.
#  Transiciones:
#    estado 1→2 : llega a Estación 1 → esperar xArm6
#    estado 3→4 : llega a Estación 2 → esperar xArm6
#    estado 2→3 y 4→5 : esquinas, sin pausa, solo cambia de dirección
# ══════════════════════════════════════════════════════════════════════
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
            dist_act = datos_vision["dist"]

        # ── Sin visión: parada de emergencia ────────────────
        if not activo or (time.time() - ultima > TIMEOUT_VISION_S):
            ctrl_activo.clear()
            frenar()
            with cmd_lock: cmd["habilitado"] = False
            estado_anterior = 0
            sys.stdout.write("\r[EMERGENCIA] Sin visión. Motores detenidos.      ")
            sys.stdout.flush()
            continue

        ctrl_activo.set()

        # ── Detectar transición de estado ────────────────────
        if est_act != estado_anterior and estado_anterior != 0:
            print(f"\n[TRANSICIÓN] estado {estado_anterior} → {est_act}")

            # Estaciones: la PC envía estado 2 cuando captura el waypoint
            # del estado 1 (est. 1), y estado 4 cuando captura el del estado 3 (est. 2).
            if estado_anterior in (1, 3):
                num_est = 1 if estado_anterior == 1 else 2
                ctrl_activo.clear()
                frenar()
                print(f"[■] ESTACIÓN {num_est} — esperando xArm6 ({ESPERA_ESTACION_S:.0f} s)...")
                time.sleep(ESPERA_ESTACION_S)
                print(f"[►] Estación {num_est} completada. Reanudando trayectoria.")
                ctrl_activo.set()

            # Esquinas (estado 2→3 o similar): sin pausa,
            # vision ya manda el nuevo (vx, vy) directamente.

        # ── Misión completada ────────────────────────────────
        if est_act > 4:
            ctrl_activo.clear()
            frenar()
            print("\n\n[✓✓] MISIÓN COMPLETADA. Carro detenido.")
            break

        estado_anterior = est_act
        time.sleep(0.03)

except KeyboardInterrupt:
    print("\n\n[!] Detenido por el usuario.")
finally:
    ctrl_activo.clear()
    frenar()
    try: esp_1.close(); esp_2.close()
    except: pass
    print("Sistemas apagados.")
