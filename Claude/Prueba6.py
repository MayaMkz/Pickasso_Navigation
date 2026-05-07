"""
Prueba6.py — Pickasso AGV | Raspberry Pi Control
==================================================
ARQUITECTURA  (3 threads)
--------------------------
  Thread UDP  (200 Hz)  — drena el buffer, guarda el paquete más fresco.
  Thread CTRL (100 Hz)  — convierte (v, omega) → RPM_L / RPM_R + corrección IMU.
  Thread MAIN           — vigila cambios de estado_mision → ejecuta giros y esperas.

PAQUETE UDP RECIBIDO: "v,omega,dist,estado_mision"
  v            : velocidad lineal  [m/s]
  omega        : velocidad angular [rad/s], con signo
  dist         : distancia al waypoint actual [m]
  estado_mision: int 1-4 enviado por la PC tras capturar cada waypoint

PARÁMETROS DE SINTONIZACIÓN — busca [TUNING] abajo.
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
#  [TUNING A] PARÁMETROS FÍSICOS — medir con cinta, NO estimar
# ══════════════════════════════════════════════════════════════════════
# Diámetro de llanta (mm). Cambia si cambias las ruedas.
DIAMETRO_LLANTA_MM = 127.7
PULSOS_POR_REV     = 20000.0
CIRCUNFERENCIA_M   = (DIAMETRO_LLANTA_MM * math.pi) / 1000.0   # ~0.4011 m

# [CRÍTICO] Distancia entre centros de llanta del mismo eje (m).
# Un error de ±1 cm aquí hace que el carro siempre vaya levemente curvo.
# Mide de centro de goma izquierda a centro de goma derecha.
L_WHEELBASE_M = 0.30   # m  ← MEDIR

# ══════════════════════════════════════════════════════════════════════
#  [TUNING B] CORRECCIÓN IMU (rechazo de perturbaciones)
# ══════════════════════════════════════════════════════════════════════
# El IMU ya NO define la trayectoria. Solo detecta si el carro derivó
# respecto al rumbo integrado y agrega una pequeña corrección.
#
# K_IMU : ganancia de corrección  [rad/s por grado de error].
#   Sube → el carro recupera más rápido de un golpe o derrape.
#   Baja → el IMU apenas influye (útil si el campo magnético es muy ruidoso).
#   Rango útil: 0.02 – 0.10
K_IMU = 0.04   # rad/s per deg

# OMEGA_IMU_MAX : máxima corrección que puede agregar el IMU (rad/s).
#   Impide que un salto magnético puntual corrija en exceso.
OMEGA_IMU_MAX = 0.25   # rad/s

# INVERTIR_IMU : si el carro corrige en la dirección equivocada, pon True.
INVERTIR_IMU  = False

# ══════════════════════════════════════════════════════════════════════
#  [TUNING C] LÍMITES DE SEGURIDAD
# ══════════════════════════════════════════════════════════════════════
# Techo de omega total (visión + IMU) antes de convertir a RPM.
OMEGA_TOTAL_MAX = 1.4   # rad/s

# Tiempo sin paquete UDP antes de activar parada de emergencia (s).
TIMEOUT_VISION_S = 1.0

# RPM para los giros de 90° en las esquinas.
RPM_GIRO = 15   # rpm  ← si el giro se pasa o no llega, ajusta aquí

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
    "v":             0.0,
    "omega":         0.0,
    "dist":          9.9,
    "estado_mision": 0,
    "activo":        False,
    "ultima_vez":    0.0,
}

# Comando al thread de control (escrito por main, leído por CTRL)
cmd_lock = threading.Lock()
cmd = {"v": 0.0, "omega": 0.0, "habilitado": False}

# Señales entre threads
ctrl_activo  = threading.Event()   # permite/pausa el thread CTRL
esquina_lock = threading.Lock()    # protege la maniobra de giro

# ══════════════════════════════════════════════════════════════════════
#  FILTRO IMU ANTI-BRINCOS
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
        if d >  180: d -= 360
        if d < -180: d += 360
        if abs(d) > self.umbral:
            self.rechazos += 1
            if self.rechazos < self.max_r:
                return self.yaw_ant
            self.yaw_ant = yaw; self.rechazos = 0; return yaw
        self.rechazos = 0; self.yaw_ant = yaw; return yaw

filtro_yaw = FiltroIMU(umbral=10.0, max_rechazos=5)

# ══════════════════════════════════════════════════════════════════════
#  HARDWARE
# ══════════════════════════════════════════════════════════════════════
print("Iniciando hardware...")
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

def _cmd_motor(placa, motor, dir_, rpm):
    rpm = max(0, min(100, abs(rpm)))
    try: placa.write(f"{motor},{dir_},{rpm:.1f}\n".encode())
    except: pass

def frenar():
    for p in (esp_1, esp_2):
        _cmd_motor(p, "A", 0, 0)
        _cmd_motor(p, "B", 0, 0)
    try: esp_1.reset_input_buffer(); esp_2.reset_input_buffer()
    except: pass

def aplicar_rpm(rpm_izq, rpm_der):
    """Aplica RPM a los cuatro motores. Signo = dirección."""
    rpm_izq = max(-100, min(100, rpm_izq))
    rpm_der = max(-100, min(100, rpm_der))
    d1A, d1B = (1, 2) if rpm_izq >= 0 else (2, 1)
    d2A, d2B = (1, 2) if rpm_der >= 0 else (2, 1)
    _cmd_motor(esp_1, "A", d1A, abs(rpm_izq))
    _cmd_motor(esp_1, "B", d1B, abs(rpm_izq))
    _cmd_motor(esp_2, "A", d2A, abs(rpm_der))
    _cmd_motor(esp_2, "B", d2B, abs(rpm_der))

def ms_a_rpm(v_ms):
    """Velocidad lineal de rueda (m/s) → RPM."""
    return (v_ms / CIRCUNFERENCIA_M) * 60.0

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

# ══════════════════════════════════════════════════════════════════════
#  DIAGNÓSTICO DE ARRANQUE
# ══════════════════════════════════════════════════════════════════════
def diagnostico():
    print("\n" + "="*44 + "\n  DIAGNÓSTICO DE MOTORES Y ENCODERS\n" + "="*44)
    while True:
        frenar(); time.sleep(0.2)
        i1A, i1B = leer_encoders(esp_1)
        i2A, i2B = leer_encoders(esp_2)
        print("[!] Pulso de prueba 15 RPM...")
        aplicar_rpm(15, 15); time.sleep(0.5)
        frenar(); time.sleep(0.2)
        f1A, f1B = leer_encoders(esp_1)
        f2A, f2B = leer_encoders(esp_2)
        fallos = [abs(f1A-i1A)<50, abs(f1B-i1B)<50,
                  abs(f2A-i2A)<50, abs(f2B-i2B)<50]
        nombres = ["ESP1-A", "ESP1-B", "ESP2-A", "ESP2-B"]
        ok = True
        for n, fallo in zip(nombres, fallos):
            if fallo: print(f"  [FALLA] {n}"); ok = False
        if ok:
            print("[OK] 4 motores y encoders responden correctamente.")
            break
        input(">>> Revisa conexiones y presiona ENTER para reintentar...")

diagnostico()

# ══════════════════════════════════════════════════════════════════════
#  RUTINA DE GIRO EN TANQUE (usa IMU)
# ══════════════════════════════════════════════════════════════════════
def girar_grados(grados, rpm=None):
    """Gira en el lugar sobre el eje del carro. grados con signo."""
    if rpm is None: rpm = RPM_GIRO
    print(f"\n[GIRO] {grados:+.1f}° a {rpm} RPM")
    yaw_ini = obtener_yaw()
    if grados > 0:   # giro izquierda (antihorario)
        _cmd_motor(esp_1, "A", 1, rpm); _cmd_motor(esp_1, "B", 2, rpm)
        _cmd_motor(esp_2, "A", 2, rpm); _cmd_motor(esp_2, "B", 1, rpm)
    else:             # giro derecha (horario)
        _cmd_motor(esp_1, "A", 2, rpm); _cmd_motor(esp_1, "B", 1, rpm)
        _cmd_motor(esp_2, "A", 1, rpm); _cmd_motor(esp_2, "B", 2, rpm)

    objetivo = abs(grados) - 1.5   # margen de frenado
    while True:
        g = obtener_yaw() - yaw_ini
        if g >  180: g -= 360
        if g < -180: g += 360
        if abs(g) >= objetivo:
            break
        time.sleep(0.01)
    frenar()
    print("[OK] Giro completado.")
    time.sleep(0.4)

# ══════════════════════════════════════════════════════════════════════
#  THREAD A — RECEPTOR UDP (200 Hz efectivo)
# ══════════════════════════════════════════════════════════════════════
def thread_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PUERTO_UDP))
    sock.setblocking(False)
    print(f"[UDP] Escuchando en :{PUERTO_UDP}")

    while True:
        fresco = None
        # Drenar TODO el buffer, quedarse con el paquete más reciente
        try:
            while True:
                data, _ = sock.recvfrom(256)
                fresco = data
        except BlockingIOError:
            pass

        if fresco is not None:
            try:
                p = fresco.decode().split(',')
                if len(p) >= 4:
                    v_r     = float(p[0])
                    omega_r = float(p[1])
                    dist_r  = float(p[2])
                    est_r   = int(float(p[3]))

                    with vision_lock:
                        datos_vision["v"]             = v_r
                        datos_vision["omega"]         = omega_r
                        datos_vision["dist"]          = dist_r
                        datos_vision["estado_mision"] = est_r
                        datos_vision["activo"]        = True
                        datos_vision["ultima_vez"]    = time.time()

                    # Propagar comando inmediatamente al thread CTRL
                    with cmd_lock:
                        cmd["v"]          = v_r
                        cmd["omega"]      = omega_r
                        cmd["habilitado"] = True

                    nuevo_dato_event.set()
            except Exception:
                pass

        time.sleep(0.005)   # 200 Hz sin quemar CPU

# ══════════════════════════════════════════════════════════════════════
#  THREAD B — CONTROL A 100 Hz
#  Convierte (v, omega) → RPM_L / RPM_R y agrega corrección IMU.
# ══════════════════════════════════════════════════════════════════════
def thread_ctrl():
    """
    ROL DEL IMU:
    1. Integra el omega comandado para llevar un 'yaw de referencia' esperado.
    2. Compara con el yaw real del BNO055.
    3. Si hay diferencia → agrega una pequeña corrección a omega.
    Esto rechaza perturbaciones (derrapes, golpes, piso irregular)
    sin interferir con la trayectoria que define visión.
    """
    DT = 0.01   # 100 Hz
    yaw_ref   = None
    yaw_init  = False

    while True:
        if not ctrl_activo.is_set():
            time.sleep(DT)
            continue

        with cmd_lock:
            v_c    = cmd["v"]
            om_c   = cmd["omega"]
            habili = cmd["habilitado"]

        with vision_lock:
            ultima = datos_vision["ultima_vez"]

        if not habili or (time.time() - ultima > TIMEOUT_VISION_S):
            time.sleep(DT)
            continue

        yaw_actual = obtener_yaw()

        # Inicializar referencia de yaw al primer dato válido
        if not yaw_init:
            yaw_ref = yaw_actual
            yaw_init = True

        # Integrar el omega comandado (deg/s)
        yaw_ref += math.degrees(om_c) * DT
        yaw_ref  = yaw_ref % 360   # mantener en [0, 360)

        # Error de heading: cuánto derivó el carro respecto al expected
        err_yaw = yaw_actual - yaw_ref
        if err_yaw >  180: err_yaw -= 360
        if err_yaw < -180: err_yaw += 360

        # Corrección IMU (pequeña)
        omega_imu = -K_IMU * err_yaw
        if INVERTIR_IMU: omega_imu = -omega_imu
        omega_imu = max(-OMEGA_IMU_MAX, min(OMEGA_IMU_MAX, omega_imu))

        # Omega total
        omega_total = max(-OMEGA_TOTAL_MAX, min(OMEGA_TOTAL_MAX, om_c + omega_imu))

        # ── Modelo diferencial: (v, omega) → velocidad lineal de cada rueda ──
        # v_izq = v - omega * L/2
        # v_der = v + omega * L/2
        v_izq_ms = v_c - omega_total * (L_WHEELBASE_M / 2.0)
        v_der_ms = v_c + omega_total * (L_WHEELBASE_M / 2.0)

        rpm_izq = ms_a_rpm(v_izq_ms)
        rpm_der = ms_a_rpm(v_der_ms)

        aplicar_rpm(rpm_izq, rpm_der)

        sys.stdout.write(
            f"\r[CTRL 100Hz] v={v_c:.3f} ω_vis={math.degrees(om_c):+.1f}° "
            f"ω_imu={math.degrees(omega_imu):+.1f}° "
            f"RPM L={rpm_izq:+.1f} R={rpm_der:+.1f}    "
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
#  MAIN THREAD — LÓGICA DE MISIÓN Y ESQUINAS
#  Corre a ritmo lento (~30 Hz), solo toma decisiones grandes.
# ══════════════════════════════════════════════════════════════════════
print("\n" + "="*44 + "\n  ESPERANDO DATOS DE VISIÓN (UDP)\n" + "="*44)

#  estado_mision que va llegando de la PC:
#   1 → yendo a Est.1
#   2 → yendo a Esq.1   (la PC lo incrementó al capturar Est.1)
#   3 → yendo a Est.2   (la PC lo incrementó al capturar Esq.1)
#   4 → yendo a Esq.2   (la PC lo incrementó al capturar Est.2)
#   >4 → misión completa

estado_anterior   = 0     # último estado procesado por el main
sentido_giro      = None  # se toma del primer paquete (campo 'sentido' fue eliminado,
                           # usamos producto cruz al inicio). Por ahora fijado manualmente.

# ── Definir el sentido de giro ──────────────────────────────────────
# La PC ya calcula el sentido internamente; para transferirlo podemos
# agregar el campo de sentido al paquete. Por ahora: ajusta este valor.
# +1 = izquierda (antihorario), -1 = derecha (horario).
SENTIDO_GIRO_MANUAL = 1   # ← ajustar según el layout de tu mesa

try:
    while True:
        # Esperar dato nuevo o timeout 100ms
        nuevo_dato_event.wait(timeout=0.1)
        nuevo_dato_event.clear()

        with vision_lock:
            est_act  = datos_vision["estado_mision"]
            activo   = datos_vision["activo"]
            ultima   = datos_vision["ultima_vez"]
            dist_act = datos_vision["dist"]

        # ── Sin visión: parada de emergencia ────────────────────
        if not activo or (time.time() - ultima > TIMEOUT_VISION_S):
            ctrl_activo.clear()
            frenar()
            with cmd_lock: cmd["habilitado"] = False
            estado_anterior = 0   # reset para redetectar al volver
            sys.stdout.write("\r[EMERGENCIA] Sin visión. Motores detenidos.    ")
            sys.stdout.flush()
            continue

        ctrl_activo.set()

        # ── Detección de transición de estado (waypoint capturado) ──
        if est_act != estado_anterior and estado_anterior != 0:
            # La PC acaba de capturar un waypoint y pasó al siguiente estado.
            # En el estado ANTERIOR estaban las estaciones/esquinas.
            print(f"\n[TRANSICIÓN] estado {estado_anterior} → {est_act}")

            with esquina_lock:
                ctrl_activo.clear()   # pausar PID mientras maniobra
                frenar()
                time.sleep(0.15)

                # Estado anterior 1 = estación 1, 3 = estación 2
                if estado_anterior in (1, 3):
                    num = 1 if estado_anterior == 1 else 2
                    print(f"[■] ESTACIÓN {num}: esperando brazo xArm6 (10 s)...")
                    time.sleep(10)
                    print("[►] Brazo listo. Girando 90°...")
                else:
                    print("[◄►] Esquina: girando 90°...")

                girar_grados(90.0 * SENTIDO_GIRO_MANUAL, rpm=RPM_GIRO)
                ctrl_activo.set()

        # ── Misión completada ────────────────────────────────────
        if est_act > 4:
            ctrl_activo.clear()
            frenar()
            print("\n\n[✓✓] MISIÓN COMPLETADA. Motores detenidos.")
            break

        estado_anterior = est_act
        time.sleep(0.03)   # ~33 Hz

except KeyboardInterrupt:
    print("\n\n[!] Detenido por el usuario.")
finally:
    ctrl_activo.clear()
    frenar()
    try: esp_1.close(); esp_2.close()
    except: pass
    print("Sistemas apagados.")
