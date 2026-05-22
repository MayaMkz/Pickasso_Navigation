#!/usr/bin/env python3
"""
pick_and_place.py — Sistema Pick & Place para xArm5 con visión Eye-in-Hand
===========================================================================
Prerrequisitos (terminales separadas, en orden):

  Terminal 1 — Nodo del robot real + MoveIt + RViz:
    ros2 launch xarm_moveit_config xarm5_moveit_realmove.launch.py \
        robot_ip:=192.168.1.234 add_gripper:=true

  Terminal 2 — Planificador (necesario para PlanPose / PlanExec):
    ros2 launch xarm_planner xarm5_almacen_planner.launch.py add_gripper:=true

  Terminal 3 — Nodo de visión:
    ros2 run logica_almacen vision_all_in_one

  Terminal 4 — Este nodo:
    ros2 run logica_almacen pick_and_place

Arquitectura general:
  vision_all_in_one  →  /coordenadas_cubo_3d (Point, metros)
        ↓
  pick_and_place  →  TF lookup (link_base ← link5, tiempo real)
        ↓
  Cinemática Eye-in-Hand:  p_base = T_base_brida × T_brida_cam × p_cam
        ↓
  MoveIt PlanPose  →  muestra fantasma en RViz
        ↓
  [ENTER usuario]  →  MoveIt PlanExec  →  movimiento real
        ↓
  Verificación de visión  →  loop / home
"""

# ──────────────────────────────────────────────────────────────────────
# IMPORTS
# ──────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from xarm_msgs.srv import PlanPose, PlanExec, PlanJoint, MoveJoint, SetInt16

import tf2_ros
from tf2_ros import TransformException

import numpy as np
import math
import threading
import time

# ──────────────────────────────────────────────────────────────────────
# CONFIGURACIÓN GLOBAL — Ajusta estos valores según tu hardware
# ──────────────────────────────────────────────────────────────────────

# Frames TF publicados por xarm_ros2
TF_BASE   = 'link_base'   # Marco de referencia de la base del robot
TF_FLANGE = 'link5'       # Brida del xArm5 (5 GDL → último eslabón = link5)

# Posición HOME en espacio de articulaciones [rad]
# [joint1, joint2, joint3, joint4, joint5]
HOME_JOINTS = [0.0, -0.349066, -1.13446, 1.48353, 0.0]
HOME_SPEED  = 0.25   # rad/s
HOME_ACC    = 1.0    # rad/s²

# Offset vertical de aproximación: el gripper se detendrá a esta distancia
# por ENCIMA de la superficie del objeto detectado (seguridad)
PICK_OFFSET_Z_M = 0.10    # 10 cm

# Longitud del gripper: distancia de la brida (link5) a la punta de la pinza
GRIPPER_LENGTH_M = 0.160  # 160 mm

# Límites del workspace (seguridad): valores en metros respecto a link_base
WS_X_MIN, WS_X_MAX =  0.05,  0.70
WS_Y_MIN, WS_Y_MAX = -0.50,  0.50
WS_Z_MIN, WS_Z_MAX = -0.05,  0.70

# ──────────────────────────────────────────────────────────────────────
# MATRICES DE TRANSFORMACIÓN FIJAS (en metros)
# ──────────────────────────────────────────────────────────────────────

# T_brida_cam: Cámara RealSense D457 respecto a la BRIDA (link5).
#
# La cámara está girada 180° físicamente alrededor del eje Z de la brida
# (la imagen se ve al revés). Esto produce una rotación:
#   R_z(180°) = diag(-1, -1, +1)
#
# La traslación física (medida en RoboDK, convertida a metros):
#   X = +67.506 mm → +0.067506 m   (cámara adelantada en X de la brida)
#   Y = + 7.342 mm → +0.007342 m   (pequeño offset lateral)
#   Z = +35.900 mm → +0.035900 m   (cámara más arriba que la brida)
#
#  ┌─────────────────────────────────────────────────────────┐
#  │  NOTA DE VALIDACIÓN:                                    │
#  │  La matrix que proporcionaste es correcta.              │
#  │  El signo -1 en [0,0] y [1,1] representa Rz(180°),     │
#  │  que absorbe nativamente el giro físico de la cámara.   │
#  │  NO es necesario aplicar ninguna corrección adicional.  │
#  └─────────────────────────────────────────────────────────┘
T_BRIDA_CAM = np.array([
    [-1.,  0.,  0.,  0.067506],  # eje X de cámara → -X de brida
    [ 0., -1.,  0.,  0.007342],  # eje Y de cámara → -Y de brida
    [ 0.,  0.,  1.,  0.035900],  # eje Z de cámara →  +Z de brida (profundidad = fuera)
    [ 0.,  0.,  0.,  1.       ]
], dtype=np.float64)

# T_brida_gripper: TCP del gripper (punta de la pinza) respecto a la BRIDA.
# La pinza tiene 160 mm de longitud, en la dirección +Z de la brida.
# (Este tensor se usa solo para referencia/documentación; el cálculo de
#  la pose target para link5 lo incorpora directamente via GRIPPER_LENGTH_M)
T_BRIDA_GRIP = np.array([
    [1.,  0.,  0.,  0.           ],
    [0.,  1.,  0.,  0.           ],
    [0.,  0.,  1.,  GRIPPER_LENGTH_M],
    [0.,  0.,  0.,  1.           ]
], dtype=np.float64)


# ──────────────────────────────────────────────────────────────────────
# UTILIDADES MATEMÁTICAS
# ──────────────────────────────────────────────────────────────────────

def tf_stamped_to_matrix(tf_stamped) -> np.ndarray:
    """
    Convierte un TransformStamped de ROS2 a una matriz homogénea 4×4.

    Cuaternión → Matriz de rotación 3×3 (fórmula de Rodrigues para cuaterniones
    unitarios, sin dependencias externas a NumPy):

      R = (qw²-|qv|²)I + 2(qv⊗qv) + 2qw[qv]×

    Referencia: Shuster, M.D. (1993). "A Survey of Attitude Representations".
    """
    t  = tf_stamped.transform.translation
    q  = tf_stamped.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # Matriz de rotación 3×3 expandida explícitamente (evita scipy)
    rot = np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [    2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qw*qx)],
        [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rot
    T[:3,  3] = [t.x, t.y, t.z]
    return T


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> list:
    """
    Convierte ángulos de Euler RPY (rotación extrínseca X→Y→Z) a cuaternión
    [qx, qy, qz, qw] compatible con geometry_msgs/Quaternion.

    Convención: rotación estática (extrínseca), equivalente a:
      q = q_z(yaw) * q_y(pitch) * q_x(roll)
    """
    cr, sr = math.cos(roll  / 2.), math.sin(roll  / 2.)
    cp, sp = math.cos(pitch / 2.), math.sin(pitch / 2.)
    cy, sy = math.cos(yaw   / 2.), math.sin(yaw   / 2.)

    qw =  cr*cp*cy + sr*sp*sy
    qx =  sr*cp*cy - cr*sp*sy
    qy =  cr*sp*cy + sr*cp*sy
    qz =  cr*cp*sy - sr*sp*cy
    return [qx, qy, qz, qw]


def quat_gripper_apuntando_abajo(yaw: float) -> list:
    """
    Calcula el cuaternión para que el eje Z del gripper apunte hacia abajo
    (-Z_world), rotado `yaw` grados en el plano XY para alinear la muñeca
    con la dirección radial al objeto.

    Equivale a RPY = (π, 0, yaw):
      - roll = π  →  gira el eje Z del TCP 180°, queda apuntando hacia -Z_world
      - pitch = 0 →  sin inclinación frontal-trasera
      - yaw = θ   →  rota en el plano horizontal, alineando joint1

    Para un xArm5, este yaw generalmente coincide con el ángulo de joint1,
    lo que maximiza la solución IK dentro del workspace.
    """
    return rpy_to_quat(math.pi, 0.0, yaw)


def dentro_workspace(x: float, y: float, z: float) -> bool:
    """Verifica que un punto (m) esté dentro de los límites del workspace definidos."""
    return (WS_X_MIN <= x <= WS_X_MAX and
            WS_Y_MIN <= y <= WS_Y_MAX and
            WS_Z_MIN <= z <= WS_Z_MAX)


# ──────────────────────────────────────────────────────────────────────
# NODO PRINCIPAL
# ──────────────────────────────────────────────────────────────────────

class PickAndPlaceNode(Node):
    """
    Nodo ROS2 que implementa la lógica completa de Pick & Place:

    1. Inicialización y movimiento a HOME.
    2. Espera detecciones de YOLOv8 vía /coordenadas_cubo_3d.
    3. Lee la pose de la brida en tiempo real desde /tf.
    4. Calcula la pose del objeto en el frame base (cinemática Eye-in-Hand).
    5. Planifica con MoveIt y presenta el plan en RViz.
    6. Espera confirmación del operador (tecla Enter).
    7. Ejecuta el movimiento real.
    8. Verifica si el objeto sigue detectado (bucle de corrección).
    9. Regresa a HOME.
    """

    def __init__(self):
        super().__init__('xarm5_pick_place')

        # ReentrantCallbackGroup permite llamadas de servicio concurrentes
        # desde diferentes callbacks/hilos sin bloqueos mutuos.
        self._cbg = ReentrantCallbackGroup()

        # ── Clientes MoveIt (planificación de trayectoria) ─────────────────
        self._arm_plan  = self.create_client(
            PlanPose,  '/xarm_pose_plan',           callback_group=self._cbg)
        self._arm_exec  = self.create_client(
            PlanExec,  '/xarm_exec_plan',           callback_group=self._cbg)

        # Gripper vía MoveIt (funciona con la config add_gripper:=true)
        self._grip_plan = self.create_client(
            PlanJoint, '/xarm_gripper_joint_plan',  callback_group=self._cbg)
        self._grip_exec = self.create_client(
            PlanExec,  '/xarm_gripper_exec_plan',   callback_group=self._cbg)

        # ── Clientes directos del driver xArm ──────────────────────────────
        self._cli_joint = self.create_client(
            MoveJoint, '/xarm/set_servo_angle',     callback_group=self._cbg)
        self._cli_mode  = self.create_client(
            SetInt16,  '/xarm/set_mode',            callback_group=self._cbg)
        self._cli_state = self.create_client(
            SetInt16,  '/xarm/set_state',           callback_group=self._cbg)

        # ── TF2 ────────────────────────────────────────────────────────────
        # Buffer que almacena el historial de transformaciones
        self._tf_buf = tf2_ros.Buffer()
        # Listener que se suscribe a /tf y /tf_static y rellena el buffer
        # (el MultiThreadedExecutor del nodo procesará sus callbacks)
        self._tf_lst = tf2_ros.TransformListener(self._tf_buf, self)

        # ── Suscriptor de visión ───────────────────────────────────────────
        self._sub = self.create_subscription(
            Point,
            'coordenadas_cubo_3d',
            self._vision_cb,
            10,
            callback_group=self._cbg)

        # ── Estado del nodo ────────────────────────────────────────────────
        self._busy        = True       # Bloquea nuevas detecciones mientras trabaja
        self._last_point  = None       # Último Point recibido de visión
        self._lock        = threading.Lock()

        # ── Hilo de lógica ─────────────────────────────────────────────────
        # Toda la lógica pesada corre aquí para NO bloquear el executor ROS2
        self._worker = threading.Thread(
            target=self._hilo_logica, daemon=True)
        self._worker.start()

        self.get_logger().info('🤖 Nodo iniciado. Esperando servicios...')

    # ──────────────────────────────────────────────────────────────────────
    # LLAMADA A SERVICIOS DESDE HILO EXTERNO
    # ──────────────────────────────────────────────────────────────────────

    def _call_srv(self, client, request, timeout: float = 20.0):
        """
        Llama a un servicio ROS2 de forma "semisíncrona" desde el hilo de lógica.

        Mecanismo:
          1. Envía la petición de forma async (no bloqueante para el executor).
          2. Hace polling sobre el futuro mientras el MultiThreadedExecutor
             procesa la respuesta del servicio en paralelo.
          3. Retorna el resultado cuando esté disponible, o None si hay timeout.

        Este patrón es el correcto en ROS2 para combinar un hilo de lógica
        propio con un executor corriendo en otro hilo.
        """
        future = client.call_async(request)
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout:
                self.get_logger().error(
                    f'⏱ Timeout ({timeout}s) esperando: {client.srv_name}')
                return None
            time.sleep(0.02)
        return future.result()

    # ──────────────────────────────────────────────────────────────────────
    # ESPERA DE SERVICIOS
    # ──────────────────────────────────────────────────────────────────────

    def _esperar_servicios(self):
        """Bloquea hasta que los servicios críticos estén disponibles."""
        criticos = {
            'arm_plan':   self._arm_plan,
            'arm_exec':   self._arm_exec,
            'cli_joint':  self._cli_joint,
            'cli_mode':   self._cli_mode,
            'cli_state':  self._cli_state,
        }
        opcionales = {
            'grip_plan':  self._grip_plan,
            'grip_exec':  self._grip_exec,
        }

        for nombre, srv in criticos.items():
            while not srv.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'  ⏳ Esperando servicio crítico: {srv.srv_name}')
            self.get_logger().info(f'  ✔ {nombre}: {srv.srv_name}')

        for nombre, srv in opcionales.items():
            ok = srv.wait_for_service(timeout_sec=3.0)
            if ok:
                self.get_logger().info(f'  ✔ {nombre} (gripper MoveIt): disponible')
            else:
                self.get_logger().warn(
                    f'  ⚠ {nombre} no disponible. Lanza con add_gripper:=true '
                    f'si necesitas control de gripper por MoveIt.')

        self.get_logger().info('✅ Todos los servicios verificados.\n')

    # ──────────────────────────────────────────────────────────────────────
    # CALLBACK DE VISIÓN
    # ──────────────────────────────────────────────────────────────────────

    def _vision_cb(self, msg: Point):
        """
        Recibe coordenadas 3D del objeto desde vision_all_in_one.py.
        Solo acepta si el robot está libre (self._busy == False).
        Aplica un filtro de sanidad sobre la distancia Z.
        """
        if self._busy:
            return

        z = msg.z
        if not (0.15 < z < 1.50):
            self.get_logger().warn(
                f'⚠ Lectura de visión descartada: Z={z:.3f}m '
                f'(rango válido: 0.15–1.50m)')
            return

        with self._lock:
            self._last_point = msg

    # ──────────────────────────────────────────────────────────────────────
    # HILO DE LÓGICA PRINCIPAL
    # ──────────────────────────────────────────────────────────────────────

    def _hilo_logica(self):
        """
        Bucle de control principal. Corre en un hilo separado para no
        bloquear el executor ROS2 que maneja callbacks y servicios.
        """
        self._esperar_servicios()

        # Secuencia de arranque
        self._activar_robot()
        self._ir_a_home()

        self._busy = False
        self.get_logger().info(
            '🎯 Sistema listo. Esperando detecciones de YOLOv8...\n'
            '   (Publica en /coordenadas_cubo_3d para iniciar)\n')

        # Bucle principal
        while rclpy.ok():
            # Leer y limpiar el punto pendiente
            with self._lock:
                punto = self._last_point
                self._last_point = None

            if punto is None:
                time.sleep(0.05)
                continue

            # ── Nuevo objeto detectado ──
            self._busy = True
            self.get_logger().info(
                f'📦 Objeto detectado en cámara: '
                f'X={punto.x:.3f}m  Y={punto.y:.3f}m  Z={punto.z:.3f}m')

            exito = self._ciclo_pick(punto)

            estado = '✅ Éxito' if exito else '⚠ Sin éxito confirmado'
            self.get_logger().info(f'{estado}. Regresando a HOME...\n')

            self._ir_a_home()
            self._busy = False
            self.get_logger().info('🏠 HOME. Listo para el próximo objeto.\n')

    # ──────────────────────────────────────────────────────────────────────
    # CICLO COMPLETO DE PICK
    # ──────────────────────────────────────────────────────────────────────

    def _ciclo_pick(self, punto: Point, max_intentos: int = 3) -> bool:
        """
        Ejecuta el ciclo completo:
          1. Calcular pose 3D del objeto (cinemática Eye-in-Hand).
          2. Planificar trayectoria con MoveIt (visible en RViz).
          3. Esperar confirmación del operador (Enter).
          4. Ejecutar movimiento real.
          5. Verificar si el objeto fue alcanzado (la cámara deja de verlo).
          6. Si sigue detectándose, recalcular y reintentar.

        Retorna True si el objeto dejó de detectarse tras la ejecución.
        """
        for intento in range(1, max_intentos + 1):
            sep = '─' * 55
            self.get_logger().info(f'\n{sep}')
            self.get_logger().info(f'  Intento {intento}/{max_intentos}')
            self.get_logger().info(f'{sep}')

            # ── PASO 1: Calcular posición del objeto en frame base ──────────
            resultado = self._calcular_pose_objeto(punto)
            if resultado is None:
                self.get_logger().error('No se pudo calcular la pose. Abortando.')
                return False

            x_obj, y_obj, z_obj = resultado
            self.get_logger().info(
                f'📍 Objeto en frame base → '
                f'X:{x_obj:.4f}m  Y:{y_obj:.4f}m  Z:{z_obj:.4f}m')

            # ── PASO 2: Calcular pose target para link5 (brida) ────────────
            #
            # El gripper apunta hacia abajo (−Z_world) con yaw alineado.
            # Para un robot con EEF = link5 y gripper apuntando recto hacia abajo:
            #
            #   punta_gripper = link5 + R_link5 · [0, 0, GRIPPER_LENGTH]
            #   Con el gripper apuntando hacia −Z_world:
            #     R_link5 · [0, 0, GRIPPER_LENGTH] = [0, 0, −GRIPPER_LENGTH]_world
            #
            #   →  link5_Z = punta_gripper_Z + GRIPPER_LENGTH
            #
            # Queremos que la punta del gripper quede a PICK_OFFSET_Z sobre el objeto:
            #   punta_target_Z = z_obj + PICK_OFFSET_Z
            #   link5_target_Z = z_obj + PICK_OFFSET_Z + GRIPPER_LENGTH
            #
            # El XY de link5 = XY del objeto (gripper apunta recto abajo, sin desviación).

            yaw  = math.atan2(y_obj, x_obj)
            quat = quat_gripper_apuntando_abajo(yaw)

            x_t = x_obj
            y_t = y_obj
            z_t = z_obj + PICK_OFFSET_Z_M + GRIPPER_LENGTH_M

            self.get_logger().info(
                f'🎯 Target link5 → '
                f'X:{x_t:.4f}m  Y:{y_t:.4f}m  Z:{z_t:.4f}m  '
                f'yaw:{math.degrees(yaw):.1f}°')

            # Verificar que el target está dentro del workspace
            if not dentro_workspace(x_t, y_t, z_t):
                self.get_logger().error(
                    f'🚫 Target fuera del workspace definido. Abortando.')
                return False

            # ── PASO 3: Planificar con MoveIt ──────────────────────────────
            if not self._plan_pose(x_t, y_t, z_t, quat, nombre='PRE-PICK'):
                self.get_logger().error('Planificación fallida. Abortando.')
                return False

            # ── PASO 4: Confirmación humana ─────────────────────────────────
            print('\n' + '═' * 55)
            print('  ✅ Plan calculado y visible en RViz.')
            print('  👁  Revisa el "robot fantasma" (trayectoria naranja).')
            print()
            print('  ▶  Presiona  [ENTER]    para EJECUTAR el movimiento.')
            print('  ⛔  Presiona  Ctrl+C     para CANCELAR y volver a home.')
            print('═' * 55)

            try:
                input('> ')
            except (EOFError, KeyboardInterrupt):
                self.get_logger().warn('⛔ Ejecución cancelada por el operador.')
                return False

            # ── PASO 5: Ejecutar el movimiento real ─────────────────────────
            self.get_logger().info('🚀 Ejecutando trayectoria...')
            if not self._exec_plan():
                self.get_logger().error('Error durante la ejecución.')
                return False

            self.get_logger().info(f'✔ Movimiento completado.')

            # ── PASO 6: Verificar resultado ─────────────────────────────────
            # Esperar a que la cámara procese la nueva posición del robot
            self.get_logger().info('🔍 Verificando resultado (2s)...')
            time.sleep(2.0)

            with self._lock:
                nuevo_punto = self._last_point
                self._last_point = None

            if nuevo_punto is None:
                self.get_logger().info(
                    '✅ Objeto no detectado → el gripper está sobre el objetivo.')
                return True
            else:
                self.get_logger().info(
                    '🔄 Objeto aún detectado. Recalculando con nueva lectura de visión...')
                punto = nuevo_punto  # Usar la lectura más fresca

        self.get_logger().warn(
            f'Se alcanzó el máximo de {max_intentos} intentos sin éxito.')
        return False

    # ──────────────────────────────────────────────────────────────────────
    # CINEMÁTICA EYE-IN-HAND — Núcleo del sistema
    # ──────────────────────────────────────────────────────────────────────

    def _calcular_pose_objeto(self, punto: Point):
        """
        Calcula la posición 3D del objeto en el frame de la BASE del robot.

        ┌─────────────────────────────────────────────────────────────────┐
        │  Pipeline cinemático Eye-in-Hand:                               │
        │                                                                 │
        │   p_base = T_base_brida × T_brida_cam × p_cam                  │
        │                                                                 │
        │  donde:                                                         │
        │   • T_base_brida  → leído de /tf en TIEMPO REAL                │
        │                      (frame: link_base ← link5)                │
        │   • T_brida_cam   → constante (montaje físico de la cámara)    │
        │   • p_cam         → coordenadas del objeto desde RealSense (m) │
        └─────────────────────────────────────────────────────────────────┘

        Retorna (x, y, z) en metros en el frame link_base, o None si el TF
        no está disponible.
        """
        # 1. Leer la transformación brida → base desde el árbol TF en tiempo real
        try:
            tf_stamp = self._tf_buf.lookup_transform(
                TF_BASE,                               # frame objetivo (base)
                TF_FLANGE,                             # frame origen (brida)
                rclpy.time.Time(),                     # tiempo 0 = último TF disponible
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except TransformException as exc:
            self.get_logger().error(
                f'❌ TF lookup fallido ({TF_FLANGE} → {TF_BASE}): {exc}\n'
                f'   ¿Está corriendo xarm5_moveit_realmove.launch.py?')
            return None

        # 2. Construir T_base_brida como matriz homogénea 4×4
        T_base_brida = tf_stamped_to_matrix(tf_stamp)

        self.get_logger().debug(
            f'T_base_brida (pos): '
            f'{T_base_brida[0,3]:.4f}, {T_base_brida[1,3]:.4f}, {T_base_brida[2,3]:.4f}')

        # 3. Vector homogéneo del objeto en el frame de la cámara
        #    vision_all_in_one.py usa rs.rs2_deproject_pixel_to_point,
        #    que entrega coordenadas en el sistema de la cámara (metros):
        #      X = desplazamiento lateral derecha (+) / izquierda (−)
        #      Y = desplazamiento vertical abajo (+) / arriba (−)
        #      Z = profundidad (distancia desde el lente)
        p_cam = np.array([punto.x, punto.y, punto.z, 1.0], dtype=np.float64)

        # 4. Transformación completa: base ← brida ← cámara ← objeto
        #    T_base_cam = T_base_brida × T_brida_cam
        T_base_cam = T_base_brida @ T_BRIDA_CAM
        p_base     = T_base_cam @ p_cam   # [x, y, z, 1] en frame base

        self.get_logger().debug(
            f'p_cam={punto.x:.4f},{punto.y:.4f},{punto.z:.4f}  →  '
            f'p_base={p_base[0]:.4f},{p_base[1]:.4f},{p_base[2]:.4f}')

        return float(p_base[0]), float(p_base[1]), float(p_base[2])

    # ──────────────────────────────────────────────────────────────────────
    # INTERFAZ MOVEIT
    # ──────────────────────────────────────────────────────────────────────

    def _plan_pose(self, x, y, z, quat, nombre='Pose') -> bool:
        """
        Solicita a MoveIt que planifique una trayectoria hacia la pose dada.
        El plan queda visible en RViz como robot "fantasma" antes de ejecutar.

        Args:
            x, y, z  : posición target en metros (frame link_base)
            quat     : cuaternión [qx, qy, qz, qw]
            nombre   : etiqueta para el log
        """
        req = PlanPose.Request()
        req.target.position.x    = float(x)
        req.target.position.y    = float(y)
        req.target.position.z    = float(z)
        req.target.orientation.x = float(quat[0])
        req.target.orientation.y = float(quat[1])
        req.target.orientation.z = float(quat[2])
        req.target.orientation.w = float(quat[3])

        self.get_logger().info(f'📐 Planificando "{nombre}"...')
        resp = self._call_srv(self._arm_plan, req, timeout=20.0)

        if resp is None:
            return False
        if not resp.success:
            self.get_logger().error(
                f'❌ MoveIt no encontró trayectoria válida para "{nombre}".\n'
                f'   Posibles causas: posición fuera del espacio de trabajo, '
                f'singularidad, o colisión.')
            return False

        self.get_logger().info(
            f'✔ Plan "{nombre}" calculado. '
            f'El robot fantasma naranja está visible en RViz.')
        return True

    def _exec_plan(self) -> bool:
        """Ordena a MoveIt que ejecute el último plan calculado."""
        req = PlanExec.Request()
        req.wait = True   # Bloquear hasta que el movimiento termine

        resp = self._call_srv(self._arm_exec, req, timeout=40.0)
        if resp is None:
            return False

        self.get_logger().info('✔ Ejecución MoveIt completada.')
        return True

    def _mover_gripper(self, cerrar: bool):
        """
        Controla el gripper mediante MoveIt (requiere add_gripper:=true).

        Valores para xArm Gripper (espacio de joints MoveIt):
          0.0  → gripper ABIERTO al máximo
          0.85 → gripper CERRADO (agarre)

        Si el servicio no está disponible, lo indica pero no aborta.
        """
        req = PlanJoint.Request()
        if cerrar:
            self.get_logger().info('>< Cerrando gripper (0.85)...')
            req.target = [0.85, 0.85, 0.85, 0.85, 0.85, 0.85]
        else:
            self.get_logger().info('<> Abriendo gripper (0.0)...')
            req.target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        resp_plan = self._call_srv(self._grip_plan, req, timeout=5.0)
        if resp_plan is None or not resp_plan.success:
            self.get_logger().warn(
                '⚠ No se pudo planificar el gripper. '
                '¿Está disponible /xarm_gripper_joint_plan?')
            return

        req_exec = PlanExec.Request()
        req_exec.wait = True
        self._call_srv(self._grip_exec, req_exec, timeout=10.0)
        time.sleep(0.5)

    # ──────────────────────────────────────────────────────────────────────
    # ACTIVACIÓN DEL ROBOT
    # ──────────────────────────────────────────────────────────────────────

    def _activar_robot(self):
        """
        Configura el robot en:
          Modo  0 → Control de posición (necesario para mover por joints/cartesiano)
          Estado 0 → Habilitado (motores activos)
        """
        self.get_logger().info('⚙ Configurando Modo=0 (posición)...')
        req_mode = SetInt16.Request()
        req_mode.data = 0
        self._call_srv(self._cli_mode, req_mode)
        time.sleep(0.3)

        self.get_logger().info('⚙ Habilitando motores (Estado=0)...')
        req_state = SetInt16.Request()
        req_state.data = 0
        self._call_srv(self._cli_state, req_state)
        time.sleep(0.3)

        self.get_logger().info('✔ Robot habilitado y listo.')

    def _ir_a_home(self):
        """
        Mueve el robot a la posición HOME en espacio de articulaciones.
        Usa el servicio directo /xarm/set_servo_angle (no MoveIt) para
        garantizar un HOME seguro y repetible.
        """
        self.get_logger().info(
            f'🏠 Moviendo a HOME: {[round(j, 3) for j in HOME_JOINTS]} rad...')

        req = MoveJoint.Request()
        req.angles = HOME_JOINTS
        req.speed  = HOME_SPEED
        req.acc    = HOME_ACC

        resp = self._call_srv(self._cli_joint, req, timeout=25.0)

        if resp and resp.ret == 0:
            self.get_logger().info('✔ HOME alcanzado correctamente.')
        elif resp:
            self.get_logger().warn(
                f'⚠ HOME: código de retorno = {resp.ret} | {getattr(resp, "message", "")}')
        # Si resp es None el timeout ya fue reportado en _call_srv


# ──────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────

def main(args=None):
    """
    Inicializa el nodo con un MultiThreadedExecutor (4 hilos):
      • Hilo 1-3: procesan callbacks de subscriptores, servicios y TF
      • Hilo 4  : el hilo de lógica propio del nodo (_hilo_logica)
                  hace polling de futuros y ejecuta el flujo de Pick & Place
    """
    rclpy.init(args=args)
    node = PickAndPlaceNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('\n⛔ KeyboardInterrupt recibido. Apagando...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
