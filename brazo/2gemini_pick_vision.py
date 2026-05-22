#!/usr/bin/env python3
"""
pick_and_place.py — Sistema Pick & Place para xArm5 con visión Eye-in-Hand
===========================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from xarm_msgs.srv import PlanPose, PlanExec, PlanJoint

import tf2_ros
from tf2_ros import TransformException

import numpy as np
import math
import threading
import time
import sys
import select
import tty
import termios
import os

# ──────────────────────────────────────────────────────────────────────
# LECTOR DE TECLADO ASÍNCRONO (Paro de Emergencia y Confirmación)
# ──────────────────────────────────────────────────────────────────────
class TecladoListener:
    def __init__(self):
        self.enter_presionado = False
        self.corriendo = True

    def escuchar(self):
        """Hilo en segundo plano para no bloquear a ROS2 con input()"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.corriendo:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    tecla = sys.stdin.read(1)
                    if tecla.lower() == 'q':
                        print("\r\n\r\n [PARO DE EMERGENCIA] Tecla 'q' detectada. Abortando al instante... \r\n")
                        os._exit(1)
                    elif tecla in ('\n', '\r'):
                        self.enter_presionado = True
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# ──────────────────────────────────────────────────────────────────────
# CONFIGURACIÓN GLOBAL
# ──────────────────────────────────────────────────────────────────────
TF_BASE   = 'link_base'
TF_FLANGE = 'link5'

HOME_JOINTS = [0.0, -0.349066, -1.13446, 1.48353, 0.0]

CAMERA_Z_OFFSET_M = 0.25  # Altura para observar y seguir el cubo
PICK_OFFSET_Z_M   = 0.10  # Altura de seguridad para la pinza antes de bajar
GRIPPER_LENGTH_M  = 0.160 # Longitud de la pinza de 2 dedos

WS_X_MIN, WS_X_MAX =  0.05,  0.70
WS_Y_MIN, WS_Y_MAX = -0.50,  0.50
WS_Z_MIN, WS_Z_MAX = -0.05,  0.70

# Transformación de la brida a la cámara (Rotación de 180 grados compensada)
T_BRIDA_CAM = np.array([
    [-1.,  0.,  0.,  0.067506],
    [ 0., -1.,  0.,  0.007342],
    [ 0.,  0.,  1.,  0.035900],
    [ 0.,  0.,  0.,  1.       ]
], dtype=np.float64)

# ──────────────────────────────────────────────────────────────────────
# UTILIDADES MATEMÁTICAS
# ──────────────────────────────────────────────────────────────────────
def tf_stamped_to_matrix(tf_stamped) -> np.ndarray:
    t  = tf_stamped.transform.translation
    q  = tf_stamped.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    rot = np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [    2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qw*qx)],
        [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rot
    T[:3,  3] = [t.x, t.y, t.z]
    return T

def quat_to_rot_matrix(q: list) -> np.ndarray:
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [    2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qw*qx)],
        [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
    ], dtype=np.float64)

def rpy_to_quat(roll: float, pitch: float, yaw: float) -> list:
    cr, sr = math.cos(roll  / 2.), math.sin(roll  / 2.)
    cp, sp = math.cos(pitch / 2.), math.sin(pitch / 2.)
    cy, sy = math.cos(yaw   / 2.), math.sin(yaw   / 2.)

    qw =  cr*cp*cy + sr*sp*sy
    qx =  sr*cp*cy - cr*sp*sy
    qy =  cr*sp*cy + sr*cp*sy
    qz =  cr*cp*sy - sr*sp*cy
    return [qx, qy, qz, qw]

def quat_gripper_apuntando_abajo(yaw: float) -> list:
    return rpy_to_quat(math.pi, 0.0, yaw)

# ──────────────────────────────────────────────────────────────────────
# NODO PRINCIPAL
# ──────────────────────────────────────────────────────────────────────
class PickAndPlaceNode(Node):
    def __init__(self, listener):
        super().__init__('xarm5_pick_place')
        self._cbg = ReentrantCallbackGroup()
        self.teclado = listener

        self._arm_plan  = self.create_client(PlanPose,  '/xarm_pose_plan',           callback_group=self._cbg)
        self._arm_exec  = self.create_client(PlanExec,  '/xarm_exec_plan',           callback_group=self._cbg)
        self._arm_joint_plan = self.create_client(PlanJoint, '/xarm_joint_plan',     callback_group=self._cbg)
        self._grip_plan = self.create_client(PlanJoint, '/xarm_gripper_joint_plan',  callback_group=self._cbg)
        self._grip_exec = self.create_client(PlanExec,  '/xarm_gripper_exec_plan',   callback_group=self._cbg)

        self._tf_buf = tf2_ros.Buffer()
        self._tf_lst = tf2_ros.TransformListener(self._tf_buf, self)

        self._sub = self.create_subscription(
            Point, 'coordenadas_cubo_3d', self._vision_cb, 10, callback_group=self._cbg)

        self._busy        = True
        self._last_point  = None
        self._lock        = threading.Lock()

        self._worker = threading.Thread(target=self._hilo_logica, daemon=True)
        self._worker.start()

    def _call_srv(self, client, request, timeout: float = 20.0):
        future = client.call_async(request)
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout:
                return None
            time.sleep(0.02)
        return future.result()

    def _esperar_servicios(self):
        criticos = [self._arm_plan, self._arm_exec, self._arm_joint_plan]
        for srv in criticos:
            while not srv.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'⏳ Esperando servicio: {srv.srv_name}')
        self._grip_plan.wait_for_service(timeout_sec=3.0)
        self._grip_exec.wait_for_service(timeout_sec=3.0)
        self.get_logger().info('✅ Todos los servicios listos.\n')

    def _vision_cb(self, msg: Point):
        if self._busy: return
        with self._lock:
            self._last_point = msg

    def _hilo_logica(self):
        self._esperar_servicios()
        self._ir_a_home()

        self._busy = False
        print("\r\n🎯 Sistema listo. Esperando detecciones...\r\n")

        while rclpy.ok():
            with self._lock:
                punto = self._last_point
                self._last_point = None

            if punto is None:
                time.sleep(0.05)
                continue

            self._busy = True
            print("\r\n📦 Dado detectado. Iniciando modo de SEGUIMIENTO (Tracking).\r\n")
            
            exito = self._ciclo_tracking_y_pick(punto)
            
            self._ir_a_home()
            self._busy = False
            self.teclado.enter_presionado = False
            print("\r\n🏠 HOME. Listo para el próximo objetivo.\r\n")

    def _ciclo_tracking_y_pick(self, punto_inicial: Point) -> bool:
        """
        Bucle 1: Centra la cámara continuamente siguiendo el cubo.
        Sale del bucle cuando el usuario presiona Enter.
        Fase 2: Ejecuta el Pick con la pinza.
        """
        self.teclado.enter_presionado = False
        ultimo_punto_valido = punto_inicial

        print("═" * 55)
        print("  👁  SEGUIMIENTO ACTIVO. Mueve el cubo si lo deseas.")
        print("  ▶  Presiona [ENTER] en esta terminal para confirmar el Pick.")
        print("  ⛔  Presiona [q] para Paro de Emergencia.")
        print("═" * 55 + "\r\n")

        # ──────────────────────────────────────────────────────────
        # FASE 1: BUCLE DE SEGUIMIENTO (TRACKING) DE LA CÁMARA
        # ──────────────────────────────────────────────────────────
        while not self.teclado.enter_presionado and rclpy.ok():
            # Limpiamos buffers antiguos de imagen para tener la pose fresca
            with self._lock:
                punto_actual = self._last_point
                self._last_point = None

            punto_a_procesar = punto_actual if punto_actual else ultimo_punto_valido

            resultado = self._calcular_pose_objeto(punto_a_procesar)
            if not resultado:
                time.sleep(0.1)
                continue

            x_obj, y_obj, z_obj = resultado
            ultimo_punto_valido = punto_a_procesar # Guardamos por si falla la siguiente lectura

            yaw  = math.atan2(y_obj, x_obj)
            quat = quat_gripper_apuntando_abajo(yaw)
            R_target = quat_to_rot_matrix(quat)

            # Cálculo para centrar el LENTE DE LA CÁMARA
            p_cam_target = np.array([x_obj, y_obj, z_obj + CAMERA_Z_OFFSET_M])
            t_cam = np.array([0.067506, 0.007342, 0.035900])
            p_brida_cam = p_cam_target - R_target @ t_cam

            # Ejecutamos el centrado de la cámara. MoveIt es 'Stop and Go'.
            if self._plan_pose(p_brida_cam[0], p_brida_cam[1], p_brida_cam[2], quat, 'TRACKING_CAMARA'):
                self._exec_plan()
            
            # Pequeña pausa para permitir que la cámara capture una imagen estable post-movimiento
            time.sleep(0.3) 

        # ──────────────────────────────────────────────────────────
        # FASE 2: PICK CONFIRMADO
        # ──────────────────────────────────────────────────────────
        print("\r\n\r\n✅ [ENTER] detectado. Iniciando secuencia de Pick.\r\n")
        self.teclado.enter_presionado = False

        # Usamos la última coordenada calculada en el bucle para el Pick
        # Recalculamos la cinemática pero ahora para el GRIPPER (Pinza 2 dedos)
        p_grip_pre = np.array([x_obj, y_obj, z_obj + PICK_OFFSET_Z_M])
        t_grip = np.array([0.0, 0.0, GRIPPER_LENGTH_M])
        p_brida_pre = p_grip_pre - R_target @ t_grip

        print('>< Desplazando para alinear gripper (Pre-Pick)...')
        if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'PRE-PICK'):
            self._exec_plan()

        print('👇 Bajando al cubo...')
        p_grip_pick = np.array([x_obj, y_obj, z_obj])
        p_brida_pick = p_grip_pick - R_target @ t_grip

        if self._plan_pose(p_brida_pick[0], p_brida_pick[1], p_brida_pick[2], quat, 'PICK'):
            self._exec_plan()

        self._mover_gripper(cerrar=True)

        print('👆 Subiendo a posición segura...')
        if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'POST-PICK'):
            self._exec_plan()

        # Verificación visual final
        time.sleep(2.0)
        with self._lock:
            nuevo_punto = self._last_point
            self._last_point = None

        if nuevo_punto is None:
            print('✅ Verificación visual: El cubo ha sido agarrado y retirado de la vista.')
            return True
        
        return False

    def _calcular_pose_objeto(self, punto: Point):
        try:
            tf_stamp = self._tf_buf.lookup_transform(
                TF_BASE, TF_FLANGE, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except TransformException as exc:
            return None

        T_base_brida = tf_stamped_to_matrix(tf_stamp)
        p_cam = np.array([punto.x, punto.y, punto.z, 1.0], dtype=np.float64)
        T_base_cam = T_base_brida @ T_BRIDA_CAM
        p_base     = T_base_cam @ p_cam   

        return float(p_base[0]), float(p_base[1]), float(p_base[2])

    def _plan_pose(self, x, y, z, quat, nombre='Pose') -> bool:
        req = PlanPose.Request()
        req.target.position.x, req.target.position.y, req.target.position.z = float(x), float(y), float(z)
        req.target.orientation.x, req.target.orientation.y = float(quat[0]), float(quat[1])
        req.target.orientation.z, req.target.orientation.w = float(quat[2]), float(quat[3])
        resp = self._call_srv(self._arm_plan, req, timeout=20.0)
        return resp is not None and resp.success

    def _exec_plan(self) -> bool:
        req = PlanExec.Request()
        req.wait = True
        resp = self._call_srv(self._arm_exec, req, timeout=40.0)
        return resp is not None

    def _mover_gripper(self, cerrar: bool):
        req = PlanJoint.Request()
        req.target = [0.85]*6 if cerrar else [0.0]*6
        resp_plan = self._call_srv(self._grip_plan, req, timeout=5.0)
        if resp_plan and resp_plan.success:
            req_exec = PlanExec.Request()
            req_exec.wait = True
            self._call_srv(self._grip_exec, req_exec, timeout=10.0)
            time.sleep(0.5)

    def _ir_a_home(self):
        req = PlanJoint.Request()
        req.target = HOME_JOINTS
        resp = self._call_srv(self._arm_joint_plan, req, timeout=25.0)
        if resp and resp.success:
            self._exec_plan()
            self._mover_gripper(cerrar=False)

def main(args=None):
    # Inicializamos el lector de teclado asíncrono
    teclado = TecladoListener()
    hilo_teclado = threading.Thread(target=teclado.escuchar, daemon=True)
    hilo_teclado.start()

    rclpy.init(args=args)
    node = PickAndPlaceNode(teclado)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try: 
        executor.spin()
    except KeyboardInterrupt: 
        pass
    finally:
        teclado.corriendo = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
