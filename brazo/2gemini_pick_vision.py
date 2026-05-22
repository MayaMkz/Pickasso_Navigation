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

# ──────────────────────────────────────────────────────────────────────
# CONFIGURACIÓN GLOBAL
# ──────────────────────────────────────────────────────────────────────
TF_BASE   = 'link_base'
TF_FLANGE = 'link5'

HOME_JOINTS = [0.0, -0.349066, -1.13446, 1.48353, 0.0]

CAMERA_Z_OFFSET_M = 0.25  # 25 cm de altura para centrar la cámara y observar
PICK_OFFSET_Z_M   = 0.10  # 10 cm de seguridad antes del agarre
GRIPPER_LENGTH_M  = 0.160 # 160 mm de longitud de la pinza de 2 dedos

WS_X_MIN, WS_X_MAX =  0.05,  0.70
WS_Y_MIN, WS_Y_MAX = -0.50,  0.50
WS_Z_MIN, WS_Z_MAX = -0.05,  0.70

# Transformación Brida a Cámara (Rotada 180 grados en el montaje físico)
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
    """Convierte cuaternión [x, y, z, w] a matriz de rotación 3x3"""
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
    def __init__(self):
        super().__init__('xarm5_pick_place')
        self._cbg = ReentrantCallbackGroup()

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
        self.get_logger().info('🎯 Sistema listo. Esperando detecciones...\n')

        while rclpy.ok():
            with self._lock:
                punto = self._last_point
                self._last_point = None

            if punto is None:
                time.sleep(0.05)
                continue

            self._busy = True
            self.get_logger().info(f'📦 Dado detectado. Iniciando secuencia Pick&Place.')

            exito = self._ciclo_pick(punto)
            self._ir_a_home()
            self._busy = False
            self.get_logger().info('🏠 HOME. Listo para el próximo objetivo.\n')

    def _ciclo_pick(self, punto: Point, max_intentos: int = 3) -> bool:
        for intento in range(1, max_intentos + 1):
            resultado = self._calcular_pose_objeto(punto)
            if not resultado: return False

            x_obj, y_obj, z_obj = resultado
            yaw  = math.atan2(y_obj, x_obj)
            quat = quat_gripper_apuntando_abajo(yaw)
            
            # Matriz de rotación de la brida para compensar offsets
            R_target = quat_to_rot_matrix(quat)

            # ──────────────────────────────────────────────────────────
            # FASE 1: CENTRAR CÁMARA
            # ──────────────────────────────────────────────────────────
            p_cam_target = np.array([x_obj, y_obj, z_obj + CAMERA_Z_OFFSET_M])
            t_cam = np.array([0.067506, 0.007342, 0.035900])
            p_brida_cam = p_cam_target - R_target @ t_cam

            self.get_logger().info('📸 FASE 1: Calculando centrado de cámara...')
            if not self._plan_pose(p_brida_cam[0], p_brida_cam[1], p_brida_cam[2], quat, 'CENTRAR_CAMARA'):
                return False

            print('\n═' * 55)
            print('  ✅ Plan calculado: La cámara se centrará sobre el cubo.')
            print('  ▶  Presiona [ENTER] para ejecutar TODA LA SECUENCIA.')
            print('═' * 55)
            try: input('> ')
            except: return False

            self._exec_plan()
            time.sleep(0.5)

            # ──────────────────────────────────────────────────────────
            # FASE 2: CENTRAR GRIPPER Y BAJAR (PRE-PICK)
            # ──────────────────────────────────────────────────────────
            self.get_logger().info('>< FASE 2: Desplazando para alinear gripper (Pre-Pick)...')
            p_grip_pre = np.array([x_obj, y_obj, z_obj + PICK_OFFSET_Z_M])
            t_grip = np.array([0.0, 0.0, GRIPPER_LENGTH_M])
            p_brida_pre = p_grip_pre - R_target @ t_grip

            if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'PRE-PICK'):
                self._exec_plan()

            # ──────────────────────────────────────────────────────────
            # FASE 3: BAJAR AL DADO (PICK)
            # ──────────────────────────────────────────────────────────
            self.get_logger().info('👇 FASE 3: Bajando al cubo...')
            p_grip_pick = np.array([x_obj, y_obj, z_obj])
            p_brida_pick = p_grip_pick - R_target @ t_grip

            if self._plan_pose(p_brida_pick[0], p_brida_pick[1], p_brida_pick[2], quat, 'PICK'):
                self._exec_plan()

            # ──────────────────────────────────────────────────────────
            # FASE 4: CERRAR GRIPPER
            # ──────────────────────────────────────────────────────────
            self._mover_gripper(cerrar=True)

            # ──────────────────────────────────────────────────────────
            # FASE 5: SUBIR AL OFFSET
            # ──────────────────────────────────────────────────────────
            self.get_logger().info('👆 FASE 5: Subiendo a posición segura...')
            if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'POST-PICK'):
                self._exec_plan()

            # Verificación visual final
            time.sleep(2.0)
            with self._lock:
                nuevo_punto = self._last_point
                self._last_point = None

            if nuevo_punto is None:
                self.get_logger().info('✅ Verificación visual: El cubo ha sido retirado.')
                return True
            else:
                punto = nuevo_punto

        return False

    def _calcular_pose_objeto(self, punto: Point):
        try:
            tf_stamp = self._tf_buf.lookup_transform(
                TF_BASE, TF_FLANGE, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except TransformException as exc:
            self.get_logger().error(f'❌ TF lookup fallido: {exc}')
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
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
