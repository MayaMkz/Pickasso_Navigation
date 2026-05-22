#!/usr/bin/env python3
"""
pick_and_place.py — Sistema Pick & Place para xArm5 con visión Eye-in-Hand
===========================================================================
Prerrequisitos (terminales separadas, en orden):

  Terminal 1 — Nodo del robot real + MoveIt + RViz (SIN GAZEBO):
    ros2 launch xarm_planner xarm5_planner_realmove.launch.py robot_ip:=192.168.1.234 add_gripper:=true

  Terminal 2 — Nodo de visión:
    ros2 run logica_almacen vision_all_in_one

  Terminal 3 — Este nodo:
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

# Posición HOME en espacio de articulaciones [rad]
HOME_JOINTS = [0.0, -0.349066, -1.13446, 1.48353, 0.0]

PICK_OFFSET_Z_M = 0.10    # 10 cm
GRIPPER_LENGTH_M = 0.160  # 160 mm

WS_X_MIN, WS_X_MAX =  0.05,  0.70
WS_Y_MIN, WS_Y_MAX = -0.50,  0.50
WS_Z_MIN, WS_Z_MAX = -0.05,  0.70

# ──────────────────────────────────────────────────────────────────────
# MATRICES DE TRANSFORMACIÓN FIJAS (en metros)
# ──────────────────────────────────────────────────────────────────────
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

def dentro_workspace(x: float, y: float, z: float) -> bool:
    return (WS_X_MIN <= x <= WS_X_MAX and
            WS_Y_MIN <= y <= WS_Y_MAX and
            WS_Z_MIN <= z <= WS_Z_MAX)

# ──────────────────────────────────────────────────────────────────────
# NODO PRINCIPAL
# ──────────────────────────────────────────────────────────────────────
class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('xarm5_pick_place')
        self._cbg = ReentrantCallbackGroup()

        # Clientes MoveIt (eliminamos servicios directos de hardware para evitar bloqueos)
        self._arm_plan  = self.create_client(PlanPose,  '/xarm_pose_plan',           callback_group=self._cbg)
        self._arm_exec  = self.create_client(PlanExec,  '/xarm_exec_plan',           callback_group=self._cbg)
        self._arm_joint_plan = self.create_client(PlanJoint, '/xarm_joint_plan',     callback_group=self._cbg)
        self._grip_plan = self.create_client(PlanJoint, '/xarm_gripper_joint_plan',  callback_group=self._cbg)
        self._grip_exec = self.create_client(PlanExec,  '/xarm_gripper_exec_plan',   callback_group=self._cbg)

        # TF2
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lst = tf2_ros.TransformListener(self._tf_buf, self)

        # Suscriptor
        self._sub = self.create_subscription(
            Point, 'coordenadas_cubo_3d', self._vision_cb, 10, callback_group=self._cbg)

        self._busy        = True
        self._last_point  = None
        self._lock        = threading.Lock()

        self._worker = threading.Thread(target=self._hilo_logica, daemon=True)
        self._worker.start()

        self.get_logger().info('🤖 Nodo iniciado. Esperando servicios...')

    def _call_srv(self, client, request, timeout: float = 20.0):
        future = client.call_async(request)
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout:
                self.get_logger().error(f'⏱ Timeout ({timeout}s) esperando: {client.srv_name}')
                return None
            time.sleep(0.02)
        return future.result()

    def _esperar_servicios(self):
        criticos = {
            'arm_plan':       self._arm_plan,
            'arm_exec':       self._arm_exec,
            'arm_joint_plan': self._arm_joint_plan
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
                self.get_logger().warn(f'  ⚠ {nombre} no disponible. ¿Lanzaste con add_gripper:=true?')

        self.get_logger().info('✅ Todos los servicios de MoveIt verificados.\n')

    def _vision_cb(self, msg: Point):
        if self._busy:
            return

        z = msg.z
        if not (0.15 < z < 1.50):
            self.get_logger().warn(f'⚠ Lectura descartada: Z={z:.3f}m')
            return

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
            self.get_logger().info(f'📦 Objeto detectado: X={punto.x:.3f}  Y={punto.y:.3f}  Z={punto.z:.3f}')

            exito = self._ciclo_pick(punto)

            estado = '✅ Éxito' if exito else '⚠ Sin éxito confirmado'
            self.get_logger().info(f'{estado}. Regresando a HOME...\n')

            self._ir_a_home()
            self._busy = False
            self.get_logger().info('🏠 HOME. Listo para el próximo objeto.\n')

    def _ciclo_pick(self, punto: Point, max_intentos: int = 3) -> bool:
        for intento in range(1, max_intentos + 1):
            self.get_logger().info(f'\n── Intento {intento}/{max_intentos} ──')

            resultado = self._calcular_pose_objeto(punto)
            if resultado is None:
                return False

            x_obj, y_obj, z_obj = resultado
            
            yaw  = math.atan2(y_obj, x_obj)
            quat = quat_gripper_apuntando_abajo(yaw)

            x_t = x_obj
            y_t = y_obj
            z_t = z_obj + PICK_OFFSET_Z_M + GRIPPER_LENGTH_M

            if not dentro_workspace(x_t, y_t, z_t):
                self.get_logger().error('🚫 Target fuera del workspace.')
                return False

            if not self._plan_pose(x_t, y_t, z_t, quat, nombre='PRE-PICK'):
                return False

            print('\n' + '═' * 55)
            print('  ✅ Plan calculado en RViz.')
            print('  ▶  Presiona  [ENTER] para EJECUTAR.')
            print('  ⛔  Presiona  Ctrl+C para CANCELAR.')
            print('═' * 55)

            try:
                input('> ')
            except (EOFError, KeyboardInterrupt):
                return False

            self.get_logger().info('🚀 Ejecutando trayectoria...')
            if not self._exec_plan():
                return False

            time.sleep(2.0)

            with self._lock:
                nuevo_punto = self._last_point
                self._last_point = None

            if nuevo_punto is None:
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
        req.target.position.x    = float(x)
        req.target.position.y    = float(y)
        req.target.position.z    = float(z)
        req.target.orientation.x = float(quat[0])
        req.target.orientation.y = float(quat[1])
        req.target.orientation.z = float(quat[2])
        req.target.orientation.w = float(quat[3])

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
        self.get_logger().info('🏠 Planificando trayectoria a HOME...')
        req = PlanJoint.Request()
        req.target = HOME_JOINTS
        resp = self._call_srv(self._arm_joint_plan, req, timeout=25.0)

        if resp and resp.success:
            self.get_logger().info('✔ Plan HOME calculado. Ejecutando...')
            self._exec_plan()
        else:
            self.get_logger().warn('⚠ No se pudo planificar a HOME. Verifica colisiones.')

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('\n⛔ KeyboardInterrupt recibido.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
