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

HOME_JOINTS         = [0.0, 0.0, -2.04, 2.02, 0.0]
# Giro de 90° (1.5708 rad) en la base para buscar el ArUco
SEARCH_ARUCO_JOINTS = [1.5708, 0.0, -2.04, 2.02, 0.0] 

CAMERA_Z_OFFSET_M = 0.20  # 20 cm de altura para centrar
PICK_OFFSET_Z_M   = 0.03  # 3 cm de seguridad antes del agarre

# --- OFFSETS DE PLACE (Ajustables) ---
PLACE_OFFSET_X_M  = 0.05  # Deja el cubo 8 cm en X respecto al ArUco
PLACE_OFFSET_Y_M  = 0.00  # Centrado en Y respecto al ArUco

# --- PARÁMETROS FÍSICOS ---
GRIPPER_LENGTH_M  = 0.170 
Z_AGARRE_EXTRA_M  = 0.19  # Baja 1 cm EXTRA desde la tapa

WS_X_MIN, WS_X_MAX =  0.05,  0.70
WS_Y_MIN, WS_Y_MAX = -0.50,  0.50
WS_Z_MIN, WS_Z_MAX = -0.05,  0.70

# Transformación Brida a Cámara 
T_BRIDA_CAM = np.array([
    [ 0.,  1.,  0.,  0.067506],
    [-1.,  0.,  0.,  0.007342],
    [ 0.,  0.,  1.,  0.035900],
    [ 0.,  0.,  0.,  1.        ]
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
    def __init__(self):
        super().__init__('xarm5_pick_place')
        self._cbg = ReentrantCallbackGroup()

        self._arm_plan  = self.create_client(PlanPose,  '/xarm_pose_plan',            callback_group=self._cbg)
        self._arm_exec  = self.create_client(PlanExec,  '/xarm_exec_plan',            callback_group=self._cbg)
        self._arm_joint_plan = self.create_client(PlanJoint, '/xarm_joint_plan',      callback_group=self._cbg)
        self._grip_plan = self.create_client(PlanJoint, '/xarm_gripper_joint_plan',   callback_group=self._cbg)
        self._grip_exec = self.create_client(PlanExec,  '/xarm_gripper_exec_plan',    callback_group=self._cbg)

        self._tf_buf = tf2_ros.Buffer()
        self._tf_lst = tf2_ros.TransformListener(self._tf_buf, self)

        # Suscriptor para el Cubo
        self._sub_cubo = self.create_subscription(
            Point, 'coordenadas_cubo_3d', self._vision_cb, 10, callback_group=self._cbg)
        
        # Suscriptor para el ArUco
        self._sub_aruco = self.create_subscription(
            Point, 'coordenadas_aruco_6', self._vision_aruco_cb, 10, callback_group=self._cbg)

        self._busy             = True
        self._last_point       = None
        self._last_aruco_point = None
        self._z_cubo_guardado  = 0.0  # Memoria para repetir la bajada Z exacta
        self._lock             = threading.Lock()

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

    def _vision_aruco_cb(self, msg: Point):
        with self._lock:
            self._last_aruco_point = msg

    def _hilo_logica(self):
        self._esperar_servicios()
        self._ir_a_home()

        self._busy = False
        self.get_logger().info('🎯 Sistema listo. Esperando detecciones de cubo...\n')

        while rclpy.ok():
            with self._lock:
                punto_cubo = self._last_point
                self._last_point = None

            if punto_cubo is None:
                time.sleep(0.05)
                continue

            self._busy = True
            self.get_logger().info(f'📦 Cubo detectado. Iniciando secuencia PICK.')

            exito_pick = self._ciclo_pick(punto_cubo)
            
            if exito_pick:
                self.get_logger().info('✅ Pick completado. Girando a buscar ArUco ID 6...')
                self._ir_a_buscar_aruco()
                
                # Esperar a que la cámara vea el ArUco
                punto_aruco = self._esperar_deteccion_aruco()
                
                if punto_aruco:
                    self.get_logger().info('🎯 ArUco detectado. Iniciando secuencia PLACE.')
                    self._ciclo_place(punto_aruco)
                else:
                    self.get_logger().error('❌ Tiempo agotado buscando ArUco. Abortando.')

            self._ir_a_home()
            self._busy = False
            self.get_logger().info('🏠 HOME. Listo para el próximo objetivo.\n')

    def _esperar_deteccion_aruco(self, timeout_sec=15.0):
        """Espera hasta 15 segundos para detectar el ArUco tras girar"""
        with self._lock:
            self._last_aruco_point = None # Limpiar lecturas viejas
            
        t_start = time.time()
        while time.time() - t_start < timeout_sec:
            with self._lock:
                if self._last_aruco_point is not None:
                    return self._last_aruco_point
            time.sleep(0.1)
        return None

    def _ciclo_pick(self, punto: Point, max_intentos: int = 3) -> bool:
        for intento in range(1, max_intentos + 1):
            resultado = self._calcular_pose_objeto(punto)
            if not resultado: return False

            x_obj, y_obj, z_obj = resultado
            yaw  = math.atan2(y_obj, x_obj)
            quat = quat_gripper_apuntando_abajo(yaw)
            
            R_target = quat_to_rot_matrix(quat)

            # --- CENTRAR CÁMARA ---
            p_cam_target = np.array([x_obj, y_obj, z_obj + CAMERA_Z_OFFSET_M])
            t_cam = np.array([0.067506, 0.007342, 0.035900])
            p_brida_cam = p_cam_target - R_target @ t_cam

            self.get_logger().info('📸 FASE 1: Calculando centrado de cámara...')
            if not self._plan_pose(p_brida_cam[0], p_brida_cam[1], p_brida_cam[2], quat, 'CENTRAR_CAMARA'):
                return False

            self._exec_plan()
            time.sleep(0.5)

            # GUARDAMOS LA ALTURA Z DEL CUBO PARA USARLA EN EL PLACE
            self._z_cubo_guardado = z_obj 

            # --- PRE-PICK ---
            p_grip_pre = np.array([x_obj, y_obj, z_obj + PICK_OFFSET_Z_M])
            t_grip = np.array([0.0, 0.0, GRIPPER_LENGTH_M])
            p_brida_pre = p_grip_pre - R_target @ t_grip

            if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'PRE-PICK'):
                self._exec_plan()

            # --- PICK ---
            p_grip_pick = np.array([x_obj, y_obj, z_obj - Z_AGARRE_EXTRA_M])
            p_brida_pick = p_grip_pick - R_target @ t_grip

            if self._plan_pose(p_brida_pick[0], p_brida_pick[1], p_brida_pick[2], quat, 'PICK'):
                self._exec_plan()
            else:
                self.get_logger().error('🚨 MoveIt rechazó la bajada.')
                return False

            # --- CERRAR Y SUBIR ---
            self._mover_gripper(cerrar=True)

            if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'POST-PICK'):
                self._exec_plan()

            return True

        return False

    def _ciclo_place(self, punto_aruco: Point) -> bool:
        resultado = self._calcular_pose_objeto(punto_aruco)
        if not resultado: return False

        x_ar, y_ar, _ = resultado
        
        # Aplicamos los offsets deseados respecto al centro del ArUco
        x_target = x_ar + PLACE_OFFSET_X_M
        y_target = y_ar + PLACE_OFFSET_Y_M
        
        # MAGIA AQUÍ: Usamos exactamente el mismo Z absoluto registrado durante el Pick
        z_target = self._z_cubo_guardado 

        yaw = math.atan2(y_target, x_target)
        quat = quat_gripper_apuntando_abajo(yaw)
        R_target = quat_to_rot_matrix(quat)
        t_grip = np.array([0.0, 0.0, GRIPPER_LENGTH_M])

        self.get_logger().info(f'📍 Coordenadas Place Calculadas: X:{x_target:.3f}, Y:{y_target:.3f}, Z:{z_target:.3f}')

        # ──────────────────────────────────────────────────────────
        # FASE 1: CENTRAR GRIPPER (PRE-PLACE)
        # ──────────────────────────────────────────────────────────
        self.get_logger().info('>< Acercando cubo a zona de descarga (Pre-Place)...')
        p_grip_pre = np.array([x_target, y_target, z_target + PICK_OFFSET_Z_M])
        p_brida_pre = p_grip_pre - R_target @ t_grip

        if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'PRE-PLACE'):
            self._exec_plan()

        # ──────────────────────────────────────────────────────────
        # FASE 2: BAJAR (PLACE)
        # ──────────────────────────────────────────────────────────
        self.get_logger().info('👇 Bajando cubo a la mesa...')
        p_grip_place = np.array([x_target, y_target, z_target - Z_AGARRE_EXTRA_M])
        p_brida_place = p_grip_place - R_target @ t_grip

        if self._plan_pose(p_brida_place[0], p_brida_place[1], p_brida_place[2], quat, 'PLACE'):
            self._exec_plan()
        else:
            self.get_logger().error('🚨 MoveIt rechazó la bajada del Place.')
            return False

        # ──────────────────────────────────────────────────────────
        # FASE 3: ABRIR GRIPPER Y SUBIR (POST-PLACE)
        # ──────────────────────────────────────────────────────────
        self._mover_gripper(cerrar=False)
        time.sleep(0.5)

        self.get_logger().info('👆 Subiendo a posición segura...')
        if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'POST-PLACE'):
            self._exec_plan()

        return True

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
        if resp is not None and resp.success:
            return True
        else:
            self.get_logger().error(f'❌ Falló el plan de MoveIt hacia: {nombre}')
            return False

    def _exec_plan(self) -> bool:
        req = PlanExec.Request()
        req.wait = True
        resp = self._call_srv(self._arm_exec, req, timeout=40.0)
        return resp is not None

    def _mover_gripper(self, cerrar: bool):
        req = PlanJoint.Request()
        req.target = [0.5]*6 if cerrar else [0.0]*6
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
            # SE ELIMINÓ LA CONDICIÓN self._busy PARA QUE SIEMPRE ABRA AL VOLVER A CASA
            self._mover_gripper(cerrar=False)

    def _ir_a_buscar_aruco(self):
        req = PlanJoint.Request()
        req.target = SEARCH_ARUCO_JOINTS
        resp = self._call_srv(self._arm_joint_plan, req, timeout=25.0)
        if resp and resp.success:
            self._exec_plan()

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
