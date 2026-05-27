#!/usr/bin/env python3
"""
pick_and_place.py — Sistema Pick & Place para xArm5 (Visión 2.5D con Z Fijas y Escape Seguro)
=============================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PointStamped 
from xarm_msgs.srv import PlanPose, PlanExec, PlanJoint

import tf2_ros
from tf2_ros import TransformException

import numpy as np
import math
import threading
import time

# ──────────────────────────────────────────────────────────────────────
# CONFIGURACIÓN DE SORTEO (SORTING)
# ──────────────────────────────────────────────────────────────────────
# Asegúrate de que estos nombres coincidan con las etiquetas de YOLO
MAPA_COLOR_ARUCO = {
    'red': '6',
    'blue': '7',
    'yellow': '8',
    'pink': '9'
}

# ──────────────────────────────────────────────────────────────────────
# CONFIGURACIÓN Z FIJAS (ABSOLUTAS EN LA BASE DEL ROBOT)
# ──────────────────────────────────────────────────────────────────────
Z_FIJA_PICK_M  = 0.04  # Altura absoluta de la tapa del cubo (ej: 4 cm)
Z_FIJA_PLACE_M = 0.01  # Altura absoluta de la mesa para el ArUco
Z_LIFT_M       = 0.05  # Elevación extra de 5cm para escape seguro

# ──────────────────────────────────────────────────────────────────────
# CONFIGURACIÓN GLOBAL
# ──────────────────────────────────────────────────────────────────────
TF_BASE   = 'link_base'
TF_FLANGE = 'link5'

HOME_JOINTS         = [0.0, 0.0, -2.04, 2.02, 0.0]
SEARCH_ARUCO_JOINTS = [1.5708, 0.0, -2.04, 2.02, 0.0] 

CAMERA_Z_OFFSET_M = 0.20 
PICK_OFFSET_Z_M   = 0.03  # Acercamiento previo (pre-pick / pre-place)

PLACE_OFFSET_X_M  = 0.05  
PLACE_OFFSET_Y_M  = 0.00  

GRIPPER_LENGTH_M  = 0.170 
Z_AGARRE_EXTRA_M  = 0.01  

T_BRIDA_CAM = np.array([
    [ 0.,  1.,  0.,  0.067506],
    [-1.,  0.,  0.,  0.007342],
    [ 0.,  0.,  1.,  0.035900],
    [ 0.,  0.,  0.,  1.        ]
], dtype=np.float64)

def tf_stamped_to_matrix(tf_stamped) -> np.ndarray:
    t, q = tf_stamped.transform.translation, tf_stamped.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    rot = np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [    2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qw*qx)],
        [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3], T[:3,  3] = rot, [t.x, t.y, t.z]
    return T

def quat_to_rot_matrix(q: list) -> np.ndarray:
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [    2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qw*qx)],
        [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
    ], dtype=np.float64)

def rpy_to_quat(roll, pitch, yaw) -> list:
    cr, sr = math.cos(roll/2.), math.sin(roll/2.)
    cp, sp = math.cos(pitch/2.), math.sin(pitch/2.)
    cy, sy = math.cos(yaw/2.), math.sin(yaw/2.)
    return [sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy]

def quat_gripper_apuntando_abajo(yaw: float) -> list:
    return rpy_to_quat(math.pi, 0.0, yaw)

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

        self._sub_cubo = self.create_subscription(PointStamped, 'coordenadas_cubo_3d', self._vision_cb, 10, callback_group=self._cbg)
        self._sub_aruco = self.create_subscription(PointStamped, 'coordenadas_arucos', self._vision_aruco_cb, 10, callback_group=self._cbg)

        self._busy             = True
        self._last_cubo_msg    = None
        self._arucos_vistos    = {}   
        self._lock             = threading.Lock()

        self._worker = threading.Thread(target=self._hilo_logica, daemon=True)
        self._worker.start()

    def _call_srv(self, client, request, timeout: float = 20.0):
        future = client.call_async(request)
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout: return None
            time.sleep(0.02)
        return future.result()

    def _esperar_servicios(self):
        for srv in [self._arm_plan, self._arm_exec, self._arm_joint_plan]:
            while not srv.wait_for_service(timeout_sec=2.0): pass
        self._grip_plan.wait_for_service(timeout_sec=3.0)
        self._grip_exec.wait_for_service(timeout_sec=3.0)

    def _vision_cb(self, msg: PointStamped):
        if self._busy: return
        with self._lock:
            self._last_cubo_msg = msg

    def _vision_aruco_cb(self, msg: PointStamped):
        with self._lock:
            id_aruco = msg.header.frame_id
            self._arucos_vistos[id_aruco] = msg.point

    def _hilo_logica(self):
        self._esperar_servicios()
        self._ir_a_home()

        self._busy = False
        self.get_logger().info('Sistema listo. Esperando detecciones de cubo...\n')

        while rclpy.ok():
            with self._lock:
                msg_cubo = self._last_cubo_msg
                self._last_cubo_msg = None

            if msg_cubo is None:
                time.sleep(0.05)
                continue

            self._busy = True
            color_cubo = msg_cubo.header.frame_id.lower()
            punto_cubo = msg_cubo.point
            
            target_aruco_id = MAPA_COLOR_ARUCO.get(color_cubo)
            
            if not target_aruco_id:
                self.get_logger().error(f'Cubo "{color_cubo}" ignorado (sin ArUco asignado).')
                self._busy = False
                continue

            self.get_logger().info(f'Cubo [{color_cubo.upper()}] detectado. Iniciando Pick -> ArUco {target_aruco_id}')

            exito_pick = self._ciclo_pick(punto_cubo)
            
            if exito_pick:
                self.get_logger().info(f'Pick completado. Buscando ArUco ID {target_aruco_id}...')
                self._ir_a_buscar_aruco()
                
                punto_aruco = self._esperar_deteccion_aruco(target_aruco_id)
                
                if punto_aruco:
                    self.get_logger().info(f'ArUco {target_aruco_id} detectado. Iniciando Place.')
                    self._ciclo_place(punto_aruco)
                else:
                    self.get_logger().error(f'No se encontró el ArUco {target_aruco_id}. Abortando.')

            self._ir_a_home()
            self._busy = False
            self.get_logger().info('HOME. Listo para el próximo objetivo.\n')

    def _esperar_deteccion_aruco(self, target_id: str, timeout_sec=15.0):
        with self._lock:
            self._arucos_vistos.clear() 
            
        t_start = time.time()
        while time.time() - t_start < timeout_sec:
            with self._lock:
                if target_id in self._arucos_vistos:
                    return self._arucos_vistos[target_id]
            time.sleep(0.1)
        return None

    def _ciclo_pick(self, punto, max_intentos=3) -> bool:
        for intento in range(1, max_intentos + 1):
            resultado = self._calcular_pose_objeto(punto)
            if not resultado: return False

            # USO DE LA Z FIJA PARA PICK
            x_obj, y_obj, _ = resultado
            z_obj = Z_FIJA_PICK_M 

            yaw  = math.atan2(y_obj, x_obj)
            quat = quat_gripper_apuntando_abajo(yaw)
            R_target = quat_to_rot_matrix(quat)
            t_grip = np.array([0.0, 0.0, GRIPPER_LENGTH_M])

            # 1. CENTRAR CÁMARA
            p_cam_target = np.array([x_obj, y_obj, z_obj + CAMERA_Z_OFFSET_M])
            t_cam = np.array([0.067506, 0.007342, 0.035900])
            p_brida_cam = p_cam_target - R_target @ t_cam

            if not self._plan_pose(p_brida_cam[0], p_brida_cam[1], p_brida_cam[2], quat, 'CENTRAR_CAMARA'): return False
            self._exec_plan()
            time.sleep(0.5)

            # 2. ACERCAMIENTO (PRE-PICK)
            p_grip_pre = np.array([x_obj, y_obj, z_obj + PICK_OFFSET_Z_M])
            p_brida_pre = p_grip_pre - R_target @ t_grip
            if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'PRE-PICK'): self._exec_plan()

            # 3. BAJAR AL CUBO (PICK)
            p_grip_pick = np.array([x_obj, y_obj, z_obj - Z_AGARRE_EXTRA_M])
            p_brida_pick = p_grip_pick - R_target @ t_grip

            if self._plan_pose(p_brida_pick[0], p_brida_pick[1], p_brida_pick[2], quat, 'PICK'):
                self._exec_plan()
            else:
                return False

            # 4. CERRAR GRIPPER
            self._mover_gripper(cerrar=True)
            time.sleep(0.5)

            # 5. ESCAPE SEGURO (Subir 5cm antes de hacer cualquier otra cosa)
            self.get_logger().info('Subiendo 5cm (Retracción segura en Pick)...')
            p_grip_post = np.array([x_obj, y_obj, z_obj + Z_LIFT_M])
            p_brida_post = p_grip_post - R_target @ t_grip
            
            if self._plan_pose(p_brida_post[0], p_brida_post[1], p_brida_post[2], quat, 'POST-PICK-LIFT'): 
                self._exec_plan()
            
            return True
        return False

    def _ciclo_place(self, punto_aruco) -> bool:
        resultado = self._calcular_pose_objeto(punto_aruco)
        if not resultado: return False

        # USO DE LA Z FIJA PARA PLACE
        x_ar, y_ar, _ = resultado
        
        x_target = x_ar + PLACE_OFFSET_X_M
        y_target = y_ar + PLACE_OFFSET_Y_M
        z_target = Z_FIJA_PLACE_M 

        yaw = math.atan2(y_target, x_target)
        quat = quat_gripper_apuntando_abajo(yaw)
        R_target = quat_to_rot_matrix(quat)
        t_grip = np.array([0.0, 0.0, GRIPPER_LENGTH_M])

        # 1. ACERCAMIENTO (PRE-PLACE)
        p_grip_pre = np.array([x_target, y_target, z_target + PICK_OFFSET_Z_M])
        p_brida_pre = p_grip_pre - R_target @ t_grip

        if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'PRE-PLACE'): self._exec_plan()

        # 2. BAJAR A LA MESA (PLACE)
        p_grip_place = np.array([x_target, y_target, z_target - Z_AGARRE_EXTRA_M])
        p_brida_place = p_grip_place - R_target @ t_grip

        if self._plan_pose(p_brida_place[0], p_brida_place[1], p_brida_place[2], quat, 'PLACE'):
            self._exec_plan()
        else:
            return False

        # 3. SOLTAR CUBO
        self._mover_gripper(cerrar=False)
        time.sleep(0.5)

        # 4. ESCAPE SEGURO (Subir 5cm antes de hacer cualquier otra cosa)
        self.get_logger().info('Subiendo 5cm (Retracción segura en Place)...')
        p_grip_post = np.array([x_target, y_target, z_target + Z_LIFT_M])
        p_brida_post = p_grip_post - R_target @ t_grip
        
        if self._plan_pose(p_brida_post[0], p_brida_post[1], p_brida_post[2], quat, 'POST-PLACE-LIFT'): 
            self._exec_plan()
            
        return True

    def _calcular_pose_objeto(self, punto):
        try:
            tf_stamp = self._tf_buf.lookup_transform(TF_BASE, TF_FLANGE, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
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
            self._mover_gripper(cerrar=False)

    def _ir_a_buscar_aruco(self):
        req = PlanJoint.Request()
        req.target = SEARCH_ARUCO_JOINTS
        resp = self._call_srv(self._arm_joint_plan, req, timeout=25.0)
        if resp and resp.success: self._exec_plan()

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
