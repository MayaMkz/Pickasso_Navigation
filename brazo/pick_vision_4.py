#!/usr/bin/env python3
"""
pick_and_place.py — Sistema Pick & Place para xArm5 (Batch por Color con Offset Dinámico)
==========================================================================================
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
MAPA_COLOR_ARUCO = {
    'red': '6',
    'blue': '7',
    'yellow': '8',
    'pink': '9',
    'green': '10'
}

# ──────────────────────────────────────────────────────────────────────
# CONFIGURACIÓN GLOBAL
# ──────────────────────────────────────────────────────────────────────
TF_BASE   = 'link_base'
TF_FLANGE = 'link5'

HOME_JOINTS         = [0.0, 0.0, -2.04, 2.02, 0.0]
SEARCH_ARUCO_JOINTS = [1.5708, 0.0, -2.04, 2.02, 0.0] 

CAMERA_Z_OFFSET_M = 0.20 
PICK_OFFSET_Z_M   = 0.03  

# --- OFFSETS DINÁMICOS ---
PLACE_OFFSET_X_BASE = 0.05  # Primer cubo a 5cm
PLACE_OFFSET_X_INC  = 0.05  # Incremento de 5cm por cada cubo extra
PLACE_OFFSET_Y_M    = 0.00  

GRIPPER_LENGTH_M  = 0.170 
Z_AGARRE_EXTRA_M  = 0.19  
Z_LIFT_M          = 0.01  

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
        self._z_cubo_guardado  = 0.0  
        self._lock             = threading.Lock()

        # --- VARIABLES DE ESTADO PARA BATCH ---
        self._color_objetivo   = None
        self._contador_cubos   = 0

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
          # Filtro Inteligente: 
            # Si estamos buscando un color específico, SOLO guardamos el cubo si coincide con ese color.
            # Si no coincide, lo ignoramos por completo.
            if self._color_objetivo is not None:
                color_detectado = msg.header.frame_id.lower()
                if color_detectado == self._color_objetivo:
                    self._last_cubo_msg = msg
            else:
                self._last_cubo_msg = msg

    def _vision_aruco_cb(self, msg: PointStamped):
        with self._lock:
            id_aruco = msg.header.frame_id
            self._arucos_vistos[id_aruco] = msg.point

    def _hilo_logica(self):
        self._esperar_servicios()
        self._ir_a_home()

        self._busy = False

        while rclpy.ok():
            # 1. SOLICITAR INPUT SI NO HAY COLOR ACTIVO
            if self._color_objetivo is None:
                print('\n' + '═'*60)
                print(f' COLORES DISPONIBLES: {list(MAPA_COLOR_ARUCO.keys())}')
                print('═'*60)
                color_in = input('¿Qué color de dado quieres recolectar hoy? ').strip().lower()

                if color_in not in MAPA_COLOR_ARUCO:
                    self.get_logger().warn(f'Color "{color_in}" no reconocido. Intenta de nuevo.')
                    continue
                
                self._color_objetivo = color_in
                self._contador_cubos = 0  # Reiniciamos el offset para el nuevo lote
                self.get_logger().info(f'Iniciando LOTE: Recolección de cubos {self._color_objetivo.upper()}')

            # 2. BUSCAR EL CUBO ESPECÍFICO (CON TIMEOUT)
            t_espera = 0.0
            timeout_busqueda = 8.0 # Si pasan 8s sin ver el color, asumimos que se acabaron
            cubo_encontrado = None

            while t_espera < timeout_busqueda and rclpy.ok():
                with self._lock:
                    if self._last_cubo_msg is not None:
                        color_detectado = self._last_cubo_msg.header.frame_id.lower()
                        # Solo procesamos si coincide con nuestro objetivo actual
                        if color_detectado == self._color_objetivo:
                            cubo_encontrado = self._last_cubo_msg
                            self._last_cubo_msg = None
                            break
                time.sleep(0.1)
                t_espera += 0.1

            if cubo_encontrado is None:
                self.get_logger().info(f'No hay más cubos {self._color_objetivo.upper()} en la mesa (Timeout). Lote terminado.')
                self._color_objetivo = None # Reiniciamos para volver a preguntar
                continue

            # 3. EJECUTAR RUTINA DE PICK & PLACE
            self._busy = True
            punto_cubo = cubo_encontrado.point
            target_aruco_id = MAPA_COLOR_ARUCO[self._color_objetivo]

            self.get_logger().info(f'Cubo [{self._color_objetivo.upper()}] (Cubo #{self._contador_cubos + 1}) detectado. Iniciando Pick...')

            exito_pick = self._ciclo_pick(punto_cubo)
            
            if exito_pick:
                self.get_logger().info(f'Girando a buscar ArUco ID {target_aruco_id}...')
                self._ir_a_buscar_aruco()
                
                punto_aruco = self._esperar_deteccion_aruco(target_aruco_id)
                
                if punto_aruco:
                    self.get_logger().info(f'ArUco {target_aruco_id} detectado. Iniciando Place.')
                    exito_place = self._ciclo_place(punto_aruco)
                    
                    if exito_place:
                        # ¡Solo aumentamos el contador si logramos colocar el cubo con éxito!
                        self._contador_cubos += 1
                else:
                    self.get_logger().error(f'No se encontró el ArUco {target_aruco_id} a tiempo. Abortando.')
            else:
                self.get_logger().error('Falló la secuencia de Pick.')

            self._ir_a_home()
            self._busy = False

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

            x_obj, y_obj, z_obj = resultado
            yaw  = math.atan2(y_obj, x_obj)
            quat = quat_gripper_apuntando_abajo(yaw)
            R_target = quat_to_rot_matrix(quat)

            p_cam_target = np.array([x_obj, y_obj, z_obj + CAMERA_Z_OFFSET_M])
            t_cam = np.array([0.067506, 0.007342, 0.035900])
            p_brida_cam = p_cam_target - R_target @ t_cam

            if not self._plan_pose(p_brida_cam[0], p_brida_cam[1], p_brida_cam[2], quat, 'CENTRAR_CAMARA'): return False
            self._exec_plan()
            time.sleep(0.5)

            self._z_cubo_guardado = z_obj 

            p_grip_pre = np.array([x_obj, y_obj, z_obj + PICK_OFFSET_Z_M])
            t_grip = np.array([0.0, 0.0, GRIPPER_LENGTH_M])
            p_brida_pre = p_grip_pre - R_target @ t_grip

            if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'PRE-PICK'): self._exec_plan()

            p_grip_pick = np.array([x_obj, y_obj, z_obj - Z_AGARRE_EXTRA_M])
            p_brida_pick = p_grip_pick - R_target @ t_grip

            if self._plan_pose(p_brida_pick[0], p_brida_pick[1], p_brida_pick[2], quat, 'PICK'):
                self._exec_plan()
            else:
                return False

            self._mover_gripper(cerrar=True)
            time.sleep(0.5)

            exito_lift = False
            alturas_escape = [Z_LIFT_M, 0.04, PICK_OFFSET_Z_M] 
            for elevacion in alturas_escape:
                p_grip_post = np.array([x_obj, y_obj, z_obj + elevacion])
                p_brida_post = p_grip_post - R_target @ t_grip
                if self._plan_pose(p_brida_post[0], p_brida_post[1], p_brida_post[2], quat, f'ESCAPE-Z({elevacion}m)'):
                    self._exec_plan()
                    exito_lift = True
                    break
                    
            if not exito_lift:
                self.get_logger().error('Imposible levantar el brazo (Singularidad absoluta).')
                return False
            return True
        return False

    def _ciclo_place(self, punto_aruco) -> bool:
        resultado = self._calcular_pose_objeto(punto_aruco)
        if not resultado: return False

        x_ar, y_ar, _ = resultado
        
        # --- CÁLCULO DEL OFFSET DINÁMICO ---
        # Cubo 1: 0.05 + (0 * 0.05) = 0.05m
        # Cubo 2: 0.05 + (1 * 0.05) = 0.10m
        # Cubo 3: 0.05 + (2 * 0.05) = 0.15m
        offset_dinamico_x = PLACE_OFFSET_X_BASE + (self._contador_cubos * PLACE_OFFSET_X_INC)
        
        x_target = x_ar + offset_dinamico_x
        y_target = y_ar + PLACE_OFFSET_Y_M
        z_target = self._z_cubo_guardado 

        yaw = math.atan2(y_target, x_target)
        quat = quat_gripper_apuntando_abajo(yaw)
        R_target = quat_to_rot_matrix(quat)
        t_grip = np.array([0.0, 0.0, GRIPPER_LENGTH_M])

        p_grip_pre = np.array([x_target, y_target, z_target + PICK_OFFSET_Z_M])
        p_brida_pre = p_grip_pre - R_target @ t_grip

        if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'PRE-PLACE'): self._exec_plan()

        p_grip_place = np.array([x_target, y_target, z_target - Z_AGARRE_EXTRA_M])
        p_brida_place = p_grip_place - R_target @ t_grip

        if self._plan_pose(p_brida_place[0], p_brida_place[1], p_brida_place[2], quat, 'PLACE'):
            self._exec_plan()
        else:
            return False

        self._mover_gripper(cerrar=False)
        time.sleep(0.5)

        exito_lift = False
        alturas_escape = [Z_LIFT_M, 0.04, PICK_OFFSET_Z_M] 
        for elevacion in alturas_escape:
            p_grip_post = np.array([x_target, y_target, z_target + elevacion])
            p_brida_post = p_grip_post - R_target @ t_grip
            if self._plan_pose(p_brida_post[0], p_brida_post[1], p_brida_post[2], quat, f'ESCAPE-Z({elevacion}m)'):
                self._exec_plan()
                exito_lift = True
                break
                
        if not exito_lift:
            self.get_logger().error('Imposible levantar el brazo (Singularidad absoluta).')
            return False

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
