#!/usr/bin/env python3
"""
pick_and_place.py — Sistema Pick & Place para xArm5 (Integración con Carro Móvil por Banderas)
==============================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PointStamped 
from std_msgs.msg import String  
from xarm_msgs.srv import PlanPose, PlanExec, PlanJoint

import tf2_ros
from tf2_ros import TransformException

import numpy as np
import math
import threading
import time

# ──────────────────────────────────────────────────────────────────────
# CONFIGURACIÓN DE COLORES
# ──────────────────────────────────────────────────────────────────────
COLORES_VALIDOS = ['red', 'blue', 'pink', 'green']
TARGET_ARUCO_ID = '9' 

# ──────────────────────────────────────────────────────────────────────
# CONFIGURACIÓN DE POSICIONES ARTICULARES (Joints)
# ──────────────────────────────────────────────────────────────────────
HOME_JOINTS         = [0.0, 0.0, -2.04, 2.02, 0.0]
HOME_MOVIMIENTO     = [0.0, -1.2566, -0.0873, 1.3614, 0.0]
PRE_PLATFORM_JOINTS = [-1.5708, 0.0, -2.04, 2.02, 0.0]
SEARCH_ARUCO_JOINTS = [1.5708, 0.0, -2.04, 2.02, 0.0] 
PLATFORM_JOINTS     = [-1.5708, -0.785398, -0.872665, 1.74533, 0.0] 

POSICIONES_PLATAFORMA = [
    [-2.0944, -0.628319, -0.174533, 0.802851, 0.0],
    [-1.5708,  -0.698132, -0.148353, 0.8813913, 0.0],
    [-1.16937, -0.628319, -0.174533, 0.802851, 0.0]
]

# ──────────────────────────────────────────────────────────────────────
# CONFIGURACIÓN CARTESIANA Y OFFSETS
# ──────────────────────────────────────────────────────────────────────
TF_BASE   = 'link_base'
TF_FLANGE = 'link5'

CAMERA_Z_OFFSET_M = 0.20 
PICK_OFFSET_Z_M   = 0.03  

PLACE_OFFSET_X_BASE = 0.07  
PLACE_OFFSET_X_INC  = 0.0  
PLACE_OFFSET_Y_M    = 0.07  
PLACE_ALTURA_Z_M    = 0.07  

GRIPPER_LENGTH_M  = 0.170 
Z_AGARRE_EXTRA_M  = 0.19  
Z_LIFT_M          = 0.01  

T_BRIDA_CAM = np.array([
    [ 0.,  1.,  0.,  0.067506],
    [-1.,  0.,  0.,  0.007342],
    [ 0.,  0.,  1.,  0.035900],
    [ 0.,  0.,  0.,  1.      ]
], dtype=np.float64)

# ──────────────────────────────────────────────────────────────────────
# UTILIDADES MATEMÁTICAS
# ──────────────────────────────────────────────────────────────────────
def tf_stamped_to_matrix(tf_stamped) -> np.ndarray:
    t, q = tf_stamped.transform.translation, tf_stamped.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    rot = np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [    2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qw*qx)],
        [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3], T[:3, 3] = rot, [t.x, t.y, t.z]
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

# ──────────────────────────────────────────────────────────────────────
# NODO PRINCIPAL
# ──────────────────────────────────────────────────────────────────────
class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('xarm5_pick_place')
        self._cbg = ReentrantCallbackGroup()

        self._arm_plan       = self.create_client(PlanPose,  '/xarm_pose_plan',          callback_group=self._cbg)
        self._arm_exec       = self.create_client(PlanExec,  '/xarm_exec_plan',          callback_group=self._cbg)
        self._arm_joint_plan = self.create_client(PlanJoint, '/xarm_joint_plan',         callback_group=self._cbg)
        self._grip_plan      = self.create_client(PlanJoint, '/xarm_gripper_joint_plan', callback_group=self._cbg)
        self._grip_exec      = self.create_client(PlanExec,  '/xarm_gripper_exec_plan',  callback_group=self._cbg)

        self._tf_buf = tf2_ros.Buffer()
        self._tf_lst = tf2_ros.TransformListener(self._tf_buf, self)

        self._sub_cubo    = self.create_subscription(PointStamped, 'coordenadas_cubo_3d', self._vision_cb,       10, callback_group=self._cbg)
        self._sub_aruco   = self.create_subscription(PointStamped, 'coordenadas_arucos',  self._vision_aruco_cb, 10, callback_group=self._cbg)
        self._sub_bandera = self.create_subscription(String,       'bandera_estacion',    self._bandera_cb,      10, callback_group=self._cbg)

        self._pub_estado = self.create_publisher(String, 'estado_brazo', 10)

        self._busy          = True
        self._last_cubo_msg = None
        self._arucos_vistos = {}
        self._lock          = threading.Lock()

        self._color_objetivo  = None
        self._estacion_actual = 'ninguna'

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

    # ────────────────────────────────────────────────────────
    # CALLBACKS
    # ────────────────────────────────────────────────────────
    def _vision_cb(self, msg: PointStamped):
        if self._busy: return
        with self._lock:
            if self._color_objetivo is not None:
                if msg.header.frame_id.lower() == self._color_objetivo:
                    self._last_cubo_msg = msg
            else:
                self._last_cubo_msg = msg

    def _vision_aruco_cb(self, msg: PointStamped):
        with self._lock:
            self._arucos_vistos[msg.header.frame_id] = msg.point

    def _bandera_cb(self, msg: String):
        with self._lock:
            self._estacion_actual = msg.data.strip().lower()

    # ────────────────────────────────────────────────────────
    # PUBLICACIÓN SEGURA DE ESTADO  ← NUEVO
    # ────────────────────────────────────────────────────────
    def _publicar_estado(self, estado: str, repeticiones: int = 5, intervalo: float = 0.3):
        """
        Publica el estado varias veces con intervalo para garantizar
        que el comandante lo reciba aunque haya una condición de carrera.
        ROS2 con tópicos normales es fire-and-forget: si el suscriptor
        no está listo en ese instante, el mensaje se pierde.
        """
        msg = String()
        msg.data = estado
        for _ in range(repeticiones):
            self._pub_estado.publish(msg)
            time.sleep(intervalo)
        self.get_logger().info(f"Estado publicado ({repeticiones}x): '{estado}'")

    # ────────────────────────────────────────────────────────
    # HILO PRINCIPAL (MÁQUINA DE ESTADOS)
    # ────────────────────────────────────────────────────────
    def _hilo_logica(self):
        self._esperar_servicios()
        self._ir_a_posicion_articular(HOME_MOVIMIENTO)
        self._mover_gripper(cerrar=False)
        self._busy = False

        while rclpy.ok():
            # 1. PEDIR COLOR
            if self._color_objetivo is None:
                print('\n' + '═'*60)
                print(f' COLORES DISPONIBLES: {COLORES_VALIDOS}')
                print('═'*60)
                color_in = input('¿Qué color de dado quiere recolectar? ').strip().lower()
                if color_in not in COLORES_VALIDOS:
                    self.get_logger().warn(f'Color "{color_in}" no reconocido.')
                    continue
                self._color_objetivo = color_in
                self.get_logger().info(f'Orden: {self._color_objetivo.upper()}. Esperando carro en PICK...')

            # 2. ESPERAR LLEGADA A PICK
            while rclpy.ok():
                with self._lock:
                    if self._estacion_actual == 'pick':
                        break
                time.sleep(0.5)

            self.get_logger().info('¡En PICK! Iniciando carga...')
            cubos_cargados = self._rutina_estacion_pick()

            if cubos_cargados == 0:
                self.get_logger().info('Sin cubos. Abortando orden.')
                self._color_objetivo = None
                with self._lock: self._estacion_actual = 'ninguna'
                self._ir_a_posicion_articular(HOME_MOVIMIENTO)
                continue

            self.get_logger().info(f'Carga OK ({cubos_cargados} cubos). Plegando...')
            self._ir_a_posicion_articular(HOME_MOVIMIENTO)

            # ── FIX: publicar pick_completado varias veces ──
            self._publicar_estado('pick_completado')
            self.get_logger().info('Esperando carro en PLACE...')

            # 3. ESPERAR LLEGADA A PLACE
            while rclpy.ok():
                with self._lock:
                    if self._estacion_actual == 'place':
                        break
                time.sleep(0.5)

            self.get_logger().info('¡En PLACE! Iniciando descarga...')
            self._rutina_estacion_place(cubos_cargados)

            self.get_logger().info('Descarga OK. Plegando...')
            self._ir_a_posicion_articular(HOME_MOVIMIENTO)

            # ── FIX: publicar place_completado varias veces ──
            self._publicar_estado('place_completado')

            self.get_logger().info('Lote completado.')
            self._color_objetivo = None
            with self._lock: self._estacion_actual = 'completado'

    # ────────────────────────────────────────────────────────
    # SUBRUTINAS DE ESTACIÓN
    # ────────────────────────────────────────────────────────
    def _rutina_estacion_pick(self) -> int:
        cubos_en_plataforma = 0
        self._ir_a_posicion_articular(HOME_JOINTS)

        while cubos_en_plataforma < 3 and rclpy.ok():
            cubo = self._esperar_cubo(timeout=5.0)
            if cubo is None:
                self.get_logger().info('Sin más cubos en mesa.')
                break

            self._busy = True
            if self._ciclo_pick(cubo.point):
                pos_objetivo = POSICIONES_PLATAFORMA[cubos_en_plataforma]
                self._ir_a_posicion_articular(PRE_PLATFORM_JOINTS)
                self._ir_a_posicion_articular(pos_objetivo)
                self._mover_gripper(cerrar=False)
                time.sleep(0.5)
                self._ir_a_posicion_articular(PRE_PLATFORM_JOINTS)
                cubos_en_plataforma += 1
            else:
                self.get_logger().error('Falló el Pick.')

            self._ir_a_posicion_articular(HOME_JOINTS)
            self._busy = False

        return cubos_en_plataforma

    def _rutina_estacion_place(self, cantidad_cubos: int):
        self._ir_a_posicion_articular(HOME_JOINTS)

        for iteracion in range(cantidad_cubos):
            if not rclpy.ok(): break

            self._ir_a_posicion_articular(PRE_PLATFORM_JOINTS)
            self._ir_a_posicion_articular(PLATFORM_JOINTS)

            cubo_plataforma = self._esperar_cubo(timeout=8.0)
            if cubo_plataforma is None:
                self.get_logger().error('Sin cubo en plataforma. Omitiendo.')
                continue

            self._busy = True
            if self._ciclo_pick(cubo_plataforma.point):
                self._ir_a_posicion_articular(PRE_PLATFORM_JOINTS)
                self._ir_a_posicion_articular(SEARCH_ARUCO_JOINTS)
                punto_aruco = self._esperar_deteccion_aruco(TARGET_ARUCO_ID)
                if punto_aruco:
                    if self._ciclo_place_elevado(punto_aruco, iteracion):
                        pass
                    self._ir_a_posicion_articular(SEARCH_ARUCO_JOINTS)
                else:
                    self.get_logger().error('Sin ArUco. Soltando cubo.')
                    self._mover_gripper(cerrar=False)
                    self._ir_a_posicion_articular(SEARCH_ARUCO_JOINTS)

            self._busy = False

    # ────────────────────────────────────────────────────────
    # FUNCIONES DE MOVIMIENTO BASE
    # ────────────────────────────────────────────────────────
    def _esperar_cubo(self, timeout=8.0):
        t = 0.0
        while t < timeout and rclpy.ok():
            with self._lock:
                if self._last_cubo_msg is not None:
                    cubo = self._last_cubo_msg
                    self._last_cubo_msg = None
                    return cubo
            time.sleep(0.1)
            t += 0.1
        return None

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

    def _ir_a_posicion_articular(self, joints):
        req = PlanJoint.Request()
        req.target = joints
        resp = self._call_srv(self._arm_joint_plan, req, timeout=25.0)
        if resp and resp.success:
            self._exec_plan()

    def _ciclo_pick(self, punto, max_intentos=3) -> bool:
        for _ in range(max_intentos):
            resultado = self._calcular_pose_objeto(punto)
            if not resultado: return False

            x_obj, y_obj, z_obj = resultado
            yaw  = math.atan2(y_obj, x_obj)
            quat = quat_gripper_apuntando_abajo(yaw)
            R_target = quat_to_rot_matrix(quat)
            t_grip = np.array([0.0, 0.0, GRIPPER_LENGTH_M])

            p_cam_target = np.array([x_obj, y_obj, z_obj + CAMERA_Z_OFFSET_M])
            t_cam = np.array([0.067506, 0.007342, 0.035900])
            p_brida_cam = p_cam_target - R_target @ t_cam
            if not self._plan_pose(p_brida_cam[0], p_brida_cam[1], p_brida_cam[2], quat, 'CENTRAR'): return False
            self._exec_plan()
            time.sleep(0.5)

            p_brida_pre = np.array([x_obj, y_obj, z_obj + PICK_OFFSET_Z_M]) - R_target @ t_grip
            if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'PRE-PICK'):
                self._exec_plan()

            p_brida_pick = np.array([x_obj, y_obj, z_obj - Z_AGARRE_EXTRA_M]) - R_target @ t_grip
            if not self._plan_pose(p_brida_pick[0], p_brida_pick[1], p_brida_pick[2], quat, 'PICK'):
                return False
            self._exec_plan()
            self._mover_gripper(cerrar=True)
            time.sleep(0.5)

            for elev in [Z_LIFT_M, 0.04, PICK_OFFSET_Z_M]:
                p_post = np.array([x_obj, y_obj, z_obj + elev]) - R_target @ t_grip
                if self._plan_pose(p_post[0], p_post[1], p_post[2], quat, f'ESCAPE({elev})'):
                    self._exec_plan()
                    return True

            self.get_logger().error('Imposible levantar (singularidad).')
            return False
        return False

    def _ciclo_place_elevado(self, punto_aruco, offset_iteracion) -> bool:
        resultado = self._calcular_pose_objeto(punto_aruco)
        if not resultado: return False

        x_ar, y_ar, z_ar = resultado
        x_target = x_ar + PLACE_OFFSET_X_BASE + (offset_iteracion * PLACE_OFFSET_X_INC)
        y_target = y_ar + PLACE_OFFSET_Y_M
        z_target = z_ar + PLACE_ALTURA_Z_M

        yaw  = math.atan2(y_target, x_target)
        quat = quat_gripper_apuntando_abajo(yaw)
        R_target = quat_to_rot_matrix(quat)
        t_grip = np.array([0.0, 0.0, GRIPPER_LENGTH_M])

        p_brida_pre = np.array([x_target, y_target, z_target + PICK_OFFSET_Z_M]) - R_target @ t_grip
        if self._plan_pose(p_brida_pre[0], p_brida_pre[1], p_brida_pre[2], quat, 'PRE-PLACE'):
            self._exec_plan()

        p_brida_place = np.array([x_target, y_target, z_target]) - R_target @ t_grip
        if not self._plan_pose(p_brida_place[0], p_brida_place[1], p_brida_place[2], quat, 'PLACE'):
            return False
        self._exec_plan()
        self._mover_gripper(cerrar=False)
        time.sleep(0.5)

        for elev in [Z_LIFT_M, 0.02, PICK_OFFSET_Z_M]:
            p_post = np.array([x_target, y_target, z_target + elev]) - R_target @ t_grip
            if self._plan_pose(p_post[0], p_post[1], p_post[2], quat, f'ESCAPE({elev})'):
                self._exec_plan()
                return True

        self.get_logger().error('Imposible levantar (singularidad).')
        return False

    def _calcular_pose_objeto(self, punto):
        try:
            tf_stamp = self._tf_buf.lookup_transform(
                TF_BASE, TF_FLANGE, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException:
            return None
        T_base_brida = tf_stamped_to_matrix(tf_stamp)
        p_cam    = np.array([punto.x, punto.y, punto.z, 1.0], dtype=np.float64)
        p_base   = (T_base_brida @ T_BRIDA_CAM) @ p_cam
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
        req.target = [0.55]*6 if cerrar else [0.0]*6
        resp = self._call_srv(self._grip_plan, req, timeout=5.0)
        if resp and resp.success:
            req_exec = PlanExec.Request()
            req_exec.wait = True
            self._call_srv(self._grip_exec, req_exec, timeout=10.0)
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
