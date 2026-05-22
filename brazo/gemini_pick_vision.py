#---Terminal 1: ros2 launch xarm_planner xarm5_planner_realmove.launch.py robot_ip:=192.168.1.234

#Terminal 2: ros2 run logica_almacen deteccion

#Terminal 3: ros2 run logica_almacen pick_place

#!/usr/bin/env python3
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
import time
import sys
import select
import tty
import termios
import threading
import os

# --- PARO DE EMERGENCIA ---
def paro_de_emergencia():
    """Hilo de fondo que escucha la tecla 'q' al instante para un E-Stop seguro"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                tecla = sys.stdin.read(1)
                if tecla.lower() == 'q':
                    print("\r\n\r\n [PARO DE EMERGENCIA] Tecla 'q' detectada. Abortando programa... \r\n")
                    os._exit(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# --- MATRICES Y CONSTANTES ---
# T_brida_cam: Cámara a la brida (rotada 180 grados, según tus mediciones)
T_BRIDA_CAM = np.array([
    [-1.0, -0.0,  0.0, 0.067506],
    [ 0.0, -1.0,  0.0, 0.007342],
    [-0.0, -0.0,  1.0, 0.035900],
    [ 0.0,  0.0,  0.0, 1.0     ]
])

GRIPPER_OFFSET_Z = 0.16  # 16 cm de la brida a la punta de los 2 dedos
PICK_OFFSET_Z = 0.10     # 10 cm de altura de seguridad sobre el dado

def tf_stamped_to_matrix(tf_stamped):
    """Convierte TransformStamped a matriz homogénea 4x4."""
    t = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    rot = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx**2 + qy**2)]
    ])
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [t.x, t.y, t.z]
    return T

class VisionPickAndPlace(Node):
    def __init__(self):
        super().__init__('vision_pick_place')
        self.cbg = ReentrantCallbackGroup()

        # Clientes MoveIt
        self.arm_plan = self.create_client(PlanPose, '/xarm_pose_plan', callback_group=self.cbg)
        self.arm_exec = self.create_client(PlanExec, '/xarm_exec_plan', callback_group=self.cbg)
        self.grip_plan = self.create_client(PlanJoint, '/xarm_gripper_joint_plan', callback_group=self.cbg)
        self.grip_exec = self.create_client(PlanExec, '/xarm_gripper_exec_plan', callback_group=self.cbg)

        # Subs y TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.vision_sub = self.create_subscription(Point, '/coordenadas_cubo_3d', self.vision_cb, 10, callback_group=self.cbg)

        self.dado_detectado = None
        self.ocupado = False

        self.get_logger().info('Conectando con MoveIt...')
        self.arm_plan.wait_for_service()
        self.arm_exec.wait_for_service()
        self.grip_plan.wait_for_service()
        self.grip_exec.wait_for_service()
        self.get_logger().info('¡Conexión establecida! PRESIONA "q" PARA PARO DE EMERGENCIA.')

    def vision_cb(self, msg):
        if not self.ocupado:
            self.dado_detectado = msg

    def mover_brazo(self, x, y, z, roll=3.1416, pitch=0.0, yaw=0.0, nombre_pose="Posición"):
        self.get_logger().info(f'---> Calculando plan a {nombre_pose}: [X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}]')
        
        # Conversión simple RPY a Cuaternión para apuntar hacia abajo
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        req_plan = PlanPose.Request()
        req_plan.target.position.x = x
        req_plan.target.position.y = y
        req_plan.target.position.z = z
        req_plan.target.orientation.x = qx
        req_plan.target.orientation.y = qy
        req_plan.target.orientation.z = qz
        req_plan.target.orientation.w = qw

        future_plan = self.arm_plan.call_async(req_plan)
        rclpy.spin_until_future_complete(self, future_plan)
        
        if future_plan.result().success:
            print("\n" + "="*50)
            print("  ✅ Plan calculado. Revisa RViz.")
            input("  ▶  Presiona [ENTER] para EJECUTAR el movimiento...")
            print("="*50 + "\n")
            
            self.get_logger().info('Ejecutando movimiento real...')
            req_exec = PlanExec.Request()
            req_exec.wait = True 
            future_exec = self.arm_exec.call_async(req_exec)
            rclpy.spin_until_future_complete(self, future_exec)
            return True
        else:
            self.get_logger().error(f'Colisión o Singularidad al ir a {nombre_pose}')
            return False

    def operar_gripper(self, cerrar=True):
        req_plan = PlanJoint.Request()
        # Pinza de 2 dedos: 0.85 cerrado, 0.0 abierto
        if cerrar:
            self.get_logger().info('>< CERRANDO gripper...')
            req_plan.target = [0.85, 0.85, 0.85, 0.85, 0.85, 0.85]
        else:
            self.get_logger().info('<> ABRIENDO gripper...')
            req_plan.target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        future_plan = self.grip_plan.call_async(req_plan)
        rclpy.spin_until_future_complete(self, future_plan)
        
        if future_plan.result().success:
            req_exec = PlanExec.Request()
            req_exec.wait = True
            future_exec = self.grip_exec.call_async(req_exec)
            rclpy.spin_until_future_complete(self, future_exec)
            time.sleep(0.5) 
        else:
            self.get_logger().error('Fallo al planificar el gripper.')

    def calcular_pose_base(self, p_cam_x, p_cam_y, p_cam_z):
        try:
            tf_stamp = self.tf_buffer.lookup_transform('link_base', 'link5', rclpy.time.Time())
            T_base_brida = tf_stamped_to_matrix(tf_stamp)
            
            p_cam = np.array([p_cam_x, p_cam_y, p_cam_z, 1.0])
            T_base_cam = T_base_brida @ T_BRIDA_CAM
            p_base = T_base_cam @ p_cam
            return p_base[0], p_base[1], p_base[2]
        except TransformException as ex:
            self.get_logger().error(f'Error de TF: {ex}')
            return None

def main(args=None):
    hilo_paro = threading.Thread(target=paro_de_emergencia, daemon=True)
    hilo_paro.start()

    rclpy.init(args=args)
    robot = VisionPickAndPlace()
    
    home_x, home_y, home_z = 0.30, 0.00, 0.40

    try:
        print("\n--- INICIANDO CICLO PICK AND PLACE CON VISIÓN ---")
        robot.mover_brazo(home_x, home_y, home_z, nombre_pose="HOME")
        robot.operar_gripper(cerrar=False)

        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            
            if robot.dado_detectado and not robot.ocupado:
                robot.ocupado = True
                dado = robot.dado_detectado
                robot.get_logger().info(f'Dado detectado por cámara: X:{dado.x:.3f}, Y:{dado.y:.3f}, Z:{dado.z:.3f}')
                
                coords_base = robot.calcular_pose_base(dado.x, dado.y, dado.z)
                if coords_base:
                    bx, by, bz = coords_base
                    robot.get_logger().info(f'Dado en frame Base: X:{bx:.3f}, Y:{by:.3f}, Z:{bz:.3f}')
                    
                    # La brida (link5) debe estar más arriba compensando la herramienta y el offset
                    target_z_pre = bz + GRIPPER_OFFSET_Z + PICK_OFFSET_Z
                    target_z_pick = bz + GRIPPER_OFFSET_Z
                    
                    exito = robot.mover_brazo(bx, by, target_z_pre, nombre_pose="PRE-PICK")
                    if exito:
                        robot.mover_brazo(bx, by, target_z_pick, nombre_pose="PICK")
                        robot.operar_gripper(cerrar=True)
                        robot.mover_brazo(bx, by, target_z_pre, nombre_pose="POST-PICK")
                        
                        robot.mover_brazo(home_x, home_y, home_z, nombre_pose="HOME FINAL")
                        robot.operar_gripper(cerrar=False)
                        
                robot.dado_detectado = None
                robot.ocupado = False

    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
