#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from xarm_msgs.srv import PlanPose, PlanExec, PlanJoint
import time

# Librerías para el Paro de Emergencia
import sys
import select
import tty
import termios
import threading
import os

def paro_de_emergencia():
    """Hilo de fondo que escucha la tecla 'q' al instante para un E-Stop seguro"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        # setcbreak permite leer la tecla sin presionar Enter
        tty.setcbreak(sys.stdin.fileno())
        while True:
            # Revisa el teclado cada 0.1 segundos
            if select.select([sys.stdin], [], [], 0.1)[0]:
                tecla = sys.stdin.read(1)
                if tecla.lower() == 'q':
                    print("\r\n\r\n🚨 [PARO DE EMERGENCIA] Tecla 'q' detectada. Abortando programa y deteniendo robot al instante... 🚨\r\n")
                    os._exit(1) # Mata el proceso de golpe
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place_maya')
        
        self.arm_plan = self.create_client(PlanPose, '/xarm_pose_plan')
        self.arm_exec = self.create_client(PlanExec, '/xarm_exec_plan')
        self.grip_plan = self.create_client(PlanJoint, '/xarm_gripper_joint_plan')
        self.grip_exec = self.create_client(PlanExec, '/xarm_gripper_exec_plan')

        self.get_logger().info('Conectando con MoveIt y Gazebo...')
        self.arm_plan.wait_for_service()
        self.arm_exec.wait_for_service()
        self.grip_plan.wait_for_service()
        self.grip_exec.wait_for_service()
        self.get_logger().info('¡Conexión establecida! PRESIONA "q" PARA PARO DE EMERGENCIA.')

    def mover_brazo(self, x, y, z, nombre_pose="Posición"):
        self.get_logger().info(f'---> Yendo a {nombre_pose}: [X:{x:.2f}, Y:{y:.2f}, Z:{z:.2f}]')
        
        req_plan = PlanPose.Request()
        req_plan.target.position.x = x
        req_plan.target.position.y = y
        req_plan.target.position.z = z
        req_plan.target.orientation.x = 1.0
        req_plan.target.orientation.y = 0.0
        req_plan.target.orientation.z = 0.0
        req_plan.target.orientation.w = 0.0

        future_plan = self.arm_plan.call_async(req_plan)
        rclpy.spin_until_future_complete(self, future_plan)
        
        if future_plan.result().success:
            
            # ---> PAUSA PARA VER AL FANTASMA <---
            self.get_logger().info('👻 Plan calculado. Esperando 3 segundos para que evalúes la ruta...')
            time.sleep(0.5) 
            
            self.get_logger().info('⚙️ Ejecutando movimiento real...')
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
        
        if cerrar:
            self.get_logger().info('>< CERRANDO gripper...')
            req_plan.target = [0.85, 0.85, 0.85, 0.85, 0.85, 0.85]
        else:
            self.get_logger().info('<> ABRIENDO gripper...')
            req_plan.target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        future_plan = self.grip_plan.call_async(req_plan)
        rclpy.spin_until_future_complete(self, future_plan)
        
        if future_plan.result().success:
            time.sleep(1.0) # Pausa visual para que evalúes la herramienta
            req_exec = PlanExec.Request()
            req_exec.wait = True
            future_exec = self.grip_exec.call_async(req_exec)
            rclpy.spin_until_future_complete(self, future_exec)
            time.sleep(0.5) 
        else:
            self.get_logger().error('Fallo al planificar el gripper.')

def main(args=None):
    # Encender el Botón de Pánico en segundo plano
    hilo_paro = threading.Thread(target=paro_de_emergencia, daemon=True)
    hilo_paro.start()

    rclpy.init(args=args)
    robot = PickAndPlace()

    home_x, home_y, home_z = 0.21, 0.00, 0.112   #0.207,0.0,0.112 zero position 
    pick_x, pick_y, pick_z = 0.21, -0.3, 0.2
    place_x, place_y, place_z = 0.4,-0.2, 0.2
    offset_z = 0.05

    try:
        print("\n--- INICIANDO CICLO PICK AND PLACE ---")
        robot.mover_brazo(home_x, home_y, home_z, "HOME")
        robot.operar_gripper(cerrar=False)

        robot.mover_brazo(pick_x, pick_y, pick_z + offset_z, "PRE-PICK")
        robot.mover_brazo(pick_x, pick_y, pick_z, "PICK")
        robot.operar_gripper(cerrar=True)
        robot.mover_brazo(pick_x, pick_y, pick_z + offset_z, "POST-PICK")
        
        robot.mover_brazo(place_x, place_y, place_z + offset_z, "PRE-PLACE")
        robot.mover_brazo(place_x, place_y, place_z, "PLACE")
        robot.operar_gripper(cerrar=False)
        robot.mover_brazo(place_x, place_y, place_z + offset_z, "POST-PLACE")
        
        robot.mover_brazo(home_x, home_y, home_z, "HOME FINAL")
        
        print("\n✅ ¡CICLO COMPLETADO CON ÉXITO!")

    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
