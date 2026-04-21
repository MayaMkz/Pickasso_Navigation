#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from xarm_msgs.srv import PlanPose, PlanExec
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
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                tecla = sys.stdin.read(1)
                if tecla.lower() == 'q':
                    print("\r\n\r\n🚨 [PARO DE EMERGENCIA] Tecla 'q' detectada. Abortando programa y cortando servomotores al instante... 🚨\r\n")
                    os._exit(1) 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place_maya')
        
        # --- CLIENTES SÓLO PARA EL BRAZO ---
        self.arm_plan = self.create_client(PlanPose, '/xarm_pose_plan')
        self.arm_exec = self.create_client(PlanExec, '/xarm_exec_plan')

        self.get_logger().info('Conectando con MoveIt y el Controlador del Robot Físico...')
        self.arm_plan.wait_for_service()
        self.arm_exec.wait_for_service()
        self.get_logger().info('¡Conexión establecida con el hardware! PRESIONA "q" PARA PARO DE EMERGENCIA.')

    def mover_brazo(self, x, y, z, nombre_pose="Posición"):
        self.get_logger().info(f'---> Yendo a {nombre_pose}: [X:{x:.2f}, Y:{y:.2f}, Z:{z:.2f}]')
        
        req_plan = PlanPose.Request()
        req_plan.target.position.x = x
        req_plan.target.position.y = y
        req_plan.target.position.z = z
        
        # Orientación viendo hacia abajo
        req_plan.target.orientation.x = 1.0
        req_plan.target.orientation.y = 0.0
        req_plan.target.orientation.z = 0.0
        req_plan.target.orientation.w = 0.0

        future_plan = self.arm_plan.call_async(req_plan)
        rclpy.spin_until_future_complete(self, future_plan)
        
        if future_plan.result().success:
            self.get_logger().info(' Plan calculado. Revisa RViz: tienes 0.5 segundos para abortar...')
            time.sleep(0.5) 
            
            self.get_logger().info(' Moviendo robot físico...')
            req_exec = PlanExec.Request()
            req_exec.wait = True 
            future_exec = self.arm_exec.call_async(req_exec)
            rclpy.spin_until_future_complete(self, future_exec)
            return True
        else:
            self.get_logger().error(f'Colisión o Singularidad al ir a {nombre_pose}')
            return False

def main(args=None):
    # Encender el Botón de Pánico en segundo plano
    hilo_paro = threading.Thread(target=paro_de_emergencia, daemon=True)
    hilo_paro.start()

    rclpy.init(args=args)
    robot = PickAndPlace()

    # Coordenadas
    home_x, home_y, home_z = 0.21, 0.00, 0.3   #0.207,0.0,0.112 zero position 
    pick_x, pick_y, pick_z = 0.21, -0.3, 0.2
    place_x, place_y, place_z = 0.4,-0.2, 0.2
    offset_z = 0.05

    try:
        print("\n--- INICIANDO CICLO (SÓLO BRAZO) REAL ---")
        robot.mover_brazo(home_x, home_y, home_z, "HOME")

        # Secuencia Pick
        robot.mover_brazo(pick_x, pick_y, pick_z + offset_z, "PRE-PICK")
        robot.mover_brazo(pick_x, pick_y, pick_z, "PICK")
        robot.mover_brazo(pick_x, pick_y, pick_z + offset_z, "POST-PICK")
        
        # Secuencia Place
        robot.mover_brazo(place_x, place_y, place_z + offset_z, "PRE-PLACE")
        robot.mover_brazo(place_x, place_y, place_z, "PLACE")
        robot.mover_brazo(place_x, place_y, place_z + offset_z, "POST-PLACE")
        
        robot.mover_brazo(home_x, home_y, home_z, "HOME FINAL")
        
        print("\n✅ ¡CICLO FÍSICO COMPLETADO CON ÉXITO!")

    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
