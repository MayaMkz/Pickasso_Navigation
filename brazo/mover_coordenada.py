
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from xarm_msgs.srv import PlanPose, PlanExec

class ControladorJerry(Node):
    def __init__(self):
        super().__init__('controlador_jerry_xarm5')
        
        self.cliente_plan = self.create_client(PlanPose, '/xarm_pose_plan')
        self.cliente_exec = self.create_client(PlanExec, '/xarm_exec_plan')

        while not self.cliente_plan.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio de planificación...')

    def mover_a_coordenada(self, x, y, z):
        # --- PASO A: DEFINIR EL PUNTO OBJETIVO ---
        request = PlanPose.Request()
        
        request.target.position.x = x
        request.target.position.y = y
        request.target.position.z = z

        request.target.orientation.x = 1.0
        request.target.orientation.y = 0.0
        request.target.orientation.z = 0.0
        request.target.orientation.w = 0.0

        self.get_logger().info(f'Calculando ruta hacia: X={x}, Y={y}, Z={z}...')

        # --- PASO B: PEDIR A MOVEIT QUE CALCULE LA RUTA ---
        future = self.cliente_plan.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        
        if response.success:
            self.get_logger().info('¡Ruta encontrada!')
            
            # ---------------------------------------------------------
            # --- AQUÍ ESTÁ EL TRUCO: PAUSAR ANTES DE EJECUTAR ---
            # ---------------------------------------------------------
            print("\n" + "="*10)
            print(" ¡Ve a RViz! Deberías estar viendo el robot fantasma moverse.")
            input(" Presiona [ENTER] en esta terminal cuando quieras que el robot real/Gazebo lo ejecute...")
            print("="*10 + "\n")
            
            # --- PASO C: EJECUTAR EL MOVIMIENTO ---
            self.get_logger().info('Ejecutando movimiento...')
            exec_req = PlanExec.Request()
            exec_req.wait = True 
            
            exec_future = self.cliente_exec.call_async(exec_req)
            rclpy.spin_until_future_complete(self, exec_future)
            self.get_logger().info('Movimiento completado.')
        else:
            self.get_logger().error('No se pudo encontrar una ruta válida.')

def main(args=None):
    rclpy.init(args=args)
    robot = ControladorJerry()

    try:
        # Prueba con otra coordenada para que veas la diferencia
        robot.mover_a_coordenada(0.4,-0.2,0.112)#(0.207,0.0,0.112)
    except KeyboardInterrupt:
        pass

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
