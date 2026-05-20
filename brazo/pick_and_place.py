import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
import numpy as np

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('xarm5_pick_place')
        
        # --- SUSCRIPTOR ---
        # Escuchamos el tópico que tu nodo de visión está publicando
        self.sub = self.create_subscription(
            Point, 'coordenadas_cubo_3d', self.cubo_callback, 10)
        
        # --- ESTADO DEL ROBOT ---
        # Bandera para ignorar nuevas detecciones mientras el robot está en movimiento
        self.ocupado = False

        # --- MATRIZ DE TRANSFORMACIÓN ---
        # TODO: Reemplaza estos valores con la matriz 4x4 de RoboDK
        # Representa la pose de la cámara respecto a la base del robot
        self.T_cam_to_base = np.array([
            [1.0, 0.0, 0.0,  0.50],  # Fila 1: Rotación X + Traslación X (metros)
            [0.0, 1.0, 0.0, -0.20],  # Fila 2: Rotación Y + Traslación Y (metros)
            [0.0, 0.0, 1.0,  0.80],  # Fila 3: Rotación Z + Traslación Z (metros)
            [0.0, 0.0, 0.0,  1.0 ]   # Fila 4: Homogénea
        ])
        
        self.get_logger().info("Nodo Pick & Place listo. Esperando coordenadas...")

    def cubo_callback(self, msg):
        if self.ocupado:
            return  # Si el robot se está moviendo, ignoramos el video en vivo

        self.get_logger().info(f"Cubo detectado por cámara en X:{msg.x:.3f}, Y:{msg.y:.3f}, Z:{msg.z:.3f}")
        
        # Bloquear recepciones para iniciar la rutina
        self.ocupado = True

        # 1. Aplicar la Transformación Homogénea (De Cámara a Base de Robot)
        punto_camara = np.array([msg.x, msg.y, msg.z, 1.0])
        punto_base = np.dot(self.T_cam_to_base, punto_camara)

        x_robot = punto_base[0]
        y_robot = punto_base[1]
        
        # COMPENSACIÓN FÍSICA: 
        # Z de la cámara es la "tapa" del cubo. Subimos un poco el Z final (ej. 2cm) 
        # para que el Centro de Herramienta (TCP) del gripper no choque con la mesa.
        z_robot = punto_base[2] + 0.02 

        self.get_logger().info(f"Objetivo xArm5 -> X:{x_robot:.3f}, Y:{y_robot:.3f}, Z:{z_robot:.3f}")

        # 2. Configurar y enviar trayectoria
        self.ejecutar_movimiento(x_robot, y_robot, z_robot)

    def ejecutar_movimiento(self, x, y, z):
        pose_objetivo = Pose()
        pose_objetivo.position.x = x
        pose_objetivo.position.y = y
        pose_objetivo.position.z = z

        # --- ORIENTACIÓN (Cuaterniones) ---
        # Recuerda que definir la orientación del gripper es crítico aquí. 
        # Así como solucionaste los errores de alcance (target reach errors) en RoboDK 
        # asegurando que la alineación del robot estuviera en la "Opción A", 
        # el cuaternión en MoveIt debe forzar esa misma configuración articular para 
        # que el brazo de 5 GDL no se bloquee al intentar bajar verticalmente.
        
        pose_objetivo.orientation.x = 0.0 
        pose_objetivo.orientation.y = 1.0 # (Ejemplo: Gripper apuntando 180° hacia abajo)
        pose_objetivo.orientation.z = 0.0
        pose_objetivo.orientation.w = 0.0

        self.get_logger().info("Enviando trayectoria a MoveIt...")

        # AQUÍ SE LLAMA A LA API DE MOVEIT O XARM
        # Dependiendo de tu setup, aquí harías la llamada al Action Client de MoveIt
        # o al servicio /xarm/set_position.
        
        # Simulamos que el movimiento tarda 5 segundos antes de liberar el robot
        self.timer_reinicio = self.create_timer(5.0, self.liberar_robot)

    def liberar_robot(self):
        self.get_logger().info("Rutina terminada. Robot libre, buscando nuevo cubo...")
        self.ocupado = False
        self.timer_reinicio.cancel()

def main(args=None):
    rclpy.init(args=args)
    nodo = PickAndPlaceNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
