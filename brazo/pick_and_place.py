import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np

# Agregamos SetInt16, que es el tipo de mensaje que usan Mode y State
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('xarm5_pick_place')
        
        # --- CLIENTES DE SERVICIOS XARM ---
        self.cli_cartesian = self.create_client(MoveCartesian, '/xarm/set_position')
        self.cli_joint = self.create_client(MoveJoint, '/xarm/set_servo_angle')
        self.cli_mode = self.create_client(SetInt16, '/xarm/set_mode')
        self.cli_state = self.create_client(SetInt16, '/xarm/set_state')
        
        # Esperar a que los servicios base estén activos
        self.get_logger().info('Esperando a los servicios del xArm...')
        self.cli_cartesian.wait_for_service()
        self.cli_joint.wait_for_service()
        self.cli_mode.wait_for_service()
        self.cli_state.wait_for_service()
        
        # --- SUSCRIPTOR DE VISIÓN ---
        self.sub = self.create_subscription(
            Point, 'coordenadas_cubo_3d', self.cubo_callback, 10)
        
        # Candado de seguridad puesto al iniciar
        self.ocupado = True 

        # --- MATRIZ DE TRANSFORMACIÓN ---
        self.T_cam_to_base = np.array([
            [1.0, 0.0, 0.0,  274.506207],
            [0.0, -1.0, 0.0, -7.3420],
            [0.0, 0.0, -1.0,  76.101772],
            [0.0, 0.0, 0.0,  1.0 ]   
        ])

        # --- CONFIGURACIÓN DE HOME ---
        self.custom_home_joints = [0.0, -1.309, -0.349066, 0.349066, 0.0] 
        
        # Iniciar cadena de activación del robot
        self.inicializar_robot()

    # ==========================================
    # LÓGICA DE INICIALIZACIÓN FÍSICA (DESPERTAR)
    # ==========================================
    def inicializar_robot(self):
        self.get_logger().info("Configurando Modo 0 (Posición)...")
        req_mode = SetInt16.Request()
        req_mode.data = 0
        future_mode = self.cli_mode.call_async(req_mode)
        future_mode.add_done_callback(self.mode_done_callback)

    def mode_done_callback(self, future):
        try:
            future.result()
            self.get_logger().info("Habilitando motores (Estado 0)...")
            req_state = SetInt16.Request()
            req_state.data = 0
            future_state = self.cli_state.call_async(req_state)
            future_state.add_done_callback(self.state_done_callback)
        except Exception as e:
            self.get_logger().error(f"Error al cambiar Modo: {e}")

    def state_done_callback(self, future):
        try:
            future.result()
            self.get_logger().info("Robot habilitado y listo. Iniciando movimiento a Home...")
            # Ahora que el robot está habilitado, lo mandamos a Home
            self.ir_a_home()
        except Exception as e:
            self.get_logger().error(f"Error al cambiar Estado: {e}")

    # ==========================================
    # LÓGICA DE MOVIMIENTO A HOME
    # ==========================================
    def ir_a_home(self):
        req = MoveJoint.Request()
        req.angles = self.custom_home_joints
        req.speed = 0.25  # rad/s
        req.mvacc = 1.0   # rad/s^2
        
        future = self.cli_joint.call_async(req)
        future.add_done_callback(self.home_done_callback)

    def home_done_callback(self, future):
        try:
            future.result()
            self.get_logger().info("Home alcanzado de forma segura. Escuchando detecciones de YOLO...")
            self.ocupado = False
        except Exception as e:
            self.get_logger().error(f"Error al mover a Home: {e}")

    # ==========================================
    # LÓGICA DE RECEPCIÓN DE VISIÓN
    # ==========================================
    def cubo_callback(self, msg):
        if self.ocupado:
            return

        self.get_logger().info(f"Cubo detectado -> X:{msg.x:.3f}, Y:{msg.y:.3f}, Z:{msg.z:.3f}")
        self.ocupado = True

        punto_camara = np.array([msg.x, msg.y, msg.z, 1.0])
        punto_base = np.dot(self.T_cam_to_base, punto_camara)

        x_robot = punto_base[0]
        y_robot = punto_base[1]
        
        # Z seguro de prueba (15 cm arriba del cubo)
        z_robot = punto_base[2] + 0.3

        self.ejecutar_pick(x_robot, y_robot, z_robot)

    # ==========================================
    # LÓGICA DE MOVIMIENTO CARTESIANO (PICK)
    # ==========================================
    def ejecutar_pick(self, x, y, z):
        req = MoveCartesian.Request()
        
        x_mm = float(x * 1000.0)
        y_mm = float(y * 1000.0)
        z_mm = float(z * 1000.0)
        
        # Orientación Opción A para brazo de 5GDL
        roll = 3.14159  
        pitch = 0.0
        yaw = 0.0
        
        req.pose = [x_mm, y_mm, z_mm, roll, pitch, yaw]
        req.speed = 100.0  
        req.mvacc = 1000.0 
        
        self.get_logger().info(f"Enviando trayectoria: X:{x_mm:.1f} Y:{y_mm:.1f} Z:{z_mm:.1f}")
        future = self.cli_cartesian.call_async(req)
        future.add_done_callback(self.pick_done_callback)

    def pick_done_callback(self, future):
        try:
            future.result()
            self.get_logger().info("Llegó a la posición segura sobre el cubo. Simulando agarre...")
            self.timer_agarre = self.create_timer(2.0, self.finalizar_agarre)
        except Exception as e:
            self.get_logger().error(f"Fallo en trayectoria cartesiana: {e}")
            self.ocupado = False

    def finalizar_agarre(self):
        self.timer_agarre.cancel()
        self.get_logger().info("Agarre completado. Retornando a Home...")
        self.ir_a_home()

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
