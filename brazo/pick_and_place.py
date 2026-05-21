import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
import math

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

        # ========================================================
        # LÓGICA DE TRANSFORMADAS HOMOGÉNEAS (EYE-IN-HAND)
        # ========================================================
        
        # 1. T_brida_camara: Desfase FÍSICO FIJO y ROTACIÓN de la cámara respecto al efector final.
        # Los -1.0 en X y Y absorben nativamente el hecho de que la cámara está invertida (girada 180° en Z)
        self.T_brida_camara = np.array([
            [-1.0,  0.0,  0.0,    67.505793],  # X invertido nativamente
            [ 0.0, -1.0,  0.0,     7.342000],  # Y invertido nativamente
            [ 0.0,  0.0,  1.0,    35.899772],  
            [ 0.0,  0.0,  0.0,     1.0     ]
        ])

        # 2. T_base_brida_home: Pose de la brida respecto a la base del robot en la postura de lectura.
        self.T_base_brida_home = np.array([
            [1.0,  0.0,  0.0,       376.919808],  
            [0.0, -1.0, -0.000174,   -0.043528],
            [0.0,  0.0, -1.0,       342.996568],  # Asegúrate de que esta altura de lectura sea > 400mm si da problemas el sensor
            [0.0,  0.0,  0.0,         1.0     ]
        ])

        # --- CONFIGURACIÓN DE HOME ---
        self.custom_home_joints = [0.0, -0.349066, -1.13446, 1.48353, 0.0] 
        
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
        req.acc = 1.0   # rad/s^2
        
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

        x_cam_mm = msg.x * 1000.0
        y_cam_mm = msg.y * 1000.0
        z_cam_mm = msg.z * 1000.0

        # --- FILTRO ANTI-BASURA (NUEVO) ---
        # Si la cámara lee que el cubo está a más de 1.5 metros o a menos de 15 cm, es un error del sensor.
        if z_cam_mm > 1500.0 or z_cam_mm < 150.0:
            self.get_logger().warn(f"Lectura óptica ignorada por seguridad (Z={z_cam_mm:.1f} mm)")
            self.ocupado = False
            return

        # --- CINEMÁTICA EYE-IN-HAND ---
        # 1. El vector homogéneo respecto al lente de la cámara
        punto_camara = np.array([x_cam_mm, y_cam_mm, z_cam_mm, 1.0])
        
        # 2. Calcular matriz de transformación total: T_base_camara = T_base_brida * T_brida_camara
        T_base_camara = np.dot(self.T_base_brida_home, self.T_brida_camara)
        
        # 3. Proyectar el vector del cubo hacia el origen de la base del robot
        punto_base = np.dot(T_base_camara, punto_camara)

        x_robot = punto_base[0]
        y_robot = punto_base[1]

        # Compensación geométrica del efector final
        longitud_gripper_mm = 160.0  
        margen_seguridad_mm = 100.0  
        
        # Cálculo del Z objetivo (10 cm por encima del cubo)
        z_robot = punto_base[2] + longitud_gripper_mm + margen_seguridad_mm

        self.ejecutar_pick(x_robot, y_robot, z_robot)

    # ==========================================
    # LÓGICA DE MOVIMIENTO CARTESIANO (PICK)
    # ==========================================
    def ejecutar_pick(self, x_mm, y_mm, z_mm):
        req = MoveCartesian.Request()
        
        # Orientación Opción A para brazo de 5GDL
        roll = 3.14159  
        pitch = 0.0

        # El Yaw debe coincidir exactamente con el giro de la base (Motor 1)
        # Usamos atan2(Y, X) para calcular ese ángulo en radianes.
        yaw = math.atan2(y_mm, x_mm)
        
        req.pose = [float(x_mm), float(y_mm), float(z_mm), roll, pitch, yaw]
        req.speed = 50.0  
        req.acc = 50.0
        
        self.get_logger().info(f"Enviando trayectoria: X:{x_mm:.1f} Y:{y_mm:.1f} Z:{z_mm:.1f} Yaw:{yaw:.2f} rad")
        future = self.cli_cartesian.call_async(req)
        future.add_done_callback(self.pick_done_callback)

    def pick_done_callback(self, future):
        try:
            respuesta = future.result()
            
            if respuesta.ret != 0:
                self.get_logger().error(f"Movimiento rechazado por xArm")
                self.get_logger().error(f"Codigo de error: {respuesta.ret} - Mensaje: {respuesta.message}")
                self.ocupado = False
                return

            self.get_logger().info("Llegó a la posición segura sobre el cubo. Simulando agarre...")
            self.timer_agarre = self.create_timer(2.0, self.finalizar_agarre)
        except Exception as e:
            self.get_logger().error(f"Fallo en comunicación ROS2: {e}")
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
