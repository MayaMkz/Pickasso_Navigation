import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
import cv2
import math
import time
from ultralytics import YOLO
import pyrealsense2 as rs

# Servicios de xArm
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16

class PickAndPlaceStaticNode(Node):
    def __init__(self):
        super().__init__('xarm5_static_pnp')
        
        # --- CONFIGURACIÓN DE YOLO ---
        # TODO: Ajusta la ruta a tu modelo 'best.pt'
        ruta_modelo = '/home/jerry/x_arm/src/xarm_ros2/logica_almacen/logica_almacen/vision/best.pt'
        self.modelo = YOLO(ruta_modelo)
        
        # --- CLIENTES DE SERVICIOS XARM ---
        self.cli_cartesian = self.create_client(MoveCartesian, '/xarm/set_position')
        self.cli_joint = self.create_client(MoveJoint, '/xarm/set_servo_angle')
        self.cli_mode = self.create_client(SetInt16, '/xarm/set_mode')
        self.cli_state = self.create_client(SetInt16, '/xarm/set_state')
        
        self.get_logger().info('Esperando servicios del xArm5...')
        self.cli_cartesian.wait_for_service()
        self.cli_joint.wait_for_service()
        self.cli_mode.wait_for_service()
        self.cli_state.wait_for_service()

        # ========================================================
        # MATRICES HOMOGÉNEAS (De tu configuración anterior)
        # ========================================================
        self.T_brida_camara = np.array([
            [-1.0,  0.0,  0.0,    67.505793],
            [ 0.0, -1.0,  0.0,     7.342000],
            [ 0.0,  0.0,  1.0,    35.899772],
            [ 0.0,  0.0,  0.0,     1.0     ]
        ])

        self.T_base_brida_home = np.array([
            [1.0,  0.0,  0.0,   376.919808],
            [0.0, -1.0,  0.0,    -0.043528],
            [0.0,  0.0, -1.0,   342.996568],
            [0.0,  0.0,  0.0,     1.0     ]
        ])

        self.custom_home_joints = [0.0, -0.349066, -1.13446, 1.48353, 0.0] 
        
        # Variables para manejo de flujo y cámara
        self.pipeline = None
        self.frame_color = None
        self.frame_depth = None
        self.camera_intrinsics = None
        
        self.temp_timer = None # Temporizador para retardos de 1 o 2 segundos
        self._next_step = None # Función que se ejecutará tras el retardo

        # Iniciar la máquina de estados
        self.inicializar_robot()

    # ==========================================
    # UTILIDAD: RETARDOS NO BLOQUEANTES (1 a 2 seg)
    # ==========================================
    def ejecutar_con_retraso(self, segundos, funcion_siguiente):
        self.get_logger().info(f"--- Pausa de {segundos} segundos ---")
        self._next_step = funcion_siguiente
        self.temp_timer = self.create_timer(segundos, self._callback_retraso)

    def _callback_retraso(self):
        self.temp_timer.cancel() # Se ejecuta una sola vez
        self._next_step()

    # ==========================================
    # FASE 1: DESPERTAR Y MOVER A HOME
    # ==========================================
    def inicializar_robot(self):
        self.get_logger().info("Habilitando robot (Modo 0, Estado 0)...")
        req_mode = SetInt16.Request(); req_mode.data = 0
        req_state = SetInt16.Request(); req_state.data = 0
        self.cli_mode.call_async(req_mode)
        
        future_state = self.cli_state.call_async(req_state)
        future_state.add_done_callback(self.state_done_callback)

    def state_done_callback(self, future):
        self.get_logger().info("Robot habilitado. Moviendo a Home...")
        req = MoveJoint.Request()
        req.angles = self.custom_home_joints
        req.speed = 0.25; req.acc = 1.0
        future_home = self.cli_joint.call_async(req)
        future_home.add_done_callback(self.home_done_callback)

    def home_done_callback(self, future):
        self.get_logger().info("Inicio de viaje a Home")
        # Pausa de 1.5 segundos para estabilizar vibraciones mecánicas antes de encender cámara
        self.ejecutar_con_retraso(6.0, self.iniciar_camara)

    # ==========================================
    # FASE 2: CÁMARA (ESPERAR LETRA 'S')
    # ==========================================
    def iniciar_camara(self):
        self.get_logger().info("Encendiendo RealSense...")
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        perfil = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        
        perfil_prof = rs.video_stream_profile(perfil.get_stream(rs.stream.depth))
        self.camera_intrinsics = perfil_prof.get_intrinsics()

        cv2.namedWindow("Presiona 'S' para capturar", cv2.WINDOW_AUTOSIZE)
        
        # Iniciar ciclo rápido para mostrar video en vivo
        self.timer_video = self.create_timer(0.033, self.actualizar_video_vivo)

    def actualizar_video_vivo(self):
        frames = self.pipeline.wait_for_frames()
        frames_alineados = self.align.process(frames)
        depth_frame = frames_alineados.get_depth_frame()
        color_frame = frames_alineados.get_color_frame()

        if not depth_frame or not color_frame: return

        self.frame_depth = np.asanyarray(depth_frame.get_data())
        self.frame_color = np.asanyarray(color_frame.get_data())

        # Mostrar video
        cv2.imshow("Presiona 'S' para capturar", self.frame_color)
        tecla = cv2.waitKey(1) & 0xFF

        if tecla == ord('s') or tecla == ord('S'):
            self.get_logger().info("¡Foto capturada! Apagando cámara...")
            self.timer_video.cancel()
            cv2.destroyAllWindows()
            self.pipeline.stop()
            
            # Pausa de 1 segundo antes de procesar imagen
            self.ejecutar_con_retraso(1.0, self.procesar_foto)

    # ==========================================
    # FASE 3: PROCESAMIENTO ESTÁTICO (Inspirado en UR3e)
    # ==========================================
    def procesar_foto(self):
        self.get_logger().info("Ejecutando YOLOv8 en la captura...")
        resultados = self.modelo(self.frame_color, conf=0.8, verbose=False)
        
        if not resultados or len(resultados[0].boxes) == 0:
            self.get_logger().error("No se detectó ningún cubo en la foto. Retornando a Home...")
            self.ejecutar_con_retraso(2.0, self.iniciar_camara) # Reiniciar ciclo
            return

        # Tomar el primer cubo detectado
        caja = resultados[0].boxes[0]
        x1, y1, x2, y2 = caja.xyxy[0].int().tolist()
        
        centro_x = int((x1 + x2) / 2)
        centro_y = int((y1 + y2) / 2)

        # Lógica de profundidad robusta (Percentil 20 para ignorar el fondo)
        roi = self.frame_depth[y1:y2, x1:x2]
        valores_validos = roi[(roi > 0) & (roi < 10000)]
        
        if valores_validos.size < 20:
            self.get_logger().error("Profundidad inválida (Ruido). Reintentando...")
            self.ejecutar_con_retraso(2.0, self.iniciar_camara)
            return
            
        profundidad_mm = float(np.percentile(valores_validos, 20))
        profundidad_m = profundidad_mm / 1000.0

        # Desproyección RealSense
        punto_3d_camara = rs.rs2_deproject_pixel_to_point(
            self.camera_intrinsics, [centro_x, centro_y], profundidad_m)

        self.get_logger().info(f"Cubo detectado respecto al LENTE: X={punto_3d_camara[0]*1000:.1f} Y={punto_3d_camara[1]*1000:.1f} Z={profundidad_mm:.1f} mm")

        # --- MATEMÁTICA CINEMÁTICA ---
        punto_camara_homo = np.array([punto_3d_camara[0]*1000, punto_3d_camara[1]*1000, profundidad_mm, 1.0])
        T_base_camara = np.dot(self.T_base_brida_home, self.T_brida_camara)
        punto_base = np.dot(T_base_camara, punto_camara_homo)

        x_robot = punto_base[0]
        y_robot = punto_base[1]
        
        longitud_gripper_mm = 160.0  
        margen_seguridad_mm = 100.0  
        z_robot = punto_base[2] + longitud_gripper_mm + margen_seguridad_mm

        # Pausa antes de moverse
        self.ejecutar_con_retraso(1.5, lambda: self.ejecutar_pick(x_robot, y_robot, z_robot))

# ==========================================
    # FASE 4: MOVIMIENTO FÍSICO AL CUBO
    # ==========================================
    def ejecutar_pick(self, x_mm, y_mm, z_mm):
        req = MoveCartesian.Request()
        roll = 3.14159  
        pitch = 0.0
        
        yaw = math.atan2(y_mm, x_mm)
        
        req.pose = [float(x_mm), float(y_mm), float(z_mm), roll, pitch, yaw]
        req.speed = 50.0; req.acc = 50.0
        
        self.get_logger().info(f"Enviando Comando: X:{x_mm:.1f} Y:{y_mm:.1f} Z:{z_mm:.1f} Yaw:{yaw:.2f}rad")
        future = self.cli_cartesian.call_async(req)
        future.add_done_callback(self.pick_done_callback)

    def pick_done_callback(self, future):
        try:
            respuesta = future.result()
            if respuesta.ret != 0:
                self.get_logger().error(f"Movimiento rechazado (Código {respuesta.ret}). Reiniciando...")
                self.ejecutar_con_retraso(2.0, self.iniciar_camara)
                return

            # --- LA CORRECCIÓN CLAVE ---
            # El controlador aceptó la orden. Ahora esperamos el tiempo FÍSICO de viaje.
            self.get_logger().info("Trayectoria aceptada. El robot está descendiendo (esperando 6s)...")
            self.ejecutar_con_retraso(6.0, self.simular_agarre)
            
        except Exception as e:
            self.get_logger().error(f"Fallo en comunicación: {e}")

    # ==========================================
    # FASE 5: AGARRE Y RETORNO
    # ==========================================
    def simular_agarre(self):
        # Esta función solo se dispara cuando el robot ya terminó de bajar físicamente
        self.get_logger().info("Robot en el objetivo. Simulando cierre de gripper (2s)...")
        self.ejecutar_con_retraso(2.0, self.finalizar_agarre)

    def finalizar_agarre(self):
        self.get_logger().info("Agarre completado. Volviendo a Home...")
        req = MoveJoint.Request()
        req.angles = self.custom_home_joints
        req.speed = 0.25; req.acc = 1.0
        future = self.cli_joint.call_async(req)
        
        # Le damos tiempo FÍSICO al robot de volver a subir a Home antes de encender la cámara
        future.add_done_callback(self.home_en_progreso_callback)

    def home_en_progreso_callback(self, future):
        self.get_logger().info("Robot en camino a Home (esperando 6s)...")
        self.ejecutar_con_retraso(6.0, self.iniciar_camara)

def main(args=None):
    rclpy.init(args=args)
    nodo = PickAndPlaceStaticNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        # Parche de seguridad para el apagado de la cámara
        try:
            if nodo.pipeline:
                nodo.pipeline.stop()
        except RuntimeError:
            pass # Ignoramos el error si la cámara nunca logró arrancar
            
        cv2.destroyAllWindows()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
if __name__ == '__main__':
    main()
