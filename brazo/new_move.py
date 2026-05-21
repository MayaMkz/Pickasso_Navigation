import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
import cv2
import math
import time
from ultralytics import YOLO
import pyrealsense2 as rs

# --- Servicios oficiales del xArm ---
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16, ClearErr

class PickAndPlaceStaticNode(Node):
    def __init__(self):
        super().__init__('xarm5_static_pnp')
        
        # --- CONFIGURACIÓN DE YOLO ---
        ruta_modelo = '/home/jerry/x_arm/src/xarm_ros2/logica_almacen/logica_almacen/vision/best.pt'
        self.modelo = YOLO(ruta_modelo)
        
        # --- CLIENTES DE SERVICIOS XARM ---
        self.cli_cartesian = self.create_client(MoveCartesian, '/xarm/set_position')
        self.cli_joint = self.create_client(MoveJoint, '/xarm/set_servo_angle')
        self.cli_mode = self.create_client(SetInt16, '/xarm/set_mode')
        self.cli_state = self.create_client(SetInt16, '/xarm/set_state')
        self.cli_clear_err = self.create_client(ClearErr, '/xarm/clear_err')
        
        self.get_logger().info('Esperando servicios del xArm5...')
        self.cli_cartesian.wait_for_service()
        self.cli_joint.wait_for_service()
        self.cli_mode.wait_for_service()
        self.cli_state.wait_for_service()
        self.cli_clear_err.wait_for_service()

        # --- CONFIGURACIÓN DE HOME ---
        self.custom_home_joints = [0.0, -0.349066, -1.13446, 1.48353, 0.0] 
        
        # Variables de estado
        self.pipeline = None
        self.frame_color = None
        self.frame_depth = None
        self.camera_intrinsics = None
        
        self.temp_timer = None 
        self._next_step = None 

        # Iniciar secuencia
        self.inicializar_robot()

    # ==========================================
    # UTILIDAD: RETARDOS NO BLOQUEANTES
    # ==========================================
    def ejecutar_con_retraso(self, segundos, funcion_siguiente):
        self._next_step = funcion_siguiente
        self.temp_timer = self.create_timer(segundos, self._callback_retraso)

    def _callback_retraso(self):
        self.temp_timer.cancel() 
        self._next_step()

    # ==========================================
    # FASE 1: DESPERTAR Y MOVER A HOME
    # ==========================================
    def inicializar_robot(self):
        self.get_logger().info("Limpiando errores previos y habilitando robot...")
        # 1. Limpiar cualquier error físico anterior (como el C21 o colisiones)
        self.cli_clear_err.call_async(ClearErr.Request())
        
        # 2. Configurar modo y estado
        req_mode = SetInt16.Request(); req_mode.data = 0
        req_state = SetInt16.Request(); req_state.data = 0
        self.cli_mode.call_async(req_mode)
        
        future_state = self.cli_state.call_async(req_state)
        future_state.add_done_callback(self.state_done_callback)

    def state_done_callback(self, future):
        self.get_logger().info("Robot habilitado. Moviendo a Home (esperando 4s)...")
        req = MoveJoint.Request()
        req.angles = self.custom_home_joints
        req.speed = 0.25; req.acc = 1.0
        future_home = self.cli_joint.call_async(req)
        
        # Esperamos físicamente a que llegue al Home antes de prender la cámara
        self.ejecutar_con_retraso(4.0, self.iniciar_camara)

    # ==========================================
    # FASE 2: CÁMARA (ESPERAR LETRA 'S')
    # ==========================================
    def iniciar_camara(self):
        self.get_logger().info("Encendiendo RealSense...")
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            perfil = self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            
            perfil_prof = rs.video_stream_profile(perfil.get_stream(rs.stream.depth))
            self.camera_intrinsics = perfil_prof.get_intrinsics()
        except Exception as e:
            self.get_logger().error(f"Error al iniciar cámara: {e}. Desconecta y reconecta el USB.")
            return

        cv2.namedWindow("Presiona 'S' para capturar", cv2.WINDOW_AUTOSIZE)
        self.timer_video = self.create_timer(0.033, self.actualizar_video_vivo)

    def actualizar_video_vivo(self):
        frames = self.pipeline.wait_for_frames()
        frames_alineados = self.align.process(frames)
        depth_frame = frames_alineados.get_depth_frame()
        color_frame = frames_alineados.get_color_frame()

        if not depth_frame or not color_frame: return

        self.frame_depth = np.asanyarray(depth_frame.get_data())
        self.frame_color = np.asanyarray(color_frame.get_data())

        cv2.imshow("Presiona 'S' para capturar", self.frame_color)
        tecla = cv2.waitKey(1) & 0xFF

        if tecla == ord('s') or tecla == ord('S'):
            self.get_logger().info("¡Foto capturada! Apagando cámara (1s)...")
            self.timer_video.cancel()
            cv2.destroyAllWindows()
            self.pipeline.stop()
            self.pipeline = None
            
            self.ejecutar_con_retraso(1.0, self.procesar_foto)

    # ==========================================
    # FASE 3: PROCESAMIENTO ESTÁTICO Y MAPEO
    # ==========================================
    def procesar_foto(self):
        self.get_logger().info("Ejecutando YOLOv8 en la captura...")
        resultados = self.modelo(self.frame_color, conf=0.8, verbose=False)
        
        if not resultados or len(resultados[0].boxes) == 0:
            self.get_logger().error("No se detectó ningún cubo. Reintentando...")
            self.iniciar_camara()
            return

        caja = resultados[0].boxes[0]
        x1, y1, x2, y2 = caja.xyxy[0].int().tolist()
        centro_x = int((x1 + x2) / 2)
        centro_y = int((y1 + y2) / 2)

        roi = self.frame_depth[y1:y2, x1:x2]
        valores_validos = roi[(roi > 0) & (roi < 10000)]
        
        if valores_validos.size < 20:
            self.get_logger().error("Ruido en la profundidad. Reintentando...")
            self.iniciar_camara()
            return
            
        profundidad_mm = float(np.percentile(valores_validos, 20))
        profundidad_m = profundidad_mm / 1000.0

        # FILTRO ANTI-BASURA (Evitar lecturas de 5 metros cuando choca)
        if profundidad_mm > 1500.0 or profundidad_mm < 150.0:
            self.get_logger().warn(f"Error óptico: Z={profundidad_mm:.1f} mm está fuera de rango.")
            self.iniciar_camara()
            return

        punto_3d_camara = rs.rs2_deproject_pixel_to_point(
            self.camera_intrinsics, [centro_x, centro_y], profundidad_m)

        x_cam_mm = punto_3d_camara[0] * 1000.0
        y_cam_mm = punto_3d_camara[1] * 1000.0

        self.get_logger().info(f"Lente -> X:{x_cam_mm:.1f} Y:{y_cam_mm:.1f} Z:{profundidad_mm:.1f}")

        # ======================================================
        # MAPEO ALGEBRAICO DIRECTO (Sustituye a las matrices)
        # ======================================================
        # 1. Posición real de tu lente en el espacio (Brida Home + Desfase Físico)
        X_lente = 376.9 + 67.5  # 444.4 mm
        Y_lente = -0.04 + 7.3   # 7.26 mm
        Z_lente = 343.0 + 35.9  # 378.9 mm

        # 2. Mapeo de Ejes (Ajusta el 1.0 a -1.0 si el robot se mueve al lado contrario)
        offset_x = y_cam_mm * 1.0  
        offset_y = x_cam_mm * 1.0  
        
        x_robot = X_lente + offset_x
        y_robot = Y_lente + offset_y

        # 3. Altura absoluta desde la mesa
        z_cubo_absoluto = Z_lente - profundidad_mm
        
        # --- ATENCIÓN ---
        # Si el tamaño de tu gripper ya está configurado en el UFactory Studio (TCP), 
        # debes dejar longitud_gripper_mm en 0.0, o el robot bajará el doble.
        longitud_gripper_mm = 0.0  
        margen_seguridad_mm = 100.0  
        z_robot = z_cubo_absoluto + longitud_gripper_mm + margen_seguridad_mm

        self.ejecutar_con_retraso(1.0, lambda: self.ejecutar_pick(x_robot, y_robot, z_robot))

    # ==========================================
    # FASE 4: MOVIMIENTO FÍSICO AL CUBO
    # ==========================================
    def ejecutar_pick(self, x_mm, y_mm, z_mm):
        req = MoveCartesian.Request()
        roll = 3.14159  
        pitch = 0.0
        
        # El Yaw dinámico evita el Error C21 (Kinematic Error) en brazos 5-DOF
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
                self.get_logger().error(f"Movimiento rechazado (Código {respuesta.ret}).")
                self.get_logger().info("Limpiando errores y reiniciando cámara en 3s...")
                self.cli_clear_err.call_async(ClearErr.Request())
                self.ejecutar_con_retraso(3.0, self.iniciar_camara)
                return

            self.get_logger().info("Trayectoria aceptada. Descendiendo (esperando 6s)...")
            self.ejecutar_con_retraso(6.0, self.simular_agarre)
            
        except Exception as e:
            self.get_logger().error(f"Fallo en comunicación: {e}")

    # ==========================================
    # FASE 5: AGARRE Y RETORNO
    # ==========================================
    def simular_agarre(self):
        self.get_logger().info("Robot en objetivo. Simulando cierre de gripper (2s)...")
        self.ejecutar_con_retraso(2.0, self.finalizar_agarre)

    def finalizar_agarre(self):
        self.get_logger().info("Agarre completado. Volviendo a Home (esperando 6s)...")
        
        # Limpiar cualquier advertencia del controlador antes de subir
        self.cli_clear_err.call_async(ClearErr.Request())
        req_state = SetInt16.Request(); req_state.data = 0
        self.cli_state.call_async(req_state)

        req = MoveJoint.Request()
        req.angles = self.custom_home_joints
        req.speed = 0.25; req.acc = 1.0
        self.cli_joint.call_async(req)
        
        # Esperamos físicamente 6 segundos a que el brazo suba antes de reiniciar la cámara
        self.ejecutar_con_retraso(6.0, self.iniciar_camara)

def main(args=None):
    rclpy.init(args=args)
    nodo = PickAndPlaceStaticNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        # Parche de seguridad para liberar el puerto USB al cerrar con Ctrl+C
        try:
            if nodo.pipeline:
                nodo.pipeline.stop()
        except RuntimeError:
            pass 
            
        cv2.destroyAllWindows()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
