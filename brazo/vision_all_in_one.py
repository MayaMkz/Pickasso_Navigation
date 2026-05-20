import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2 as rs # SDK para controlar la cámara directamente

class Vision3DNode(Node):
    def __init__(self):
        super().__init__('yolo_realsense_node')
        
        # --- CONFIGURACIÓN DE YOLOV8 ---
        # Corregido: ruta_modelo debe ser un texto (string), no llamar a YOLO() aquí
        ruta_modelo = '/home/jerry/x_arm/src/xarm_ros2/logica_almacen/logica_almacen/vision/best.pt'
        self.modelo = YOLO(ruta_modelo)
        
        # --- INICIALIZACIÓN FÍSICA DE LA CÁMARA REALSENSE ---
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # Habilitar los flujos de color y profundidad (resolución estándar 640x480 a 30fps)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.get_logger().info("Encendiendo la cámara RealSense D457...")
        self.perfil = self.pipeline.start(config)
        
        # Objeto para alinear la profundidad con la imagen a color
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
        # Obtener los intrínsecos directamente del hardware
        perfil_profundidad = rs.video_stream_profile(self.perfil.get_stream(rs.stream.depth))
        self.camera_intrinsics = perfil_profundidad.get_intrinsics()

        # --- PUBLICADOR EN ROS2 ---
        self.publisher_ = self.create_publisher(Point, 'coordenadas_cubo_3d', 10)
        
        # Timer para el ciclo principal (aprox 20 FPS)
        self.timer = self.create_timer(0.05, self.procesar_y_publicar)
        self.get_logger().info("Cámara encendida. Procesando e infiriendo...")

    # --- CICLO PRINCIPAL ---
    def procesar_y_publicar(self):
        # 1. Leer los frames directamente del hardware de la cámara
        try:
            frames = self.pipeline.wait_for_frames()
        except Exception as e:
            self.get_logger().error(f"Error al leer la cámara: {e}")
            return

        # 2. Alinear los frames (crucial para que el píxel X,Y sea el mismo en color y profundidad)
        frames_alineados = self.align.process(frames)
        depth_frame = frames_alineados.get_depth_frame()
        color_frame = frames_alineados.get_color_frame()

        if not depth_frame or not color_frame:
            return

        # 3. Convertir a matrices de Numpy para OpenCV y YOLO
        imagen_profundidad = np.asanyarray(depth_frame.get_data())
        frame = np.asanyarray(color_frame.get_data())

        # 4. Inferencia con YOLO
        resultados = self.modelo(frame, conf=0.8, verbose=False)

        for r in resultados:
            cajas = r.boxes
            for caja in cajas:
                # Bounding Box y Clase
                x1, y1, x2, y2 = caja.xyxy[0].int().tolist()
                nombre_clase = self.modelo.names[int(caja.cls[0])]
                
                # Centro 2D
                centro_x = int((x1 + x2) / 2)
                centro_y = int((y1 + y2) / 2)
                
                # --- FILTRO ESPACIAL DE PROFUNDIDAD (Ventana 5x5) ---
                mitad_ventana = 2
                alto_img, ancho_img = imagen_profundidad.shape
                
                y_min = max(0, centro_y - mitad_ventana)
                y_max = min(alto_img, centro_y + mitad_ventana + 1)
                x_min = max(0, centro_x - mitad_ventana)
                x_max = min(ancho_img, centro_x + mitad_ventana + 1)
                
                region_profundidad = imagen_profundidad[y_min:y_max, x_min:x_max]
                valores_validos = region_profundidad[region_profundidad > 0]
                
                if len(valores_validos) == 0:
                    continue # Ignorar si no hay profundidad válida
                
                profundidad_mm = np.median(valores_validos)
                profundidad_m = profundidad_mm / 1000.0

                # 5. Desproyección: De 2D a 3D espacial (metros)
                punto_3d_camara = rs.rs2_deproject_pixel_to_point(
                    self.camera_intrinsics, [centro_x, centro_y], profundidad_m)

                # --- PUBLICAR EN ROS2 ---
                msg_punto = Point()
                msg_punto.x = float(punto_3d_camara[0])
                msg_punto.y = float(punto_3d_camara[1])
                msg_punto.z = float(punto_3d_camara[2])
                self.publisher_.publish(msg_punto)
                
                # --- DIBUJO SOBRE LA IMAGEN ---
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.circle(frame, (centro_x, centro_y), 4, (0, 0, 255), -1)
                etiqueta = f"{nombre_clase} Z:{profundidad_m:.2f}m"
                cv2.putText(frame, etiqueta, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Mostrar la imagen en vivo en la pantalla
        cv2.imshow("Pickasso - Visión 3D (YOLO + RealSense)", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    nodo = Vision3DNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        # Apagar la cámara correctamente al cerrar
        nodo.pipeline.stop()
        cv2.destroyAllWindows()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
