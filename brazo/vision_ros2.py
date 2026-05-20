import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2 as rs # Necesario para la desproyección matemática

class Vision3DNode(Node):
    def __init__(self):
        super().__init__('yolo_realsense_node')
        
        # --- CONFIGURACIÓN DE YOLOV8 ---
        # Asegúrate de actualizar la ruta
        ruta_modelo = '/ruta/en/ubuntu/hacia/tu/best.pt'
        self.modelo = YOLO(ruta_modelo)
        
        # Herramienta para convertir imágenes de ROS a OpenCV
        self.bridge = CvBridge()
        
        # Variables para almacenar los datos recibidos
        self.imagen_color = None
        self.imagen_profundidad = None
        self.camera_intrinsics = None

        # --- PUBLICADOR ---
        # Ahora publicamos las coordenadas (X, Y, Z) en metros respecto a la cámara
        self.publisher_ = self.create_publisher(Point, 'coordenadas_cubo_3d', 10)
        
        # --- SUSCRIPTORES ---
        # Suscripción a la imagen a color
        self.sub_color = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, 10)
            
        # Suscripción a la imagen de profundidad ALINEADA al color
        self.sub_depth = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
            
        # Suscripción a la información de la cámara (para obtener la matriz de intrínsecos)
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.info_callback, 10)

        # Timer para el ciclo principal de procesamiento
        self.timer = self.create_timer(0.05, self.procesar_y_publicar)
        self.get_logger().info("Nodo YOLO+RealSense iniciado. Esperando imágenes...")

    # --- CALLBACKS DE RECEPCIÓN DE DATOS ---
    def color_callback(self, msg):
        # Convertir mensaje de ROS a imagen de OpenCV (BGR para YOLO/cv2)
        self.imagen_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        # Convertir mensaje a imagen de profundidad (16-bit)
        self.imagen_profundidad = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def info_callback(self, msg):
        # Solo necesitamos almacenar los intrínsecos una vez
        if self.camera_intrinsics is None:
            self.camera_intrinsics = rs.intrinsics()
            self.camera_intrinsics.width = msg.width
            self.camera_intrinsics.height = msg.height
            self.camera_intrinsics.ppx = msg.k[2]
            self.camera_intrinsics.ppy = msg.k[5]
            self.camera_intrinsics.fx = msg.k[0]
            self.camera_intrinsics.fy = msg.k[4]
            # Modelo de distorsión (plumb_bob en ROS se aproxima a Brown Conrady en RealSense)
            self.camera_intrinsics.model = rs.distortion.brown_conrady 
            self.camera_intrinsics.coeffs = list(msg.d)
            self.get_logger().info("Intrínsecos de la cámara recibidos.")

    # --- CICLO PRINCIPAL ---
    def procesar_y_publicar(self):
        # Verificar que tenemos todos los datos necesarios
        if self.imagen_color is None or self.imagen_profundidad is None or self.camera_intrinsics is None:
            return

        # Hacer una copia para no alterar los datos mientras recibimos nuevos frames
        frame = self.imagen_color.copy()
        depth_frame = self.imagen_profundidad.copy()

        # PASO 1: Inferencia con YOLO
        resultados = self.modelo(frame, conf=0.8, verbose=False)

        for r in resultados:
            cajas = r.boxes
            for caja in cajas:
                # 1. Bounding Box
                x1, y1, x2, y2 = caja.xyxy[0].int().tolist()
                nombre_clase = self.modelo.names[int(caja.cls[0])]
                
                # 2. Centro en 2D (píxeles)
                centro_x = int((x1 + x2) / 2)
                centro_y = int((y1 + y2) / 2)
                
                # 3. Extraer la profundidad Z en ese píxel
                # La RealSense D457 suele entregar la profundidad en milímetros
                profundidad_mm = depth_frame[centro_y, centro_x]
                
                # Filtrar lecturas inválidas (0 = no se pudo calcular la profundidad)
                if profundidad_mm == 0:
                    continue
                
                # Convertir a metros
                profundidad_m = profundidad_mm / 1000.0

                # 4. Desproyección: De píxeles 2D a coordenadas espaciales 3D (X, Y, Z)
                punto_3d_camara = rs.rs2_deproject_pixel_to_point(
                    self.camera_intrinsics, [centro_x, centro_y], profundidad_m)
                
                # punto_3d_camara es una lista [X, Y, Z] en metros, relativas al sensor de la cámara

                # --- PUBLICAR EN ROS2 ---
                msg_punto = Point()
                msg_punto.x = float(punto_3d_camara[0])
                msg_punto.y = float(punto_3d_camara[1])
                msg_punto.z = float(punto_3d_camara[2])
                self.publisher_.publish(msg_punto)
                
                self.get_logger().info(f"{nombre_clase} a Z={profundidad_m:.3f}m -> 3D(X:{msg_punto.x:.3f}, Y:{msg_punto.y:.3f})")

                # --- DIBUJO SOBRE LA IMAGEN ---
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.circle(frame, (centro_x, centro_y), 4, (0, 0, 255), -1)
                etiqueta = f"{nombre_clase} Z:{profundidad_m:.2f}m"
                cv2.putText(frame, etiqueta, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Mostrar para depuración
        cv2.imshow("YOLOv8 + RealSense D457", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    nodo = Vision3DNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
