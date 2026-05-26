import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped # <-- NUEVO MENSAJE
import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2 as rs 
import cv2.aruco as aruco 
import math

class Vision3DNode(Node):
    def __init__(self):
        super().__init__('yolo_realsense_node')
        
        ruta_modelo = '/home/jerry/x_arm/src/xarm_ros2/logica_almacen/logica_almacen/vision/best.pt'
        self.modelo = YOLO(ruta_modelo)
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.get_logger().info("Encendiendo la cámara RealSense D457...")
        self.perfil = self.pipeline.start(config)
        
        sensor_profundidad = self.perfil.get_device().first_depth_sensor()
        self.depth_scale = sensor_profundidad.get_depth_scale()
        
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
        perfil_color = rs.video_stream_profile(self.perfil.get_stream(rs.stream.color))
        self.camera_intrinsics = perfil_color.get_intrinsics()

        # Usamos PointStamped para enviar la coordenada + el nombre
        self.pub_cubo = self.create_publisher(PointStamped, 'coordenadas_cubo_3d', 10)
        self.pub_aruco = self.create_publisher(PointStamped, 'coordenadas_arucos', 10)

        self.nombre_ventana = "Vision 3D"
        cv2.namedWindow(self.nombre_ventana, cv2.WINDOW_AUTOSIZE)
        
        self.timer = self.create_timer(0.01, self.procesar_y_publicar)
        self.get_logger().info("Cámara encendida. Detección Multi-Color activa...")

    def procesar_y_publicar(self):
        try:
            frames = self.pipeline.wait_for_frames()
        except Exception as e:
            return

        frames_alineados = self.align.process(frames)
        depth_frame = frames_alineados.get_depth_frame()
        color_frame = frames_alineados.get_color_frame()

        if not depth_frame or not color_frame: return

        imagen_profundidad = np.asanyarray(depth_frame.get_data())
        frame = np.asanyarray(color_frame.get_data())

        # ──────────────────────────────────────────────────────────
        # TAREA 1: DETECCIÓN DE TODOS LOS ARUCOS
        # ──────────────────────────────────────────────────────────
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        esquinas, ids, _ = self.aruco_detector.detectMarkers(gray)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, esquinas, ids)
            for i in range(len(ids)):
                id_actual = ids[i][0]
                c_esquinas = esquinas[i][0]
                centro_x_ar = int(np.mean(c_esquinas[:, 0]))
                centro_y_ar = int(np.mean(c_esquinas[:, 1]))
                
                mitad_vent_ar = 2
                y_min_ar = max(0, centro_y_ar - mitad_vent_ar)
                y_max_ar = min(imagen_profundidad.shape[0], centro_y_ar + mitad_vent_ar + 1)
                x_min_ar = max(0, centro_x_ar - mitad_vent_ar)
                x_max_ar = min(imagen_profundidad.shape[1], centro_x_ar + mitad_vent_ar + 1)
                
                region_prof_ar = imagen_profundidad[y_min_ar:y_max_ar, x_min_ar:x_max_ar]
                valores_validos_ar = region_prof_ar[region_prof_ar > 0]
                
                if len(valores_validos_ar) > 0:
                    prof_bruta_ar = np.median(valores_validos_ar)
                    prof_m_ar = prof_bruta_ar * self.depth_scale
                    
                    punto_3d_ar = rs.rs2_deproject_pixel_to_point(
                        self.camera_intrinsics, [centro_x_ar, centro_y_ar], prof_m_ar)
                        
                    # NUEVO: PointStamped para ArUcos
                    msg_aruco = PointStamped()
                    msg_aruco.header.stamp = self.get_clock().now().to_msg()
                    msg_aruco.header.frame_id = str(id_actual) # Inyectamos el ID como string
                    msg_aruco.point.x = float(punto_3d_ar[0])
                    msg_aruco.point.y = float(punto_3d_ar[1])
                    msg_aruco.point.z = float(punto_3d_ar[2])
                    self.pub_aruco.publish(msg_aruco)
                    
                    cv2.circle(frame, (centro_x_ar, centro_y_ar), 5, (255, 0, 0), -1)

        # ──────────────────────────────────────────────────────────
        # TAREA 2: DETECCIÓN DE CUBO (Filtrando el más centrado)
        # ──────────────────────────────────────────────────────────
        resultados = self.modelo(frame, conf=0.8, verbose=False)
        mejor_caja = None
        menor_distancia_centro = float('inf')

        for r in resultados:
            for caja in r.boxes:
                x1, y1, x2, y2 = caja.xyxy[0].int().tolist()
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                
                distancia = math.hypot(cx - 320, cy - 240)
                if distancia < menor_distancia_centro:
                    menor_distancia_centro = distancia
                    mejor_caja = caja

        if mejor_caja is not None:
            x1, y1, x2, y2 = mejor_caja.xyxy[0].int().tolist()
            nombre_clase = self.modelo.names[int(mejor_caja.cls[0])]
            
            centro_x = int((x1 + x2) / 2)
            centro_y = int((y1 + y2) / 2)
            
            mitad_ventana = 2
            y_min = max(0, centro_y - mitad_ventana)
            y_max = min(imagen_profundidad.shape[0], centro_y + mitad_ventana + 1)
            x_min = max(0, centro_x - mitad_ventana)
            x_max = min(imagen_profundidad.shape[1], centro_x + mitad_ventana + 1)
            
            region_profundidad = imagen_profundidad[y_min:y_max, x_min:x_max]
            valores_validos = region_profundidad[region_profundidad > 0]
            
            if len(valores_validos) > 0:
                profundidad_bruta = np.median(valores_validos)
                profundidad_m = profundidad_bruta * self.depth_scale

                punto_3d_camara = rs.rs2_deproject_pixel_to_point(
                    self.camera_intrinsics, [centro_x, centro_y], profundidad_m)

                # NUEVO: PointStamped para el Cubo
                msg_punto = PointStamped()
                msg_punto.header.stamp = self.get_clock().now().to_msg()
                msg_punto.header.frame_id = nombre_clase # Inyectamos el color (ej: "rosa")
                msg_punto.point.x = float(punto_3d_camara[0])
                msg_punto.point.y = float(punto_3d_camara[1])
                msg_punto.point.z = float(punto_3d_camara[2])
                self.pub_cubo.publish(msg_punto)
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.putText(frame, f"TARGET: {nombre_clase}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow(self.nombre_ventana, frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    nodo = Vision3DNode()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally:
        nodo.pipeline.stop()
        cv2.destroyAllWindows()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
