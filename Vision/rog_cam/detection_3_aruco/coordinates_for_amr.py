#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math

# NUEVO: Importamos el tipo de mensaje estándar para enviar coordenadas X, Y, Z
from geometry_msgs.msg import Point 

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # --- CONFIGURACIÓN ---
        self.marker_size = 0.063
        self.station_threshold = 0.10
        
        self.aruco_meanings = {
            0: "Robot",
            1: "Estacion 1",
            2: "Estacion 2"
        }

        # --- NUEVO: PUBLICADOR ROS 2 ---
        # Publicaremos en el tópico '/pickasso/vector_meta'
        self.meta_pub = self.create_publisher(Point, '/pickasso/vector_meta', 10)

        # --- INICIALIZACIÓN DE VISIÓN ---
        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        half_size = self.marker_size / 2.0
        self.obj_points = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        # --- CONFIGURACIÓN DE CÁMARA ---
        self.cap = cv2.VideoCapture(4)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara. Verifica la conexión.")
            return

        # Corremos el loop a ~30 FPS (0.033 seg)
        timer_period = 0.033  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("Nodo de detección iniciado. Publicando en /pickasso/vector_meta...")

    def timer_callback(self):
        ret, cv_image_raw = self.cap.read()
        if not ret:
            self.get_logger().warning("Fallo al capturar imagen.")
            return

        if self.camera_matrix is None:
            h, w = cv_image_raw.shape[:2]
            focal_length = w  
            self.camera_matrix = np.array([
                [focal_length, 0, w / 2],
                [0, focal_length, h / 2],
                [0, 0, 1]
            ], dtype=np.float32)
            self.dist_coeffs = np.zeros((4, 1))

        try:
            cv_image = cv2.undistort(cv_image_raw, self.camera_matrix, self.dist_coeffs)
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
            current_poses = {}

            if ids is not None:
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    marker_corners = corners[i][0]

                    success, rvec, tvec = cv2.solvePnP(
                        self.obj_points, marker_corners, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if success:
                        x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]
                        current_poses[marker_id] = (x, y, z, marker_corners)

                        cv2.aruco.drawDetectedMarkers(cv_image, corners)
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size)
                        
                        meaning = self.aruco_meanings.get(marker_id, f"Tag {marker_id}")
                        px_x, px_y = int(marker_corners[0][0]), int(marker_corners[0][1])
                        cv2.putText(cv_image, meaning, (px_x, px_y - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # --- LÓGICA DE ALCANCE Y PUBLICACIÓN ROS 2 ---
                id_robot = 0  
                
                if id_robot in current_poses:
                    rx, ry, rz, _ = current_poses[id_robot]
                    
                    # Supongamos que por ahora queremos ir a la Estación 1
                    estacion_objetivo = 1 
                    
                    if estacion_objetivo in current_poses:
                        sx, sy, sz, s_corners = current_poses[estacion_objetivo]
                        
                        dx = sx - rx
                        dy = sy - ry
                        distancia_total = math.sqrt(dx**2 + dy**2) # Distancia en plano 2D
                        
                        # CREAR Y PUBLICAR EL MENSAJE
                        msg_meta = Point()
                        msg_meta.x = float(dx)
                        msg_meta.y = float(dy)
                        msg_meta.z = float(distancia_total)
                        self.meta_pub.publish(msg_meta) # <--- ¡Enviando datos al aire!
                        
                        px_x = int(s_corners[0][0])
                        px_y = int(s_corners[0][1])
                        
                        if distancia_total < self.station_threshold:
                            cv2.rectangle(cv_image, (px_x - 5, px_y - 65), (px_x + 115, px_y - 35), (0, 0, 255), -1)
                            cv2.putText(cv_image, "REACHED", (px_x, px_y - 45), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                        else:
                            cv2.putText(cv_image, f"Dist: {distancia_total:.2f}m", (px_x, px_y - 45), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Camara", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error procesando: {e}")

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
