#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # --- CONFIGURACIÓN ---
        # Tamaño real del ArUco/AprilTag en metros (0.063 m = 6.3 cm)
        self.marker_size = 0.063
        
        # Distancia para considerar que el robot llegó a la estación (en metros)
        self.station_threshold = 0.10
        
        # Diccionario de significados (ID -> Qué es)
        self.aruco_meanings = {
            0: "Robot",
            1: "Estacion 1",
            2: "Estacion 2"
        }

        # --- INICIALIZACIÓN DE VISIÓN ---
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Configuración del detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Definir los puntos 3D de las esquinas del marcador
        half_size = self.marker_size / 2.0
        self.obj_points = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        # --- CONFIGURACIÓN DE CÁMARA WEB USB ---
        # El índice 0 suele ser la cámara web principal. Si tienes otra cámara conectada (ej. laptop), prueba con 1.
        self.cap = cv2.VideoCapture(4)
        
        # Opcional: Intentar forzar resolución HD para mejor lectura de lejos
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara ROG EYE S. Verifica la conexión.")
            return

        # Crear un temporizador que simule el callback de la imagen (a ~30 fps)
        timer_period = 0.033  # Segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("Nodo de detección iniciado con ROG EYE S. Buscando Tags...")

    def timer_callback(self):
        ret, cv_image_raw = self.cap.read()
        if not ret:
            self.get_logger().warning("Fallo al capturar imagen de la cámara web.")
            return

        # Generar una matriz de cámara aproximada en el primer frame si no tenemos una
        if self.camera_matrix is None:
            h, w = cv_image_raw.shape[:2]
            focal_length = w  # Aproximación estándar
            self.camera_matrix = np.array([
                [focal_length, 0, w / 2],
                [0, focal_length, h / 2],
                [0, 0, 1]
            ], dtype=np.float32)
            # Asumimos distorsión cero para la webcam sin calibración previa
            self.dist_coeffs = np.zeros((4, 1))
            self.get_logger().info(f"Matriz aproximada creada para resolución {w}x{h}.")

        try:
            # Corregir distorsión (hará poco efecto hasta que calibres, pero mantiene la estructura)
            cv_image = cv2.undistort(cv_image_raw, self.camera_matrix, self.dist_coeffs)
            
            # Detectar tags
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
            
            # Diccionario para guardar datos de este frame
            current_poses = {}

            if ids is not None:
                # PRIMER PASO: Calcular la posición 3D
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

                # SEGUNDO PASO: Distancias relativas y lógica "REACHED"
                id_robot = 0  # Robot es el ID 0 según tu diccionario
                
                if id_robot in current_poses:
                    rx, ry, rz, _ = current_poses[id_robot]
                    
                    for station_id in [1, 2]: # Estaciones son ID 1 y 2
                        if station_id in current_poses:
                            sx, sy, sz, s_corners = current_poses[station_id]
                            
                            # Distancia en ejes
                            dx = sx - rx
                            dy = sy - ry
                            dz = sz - rz
                            
                            # Distancia euclidiana real (línea recta en el espacio 3D)
                            distancia_total = math.sqrt(dx**2 + dy**2 + dz**2)
                            
                            px_x = int(s_corners[0][0])
                            px_y = int(s_corners[0][1])
                            
                            # Mostrar información básica de distancia en ejes
                            cv2.putText(cv_image, f"dx: {dx:.2f} m", (px_x, px_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            cv2.putText(cv_image, f"dy: {dy:.2f} m", (px_x, px_y + 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                            # --- LÓGICA DE ALCANCE (REACHED) ---
                            if distancia_total < self.station_threshold:
                                # Fondo rojo
                                cv2.rectangle(cv_image, (px_x - 5, px_y - 65), (px_x + 115, px_y - 35), (0, 0, 255), -1)
                                # Texto blanco REACHED
                                cv2.putText(cv_image, "REACHED", (px_x, px_y - 45), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                            else:
                                # Si no ha llegado, mostrar distancia restante
                                cv2.putText(cv_image, f"Dist: {distancia_total:.2f}m", (px_x, px_y - 45), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Camara ROG EYE S", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error procesando la imagen: {e}")

    def destroy_node(self):
        # Asegurarse de liberar la cámara al cerrar
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
