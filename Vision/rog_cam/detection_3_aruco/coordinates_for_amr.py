import cv2
import numpy as np
import math
import socket
import time

# ==========================================
# 1. CONFIGURACIÓN DEL ENLACE UDP
# ==========================================
# ¡PON AQUÍ LA IP DE TU RASPBERRY PI!
IP_RASPBERRY = "192.168.0.100" 
PUERTO_UDP = 5005

print(f"Iniciando Transmisor de Visión hacia {IP_RASPBERRY}:{PUERTO_UDP}")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ==========================================
# 2. CONFIGURACIÓN DE VISIÓN
# ==========================================
marker_size = 0.063
station_threshold = 0.10
aruco_meanings = {0: "Robot", 1: "Estacion 1", 2: "Estacion 2"}

camera_matrix = None
dist_coeffs = None
        
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

half_size = marker_size / 2.0
obj_points = np.array([
    [-half_size,  half_size, 0], [ half_size,  half_size, 0],
    [ half_size, -half_size, 0], [-half_size, -half_size, 0]
], dtype=np.float32)

cap = cv2.VideoCapture(4)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

if not cap.isOpened():
    print("Error: No se pudo abrir la cámara.")
    exit()

# ==========================================
# 3. BUCLE PRINCIPAL
# ==========================================
try:
    while True:
        ret, cv_image_raw = cap.read()
        if not ret: continue

        if camera_matrix is None:
            h, w = cv_image_raw.shape[:2]
            focal_length = w  
            camera_matrix = np.array([[focal_length, 0, w / 2], [0, focal_length, h / 2], [0, 0, 1]], dtype=np.float32)
            dist_coeffs = np.zeros((4, 1))

        cv_image = cv2.undistort(cv_image_raw, camera_matrix, dist_coeffs)
        corners, ids, rejected = detector.detectMarkers(cv_image)
        current_poses = {}

        if ids is not None:
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                marker_corners = corners[i][0]
                success, rvec, tvec = cv2.solvePnP(obj_points, marker_corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                if success:
                    x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]
                    current_poses[marker_id] = (x, y, z, marker_corners)

                    cv2.aruco.drawDetectedMarkers(cv_image, corners)
                    cv2.drawFrameAxes(cv_image, camera_matrix, dist_coeffs, rvec, tvec, marker_size)
                    
                    px_x, px_y = int(marker_corners[0][0]), int(marker_corners[0][1])
                    cv2.putText(cv_image, aruco_meanings.get(marker_id, f"Tag {marker_id}"), (px_x, px_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # --- LÓGICA DE ALCANCE Y TRANSMISIÓN UDP ---
            id_robot = 0  
            estacion_objetivo = 1 
            
            if id_robot in current_poses and estacion_objetivo in current_poses:
                rx, ry, rz, _ = current_poses[id_robot]
                sx, sy, sz, s_corners = current_poses[estacion_objetivo]
                
                dx = sx - rx
                dy = sy - ry
                distancia_total = math.sqrt(dx**2 + dy**2)
                
                # ¡MAGIA AQUÍ! Enviamos los datos crudos por WiFi a la Raspberry
                mensaje = f"{dx:.4f},{dy:.4f},{distancia_total:.4f}"
                sock.sendto(mensaje.encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))
                
                px_x, px_y = int(s_corners[0][0]), int(s_corners[0][1])
                if distancia_total < station_threshold:
                    cv2.rectangle(cv_image, (px_x - 5, px_y - 65), (px_x + 115, px_y - 35), (0, 0, 255), -1)
                    cv2.putText(cv_image, "REACHED", (px_x, px_y - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                else:
                    cv2.putText(cv_image, f"Dist: {distancia_total:.2f}m", (px_x, px_y - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Camara Cenital (Transmisor UDP)", cv_image)
        if cv2.waitKey(30) & 0xFF == 27: # Presiona ESC para salir
            break

except KeyboardInterrupt:
    pass
finally:
    cap.release()
    cv2.destroyAllWindows()
    sock.close()
