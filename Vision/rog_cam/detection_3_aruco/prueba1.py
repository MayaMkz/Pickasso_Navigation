import cv2
import numpy as np
import math
import socket

# Configuración de Red
IP_RASPBERRY = "TU_IP_AQUI" 
PUERTO_UDP = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Configuración ArUco
marker_size = 0.063
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

cap = cv2.VideoCapture(4) # Cámara ROG EYE S
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Puntos 3D para solvePnP
half = marker_size / 2.0
obj_points = np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]], dtype=np.float32)

try:
    while True:
        ret, frame = cap.read()
        if not ret: continue
        
        corners, ids, _ = detector.detectMarkers(frame)
        poses = {}

        if ids is not None:
            # Matriz de cámara aproximada (mejorar con calibración real después)
            h, w = frame.shape[:2]
            f = w
            cam_mtx = np.array([[f,0,w/2],[0,f,h/2],[0,0,1]], dtype=np.float32)
            dist_eff = np.zeros((4,1))

            for i in range(len(ids)):
                id_val = int(ids[i][0])
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], cam_mtx, dist_eff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    poses[id_val] = (tvec[0][0], tvec[1][0], tvec[2][0])
                    cv2.aruco.drawDetectedMarkers(frame, corners)

            # Lógica de envío: Mandamos los datos de TODOS los tags detectados
            # Formato: "ID:x,y,z|ID:x,y,z"
            msg_parts = []
            for id_found, (x, y, z) in poses.items():
                msg_parts.append(f"{id_found}:{x:.4f},{y:.4f},{z:.4f}")
            
            if msg_parts:
                mensaje = "|".join(msg_parts)
                sock.sendto(mensaje.encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))

        cv2.imshow("Vision Cenital - Pickasso", frame)
        if cv2.waitKey(1) & 0xFF == 27: break
finally:
    cap.release()
    cv2.destroyAllWindows()
