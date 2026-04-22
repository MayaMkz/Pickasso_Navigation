import cv2
import numpy as np
import socket
import math

# ==========================================
# 1. CONFIGURACIÓN
# ==========================================
IP_RASPBERRY = "192.168.137.240"
PUERTO_UDP = 5005
PUERTO_CAMARA = 2  # Intenta 0, 2 o 4

print(f"[*] Encendiendo cámara en modo SEGURO (Sin forzar hardware)...")
cap = cv2.VideoCapture(PUERTO_CAMARA)

if not cap.isOpened():
    print(f"[!] Error: No se pudo abrir el puerto {PUERTO_CAMARA}. Intenta con el 0.")
    exit()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

obj_points = np.array([[-0.0315, 0.0315, 0], [0.0315, 0.0315, 0], 
                       [0.0315, -0.0315, 0], [-0.0315, -0.0315, 0]], dtype=np.float32)

print("[OK] Sistema en línea. Transmitiendo...")

try:
    while True:
        ret, frame_raw = cap.read()
        if not ret: continue

        # [!] EL SECRETO ANTI-TRABAS [!]
        # Reducimos la imagen por software, el USB ni se entera.
        frame = cv2.resize(frame_raw, (800, 600))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, _ = detector.detectMarkers(gray)
        poses_3d = {}
        centros_2d = {}

        if ids is not None:
            h, w = frame.shape[:2]
            cam_mtx = np.array([[w,0,w/2],[0,w,h/2],[0,0,1]], dtype=np.float32)
            dist_eff = np.zeros((4,1))

            # Leer ArUcos detectados
            for i in range(len(ids)):
                id_val = int(ids[i][0])
                cx, cy = int(np.mean(corners[i][0][:, 0])), int(np.mean(corners[i][0][:, 1]))
                
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], cam_mtx, dist_eff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    poses_3d[id_val] = (tvec[0][0], tvec[1][0], tvec[2][0])
                    centros_2d[id_val] = (cx, cy)
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    cv2.putText(frame, f"ID {id_val}", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # LÓGICA DE DIBUJO Y ESQUINAS VIRTUALES
            # Asumimos Tag 1 y Tag 2 como las esquinas físicas opuestas de tu pista
            if 1 in poses_3d and 2 in poses_3d:
                # Matemáticas para las esquinas invisibles (3 y 4)
                x1, y1, z1 = poses_3d[1]
                x2, y2, z2 = poses_3d[2]
                poses_3d[3] = (x2, y1, z1) # Esquina Virtual A
                poses_3d[4] = (x1, y2, z1) # Esquina Virtual B
                
                c1, c2 = centros_2d[1], centros_2d[2]
                c3, c4 = (c2[0], c1[1]), (c1[0], c2[1])
                
                # Dibujar ruta del rectángulo
                puntos = np.array([c1, c3, c2, c4], np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [puntos], isClosed=True, color=(0, 255, 255), thickness=2)

                # Si vemos al robot, dibujamos la línea de trayectoria hacia el Tag 1
                if 0 in poses_3d:
                    rx, ry, _ = poses_3d[0]
                    cv2.line(frame, centros_2d[0], c1, (0, 0, 255), 2) # Línea Roja de trayectoria
                    
                    # Mostrar error X e Y en la pantalla
                    err_x = x1 - rx
                    err_y = y1 - ry
                    cv2.putText(frame, f"Robot->Tag 1 | Error X: {err_x:.2f}m | Error Y: {err_y:.2f}m", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Enviar datos a Raspberry
            msg_parts = [f"{id_val}:{x:.4f},{y:.4f},{z:.4f}" for id_val, (x, y, z) in poses_3d.items()]
            if msg_parts:
                sock.sendto("|".join(msg_parts).encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))

        cv2.imshow("HUD Pickasso (Estable)", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

finally:
    cap.release()
    cv2.destroyAllWindows()
    sock.close()
