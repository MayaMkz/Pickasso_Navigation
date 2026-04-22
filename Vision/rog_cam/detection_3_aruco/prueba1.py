import cv2
import numpy as np
import socket
import sys

# ==========================================
# 1. CONFIGURACIÓN
# ==========================================
IP_RASPBERRY = "192.168.137.240"
PUERTO_UDP = 5005
PUERTO_CAMARA = 2  # Asegúrate de usar el que te funciona

print(f"[*] Encendiendo motor de visión en modo Headless (Sin video)...")
cap = cv2.VideoCapture(PUERTO_CAMARA)

if not cap.isOpened():
    print(f"[!] Error: No se pudo abrir el puerto {PUERTO_CAMARA}.")
    exit()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

obj_points = np.array([[-0.0315, 0.0315, 0], [0.0315, 0.0315, 0], 
                       [0.0315, -0.0315, 0], [-0.0315, -0.0315, 0]], dtype=np.float32)

print(f"[OK] Transmitiendo telemetría a {IP_RASPBERRY}:{PUERTO_UDP}")
print("Presiona Ctrl+C en la terminal para detener el programa.\n")

try:
    while True:
        ret, frame_raw = cap.read()
        if not ret: continue

        # Reducimos tamaño solo para aligerar la detección (sin mostrarlo)
        frame = cv2.resize(frame_raw, (800, 600))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, _ = detector.detectMarkers(gray)
        poses_3d = {}

        if ids is not None:
            h, w = frame.shape[:2]
            cam_mtx = np.array([[w,0,w/2],[0,w,h/2],[0,0,1]], dtype=np.float32)
            dist_eff = np.zeros((4,1))

            for i in range(len(ids)):
                id_val = int(ids[i][0])
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], cam_mtx, dist_eff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    poses_3d[id_val] = (tvec[0][0], tvec[1][0], tvec[2][0])

            # Lógica de esquinas virtuales
            if 1 in poses_3d and 2 in poses_3d:
                x1, y1, z1 = poses_3d[1]
                x2, y2, z2 = poses_3d[2]
                poses_3d[3] = (x2, y1, z1) 
                poses_3d[4] = (x1, y2, z1) 

            # Empaquetar datos para UDP
            msg_parts = [f"{id_val}:{x:.4f},{y:.4f},{z:.4f}" for id_val, (x, y, z) in poses_3d.items()]
            
            if msg_parts:
                sock.sendto("|".join(msg_parts).encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))
                
                # --- IMPRESIÓN EN TERMINAL A ALTA VELOCIDAD ---
                texto_terminal = " | ".join([f"ID {id_val} (X:{x:.2f} Y:{y:.2f})" for id_val, (x, y, z) in poses_3d.items()])
                sys.stdout.write(f"\r[Transmisión en Vivo] {texto_terminal}                                ")
                sys.stdout.flush()

except KeyboardInterrupt:
    print("\n\n[!] Transmisión detenida por el usuario.")
finally:
    cap.release()
    sock.close()
    print("Puerto de cámara liberado.")
