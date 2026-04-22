import cv2
import numpy as np
import socket
import sys
import time

# ==========================================
# 1. CONFIGURACIÓN
# ==========================================
IP_RASPBERRY = "192.168.137.240"
PUERTO_UDP = 5005
PUERTO_CAMARA = 2  # Cambia a 0 o 4 si no agarra tu ROG EYE S

print("\n" + "="*50)
print("[*] INICIANDO SERVIDOR DE VISIÓN (MODO HEADLESS)")
print(f"[*] Destino UDP: {IP_RASPBERRY}:{PUERTO_UDP}")
print("="*50 + "\n")

cap = cv2.VideoCapture(PUERTO_CAMARA)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

if not cap.isOpened():
    print(f"[!] Error: No se pudo abrir la cámara en el puerto {PUERTO_CAMARA}.")
    sys.exit()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Tamaño de 6.3 cm
half = 0.063 / 2.0
obj_points = np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]], dtype=np.float32)

print("[OK] Transmisión en vivo. Presiona Ctrl+C para detener.\n")

try:
    while True:
        ret, frame = cap.read()
        if not ret: 
            time.sleep(0.01)
            continue

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

            # ==========================================
            # LÓGICA DE ESQUINAS VIRTUALES
            # Asumimos que Tag 1 y Tag 2 son las esquinas físicas opuestas
            # ==========================================
            if 1 in poses_3d and 2 in poses_3d:
                x1, y1, z1 = poses_3d[1]
                x2, y2, z2 = poses_3d[2]
                
                # Generamos las esquinas 3 y 4 matemáticamente
                poses_3d[3] = (x2, y1, z1) 
                poses_3d[4] = (x1, y2, z1) 

            # ==========================================
            # TRANSMISIÓN UDP Y MONITOR EN TERMINAL
            # ==========================================
            msg_parts = [f"{id_val}:{x:.4f},{y:.4f},{z:.4f}" for id_val, (x, y, z) in poses_3d.items()]
            
            if msg_parts:
                mensaje_final = "|".join(msg_parts)
                # Enviar a la Raspberry
                sock.sendto(mensaje_final.encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))
                
                # Imprimir en la computadora para que veas qué se está mandando
                sys.stdout.write(f"\r[TX] Enviando: {mensaje_final}                                      ")
                sys.stdout.flush()

        else:
            # Si no ve ningún ArUco, limpia la línea de la terminal
            sys.stdout.write("\r[--] Buscando ArUcos...                                               ")
            sys.stdout.flush()

except KeyboardInterrupt:
    print("\n\n[!] Servidor detenido por el usuario.")
finally:
    cap.release()
    sock.close()
    print("[OK] Puertos liberados correctamente.")
