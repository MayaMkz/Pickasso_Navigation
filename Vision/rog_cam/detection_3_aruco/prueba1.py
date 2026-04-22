import cv2
import numpy as np
import socket
import sys
import time

# ==========================================
# CONFIGURACIÓN
# ==========================================
IP_RASPBERRY = "192.168.137.240"
PUERTO_UDP = 5005
PUERTO_CAMARA = 2 # Cambia a 0 o 4 si es necesario

print("[*] Iniciando Visión Minimalista (Transmisor de Coordenadas)...")
cap = cv2.VideoCapture(PUERTO_CAMARA)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

if not cap.isOpened(): sys.exit("[!] Error abriendo cámara.")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Tamaño del ArUco (6.3 cm)
half = 0.063 / 2.0
obj_points = np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]], dtype=np.float32)

print("[OK] Sistema en línea. Presiona Ctrl+C para detener.\n")

try:
    while True:
        ret, frame = cap.read()
        if not ret: time.sleep(0.01); continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        poses = {}

        if ids is not None:
            h, w = frame.shape[:2]
            cam_mtx = np.array([[w,0,w/2],[0,w,h/2],[0,0,1]], dtype=np.float32)
            dist_eff = np.zeros((4,1))

            for i in range(len(ids)):
                id_val = int(ids[i][0])
                success, _, tvec = cv2.solvePnP(obj_points, corners[i][0], cam_mtx, dist_eff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    # Guardamos solo X y Y (Z no nos importa para navegar en el piso)
                    poses[id_val] = (tvec[0][0], tvec[1][0])

            # Transmitir por UDP
            msg_parts = [f"{id_val}:{x:.4f},{y:.4f}" for id_val, (x, y) in poses.items()]
            if msg_parts:
                mensaje = "|".join(msg_parts)
                sock.sendto(mensaje.encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))
                
                # Imprimir en terminal para verificar que no hay latencia
                sys.stdout.write(f"\r[TX] {mensaje}                                      ")
                sys.stdout.flush()

except KeyboardInterrupt: print("\n[!] Detenido.")
finally: cap.release(); sock.close()
