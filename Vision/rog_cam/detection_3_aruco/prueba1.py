import cv2
import numpy as np
import socket
import math

# ==========================================
# 1. PANEL DE CONFIGURACIÓN
# ==========================================
IP_RASPBERRY = "192.168.137.240"  
PUERTO_UDP = 5005
PUERTO_CAMARA = 2  # Asegúrate de usar el número que sí te abre la ROG (0, 2 o 4)

# ==========================================
# 2. INICIALIZACIÓN ROBUSTA (A PRUEBA DE FALLOS)
# ==========================================
print(f"[*] Iniciando cámara en el puerto {PUERTO_CAMARA}...")
# Dejamos que Linux elija el mejor driver nativo automáticamente
cap = cv2.VideoCapture(PUERTO_CAMARA)

# Negociación amigable con la cámara (Sin forzar buffers)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print(f"[!] Error crítico: No se pudo acceder a la cámara en el puerto {PUERTO_CAMARA}.")
    exit()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
marker_size = 0.063
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
half = marker_size / 2.0
obj_points = np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]], dtype=np.float32)

# Memorias
memoria_3d = {}
memoria_2d = {}

print(f"[OK] HUD Estable Iniciado. Transmitiendo a {IP_RASPBERRY}")

# ==========================================
# 3. BUCLE PRINCIPAL (SECUENCIAL Y SEGURO)
# ==========================================
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[!] La cámara dejó de enviar video. Verifica el cable USB.")
            break
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        poses_actuales_3d = {}

        if ids is not None:
            h, w = frame.shape[:2]
            cam_mtx = np.array([[w,0,w/2],[0,w,h/2],[0,0,1]], dtype=np.float32)
            dist_eff = np.zeros((4,1))

            for i in range(len(ids)):
                id_val = int(ids[i][0])
                cx, cy = int(np.mean(corners[i][0][:, 0])), int(np.mean(corners[i][0][:, 1]))
                
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], cam_mtx, dist_eff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]
                    poses_actuales_3d[id_val] = (x, y, z)
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    
                    if id_val in [1, 3]:
                        memoria_3d[id_val] = (x, y, z)
                        memoria_2d[id_val] = (cx, cy)
                        cv2.putText(frame, f"Ancla {id_val}", (cx-20, cy-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    elif id_val == 0:
                        cv2.putText(frame, "PICKASSO", (cx-30, cy-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        # Generar Rectángulo y HUD Visual
        if 1 in memoria_3d and 3 in memoria_3d:
            x1, y1, z1 = memoria_3d[1]; x3, y3, z3 = memoria_3d[3]
            poses_actuales_3d[2] = (x3, y1, z1) 
            poses_actuales_3d[4] = (x1, y3, z1) 
            
            c1, c3 = memoria_2d[1], memoria_2d[3]
            c2, c4 = (c3[0], c1[1]), (c1[0], c3[1])
            
            for v_id, coords in zip([2, 4], [c2, c4]):
                cv2.circle(frame, coords, 8, (255, 0, 255), -1)
                cv2.putText(frame, f"Virtual {v_id}", (coords[0]-20, coords[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

            puntos_rect = np.array([c1, c2, c3, c4], np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [puntos_rect], isClosed=True, color=(0, 255, 0), thickness=2)

            centro_pista = (int((c1[0] + c3[0])/2), int((c1[1] + c3[1])/2))
            cv2.circle(frame, centro_pista, 5, (0, 255, 0), -1)
            
            if 0 in poses_actuales_3d:
                rx, ry, _ = poses_actuales_3d[0]
                cv2.putText(frame, f"ROBOT X: {rx:.2f}m | Y: {ry:.2f}m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        # Transmitir por red
        if 0 in poses_actuales_3d and 1 in memoria_3d and 3 in memoria_3d:
            if 1 not in poses_actuales_3d: poses_actuales_3d[1] = memoria_3d[1]
            if 3 not in poses_actuales_3d: poses_actuales_3d[3] = memoria_3d[3]
            msg_parts = [f"{id_val}:{x:.4f},{y:.4f},{z:.4f}" for id_val, (x, y, z) in poses_actuales_3d.items()]
            sock.sendto("|".join(msg_parts).encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))

        cv2.imshow("HUD Pickasso (Modo Estable)", frame)
        if cv2.waitKey(1) & 0xFF == 27: break 

finally:
    cap.release()
    cv2.destroyAllWindows()
    sock.close()
    print("[!] Sistema Apagado.")
