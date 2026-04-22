import cv2
import numpy as np
import socket
import math

# ==========================================
# 1. CONFIGURACIÓN Y RED
# ==========================================
IP_RASPBERRY = "192.168.137.240"  # Confirma que siga siendo esta
PUERTO_UDP = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

marker_size = 0.063
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# --- TRUCO ANTI-LATENCIA ---
# Si estás en Linux/Ubuntu, agregar cv2.CAP_V4L2 ayuda muchísimo
cap = cv2.VideoCapture(4, cv2.CAP_V4L2) 
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # ¡Cero fila de espera!
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # Menos pixeles = Menos lag
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

half = marker_size / 2.0
obj_points = np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]], dtype=np.float32)

print(f"HUD Anti-Latencia Iniciado. Transmitiendo a {IP_RASPBERRY}:{PUERTO_UDP}")

try:
    while True:
        # Purgar el buffer manualmente para asegurar la imagen más reciente
        cap.grab()
        ret, frame = cap.retrieve()
        if not ret: continue
        
        corners, ids, _ = detector.detectMarkers(frame)
        poses_3d = {}
        centros_2d = {}

        if ids is not None:
            h, w = frame.shape[:2]
            cam_mtx = np.array([[w,0,w/2],[0,w,h/2],[0,0,1]], dtype=np.float32)
            dist_eff = np.zeros((4,1))

            for i in range(len(ids)):
                id_val = int(ids[i][0])
                cx, cy = int(np.mean(corners[i][0][:, 0])), int(np.mean(corners[i][0][:, 1]))
                
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], cam_mtx, dist_eff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    poses_3d[id_val] = (tvec[0][0], tvec[1][0], tvec[2][0])
                    centros_2d[id_val] = (cx, cy)
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    
                    etiqueta = "Robot" if id_val == 0 else f"Esq. FIsica {id_val}"
                    cv2.putText(frame, etiqueta, (cx-20, cy-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # --- GENERAR ESQUINAS VIRTUALES (2 y 4) ---
            if 1 in poses_3d and 3 in poses_3d:
                x1, y1, z1 = poses_3d[1]; x3, y3, z3 = poses_3d[3]
                poses_3d[2] = (x3, y1, z1); centros_2d[2] = (centros_2d[3][0], centros_2d[1][1])
                poses_3d[4] = (x1, y3, z1); centros_2d[4] = (centros_2d[1][0], centros_2d[3][1])
                
                for v_id in [2, 4]:
                    cv2.circle(frame, centros_2d[v_id], 6, (255, 0, 255), -1)
                    cv2.putText(frame, f"Virtual {v_id}", (centros_2d[v_id][0]-30, centros_2d[v_id][1]-15), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

            # --- DIBUJAR RECTÁNGULO DE RUTA ---
            rutas = [(1, 2), (2, 3), (3, 4), (4, 1)] 
            for inicio, fin in rutas:
                if inicio in centros_2d and fin in centros_2d:
                    cv2.line(frame, centros_2d[inicio], centros_2d[fin], (0, 255, 0), 2)

            # --- DIBUJAR ERROR LATERAL (Cross-Track) Y MANDAR DATOS ---
            if 0 in poses_3d:
                rx, ry, rz = poses_3d[0]
                
                # Elegimos dibujar la línea hacia el Tag 1 por defecto en pantalla
                if 1 in poses_3d:
                    tx, ty, tz = poses_3d[1]
                    err_x = tx - rx
                    err_y = ty - ry
                    dist = math.hypot(err_x, err_y)
                    
                    cv2.line(frame, centros_2d[0], centros_2d[1], (0, 165, 255), 2)
                    cv2.putText(frame, f"Dist: {dist:.2f}m | dX: {err_x:.2f} | dY: {err_y:.2f}", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

            # Enviar todo por WiFi
            msg_parts = [f"{id_val}:{x:.4f},{y:.4f},{z:.4f}" for id_val, (x, y, z) in poses_3d.items()]
            if msg_parts:
                sock.sendto("|".join(msg_parts).encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))

        cv2.imshow("HUD Pickasso (Alta Velocidad)", frame)
        if cv2.waitKey(1) & 0xFF == 27: break 

finally:
    cap.release()
    cv2.destroyAllWindows()
    sock.close()
