import cv2
import numpy as np
import socket

# ==========================================
# 1. CONFIGURACIÓN
# ==========================================
IP_RASPBERRY = "192.168.137.240"  # <--- ¡TU IP AQUÍ!
PUERTO_UDP = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

marker_size = 0.063
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

cap = cv2.VideoCapture(4)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

half = marker_size / 2.0
obj_points = np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]], dtype=np.float32)

print(f"HUD de Visión Iniciado. Transmitiendo a {IP_RASPBERRY}:{PUERTO_UDP}")

try:
    while True:
        ret, frame = cap.read()
        if not ret: continue
        
        corners, ids, _ = detector.detectMarkers(frame)
        poses_3d = {}
        centros_2d = {}

        if ids is not None:
            h, w = frame.shape[:2]
            cam_mtx = np.array([[w,0,w/2],[0,w,h/2],[0,0,1]], dtype=np.float32)
            dist_eff = np.zeros((4,1))

            msg_parts = []
            
            for i in range(len(ids)):
                id_val = int(ids[i][0])
                
                # Calcular centro 2D (Para dibujar en pantalla)
                cx = int(np.mean(corners[i][0][:, 0]))
                cy = int(np.mean(corners[i][0][:, 1]))
                centros_2d[id_val] = (cx, cy)
                
                # Calcular posición 3D (Para mandar al robot)
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], cam_mtx, dist_eff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]
                    poses_3d[id_val] = (x, y, z)
                    msg_parts.append(f"{id_val}:{x:.4f},{y:.4f},{z:.4f}")
                    
                    # Dibujar Textos y Cuadros
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    cv2.putText(frame, f"ID {id_val} [X:{x:.2f} Y:{y:.2f}]", (cx - 40, cy - 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # --- DIBUJAR LA TRAYECTORIA (El Rectángulo) ---
            # Si ve los tags de las esquinas, dibuja líneas entre ellos
            rutas = [(1, 2), (2, 3), (3, 4), (4, 1)] # Puedes añadir más estaciones aquí
            for inicio, fin in rutas:
                if inicio in centros_2d and fin in centros_2d:
                    cv2.line(frame, centros_2d[inicio], centros_2d[fin], (0, 255, 0), 2)

            # Transmitir datos a la Raspberry
            if msg_parts:
                sock.sendto("|".join(msg_parts).encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))

        cv2.imshow("HUD Pickasso Cenital", frame)
        if cv2.waitKey(1) & 0xFF == 27: break # Presiona ESC para salir

finally:
    cap.release()
    cv2.destroyAllWindows()
    sock.close()
