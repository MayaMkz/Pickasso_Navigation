import cv2
import numpy as np
import socket
import math
import threading

# ==========================================
# 1. PANEL DE CONFIGURACIÓN PRINCIPAL
# ==========================================
IP_RASPBERRY = "192.168.137.240"  
PUERTO_UDP = 5005
PUERTO_CAMARA = 2  # 0 suele ser la laptop. 2 o 4 suelen ser la ROG EYE S.

# ==========================================
# 2. MOTOR DE VISIÓN ANTI-LATENCIA
# ==========================================
class CamaraUltraRapida:
    def __init__(self, src):
        print(f"[*] Intentando forzar conexión con la cámara en el puerto {src}...")
        # cv2.CAP_V4L2 es el driver nativo de Linux para obligarlo a usar el USB externo
        self.stream = cv2.VideoCapture(src, cv2.CAP_V4L2)
        
        if not self.stream.isOpened():
            print(f"[ERROR] Linux bloqueó el puerto {src} o no hay cámara ahí.")
            print("=> SOLUCIÓN: Cambia la variable 'PUERTO_CAMARA' al principio del código por un 0, 2 o 4.")
            exit()
            
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1) # Cero lag
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
        print(f"[OK] ¡Cámara externa conectada con éxito!")

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()

# ==========================================
# 3. INICIALIZACIÓN DE VARIABLES
# ==========================================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
marker_size = 0.063
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
half = marker_size / 2.0
obj_points = np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]], dtype=np.float32)

# Arrancar hardware
camara = CamaraUltraRapida(PUERTO_CAMARA).start()

# ==========================================
# 4. BUCLE PRINCIPAL (HUD Y TELEMETRÍA)
# ==========================================
try:
    while True:
        frame = camara.read()
        if frame is None: continue
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        
        poses_3d = {}
        centros_2d = {}

        if ids is not None:
            h, w = frame.shape[:2]
            cam_mtx = np.array([[w,0,w/2],[0,w,h/2],[0,0,1]], dtype=np.float32)
            dist_eff = np.zeros((4,1))

            # 1. Procesar Tags Físicos
            for i in range(len(ids)):
                id_val = int(ids[i][0])
                cx, cy = int(np.mean(corners[i][0][:, 0])), int(np.mean(corners[i][0][:, 1]))
                
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], cam_mtx, dist_eff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    poses_3d[id_val] = (tvec[0][0], tvec[1][0], tvec[2][0])
                    centros_2d[id_val] = (cx, cy)
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    
                    # Etiquetas en pantalla
                    if id_val == 0:
                        cv2.putText(frame, "PICKASSO", (cx-30, cy-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                    else:
                        cv2.putText(frame, f"Esquina {id_val}", (cx-20, cy-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 2. Generar Rectángulo y Tags Virtuales
            if 1 in poses_3d and 3 in poses_3d:
                x1, y1, z1 = poses_3d[1]; x3, y3, z3 = poses_3d[3]
                
                # Esquinas Virtuales 3D
                poses_3d[2] = (x3, y1, z1) 
                poses_3d[4] = (x1, y3, z1) 
                
                # Esquinas Virtuales 2D (Para la pantalla)
                c1, c3 = centros_2d[1], centros_2d[3]
                c2, c4 = (c3[0], c1[1]), (c1[0], c3[1])
                centros_2d[2], centros_2d[4] = c2, c4
                
                for v_id in [2, 4]:
                    cv2.circle(frame, centros_2d[v_id], 8, (255, 0, 255), -1)
                    cv2.putText(frame, f"Virtual {v_id}", (centros_2d[v_id][0]-20, centros_2d[v_id][1]-15), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

                # Dibujar ruta
                puntos_rect = np.array([c1, c2, c3, c4], np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [puntos_rect], isClosed=True, color=(0, 255, 0), thickness=2)

                # Radar de distancia (Robot -> Centro)
                if 0 in centros_2d:
                    cx_rob, cy_rob = centros_2d[0]
                    centro_pista = (int((c1[0] + c3[0])/2), int((c1[1] + c3[1])/2))
                    cv2.line(frame, (cx_rob, cy_rob), centro_pista, (0, 165, 255), 1, cv2.LINE_AA)
                    cv2.putText(frame, "Centro", (centro_pista[0]-20, centro_pista[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)

            # 3. Transmitir Datos UDP
            msg_parts = [f"{id_val}:{x:.4f},{y:.4f},{z:.4f}" for id_val, (x, y, z) in poses_3d.items()]
            if msg_parts:
                sock.sendto("|".join(msg_parts).encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))

        cv2.imshow("HUD Central Pickasso", frame)
        if cv2.waitKey(1) & 0xFF == 27: break # ESC para salir

finally:
    camara.stop()
    cv2.destroyAllWindows()
    sock.close()
    print("[!] Transmisión Finalizada.")
