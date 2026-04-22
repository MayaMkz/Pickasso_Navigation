import cv2
import numpy as np
import socket
import math
import threading
import time

# ==========================================
# 1. PANEL DE CONFIGURACIÓN
# ==========================================
IP_RASPBERRY = "192.168.137.240"  
PUERTO_UDP = 5005
PUERTO_CAMARA = 2  # Cambia a 0, 2 o 4 según tu computadora

# ==========================================
# 2. MOTOR DE VISIÓN ANTI-CONGELAMIENTO
# ==========================================
class CamaraUltraRapida:
    def __init__(self, src):
        self.stream = cv2.VideoCapture(src, cv2.CAP_V4L2)
        if not self.stream.isOpened():
            print(f"[!] Falló el puerto {src}. Intentando el 0...")
            self.stream = cv2.VideoCapture(0) # Respaldo automático
            
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()
            # ESTE ES EL RESPIRO PARA QUE EL USB NO SE TRABE
            time.sleep(0.01) 

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()

# ==========================================
# 3. INICIALIZACIÓN
# ==========================================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
marker_size = 0.063
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
half = marker_size / 2.0
obj_points = np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]], dtype=np.float32)

camara = CamaraUltraRapida(PUERTO_CAMARA).start()

# --- LA MEMORIA DEL SISTEMA ---
# Aquí guardaremos la última posición conocida de los ArUcos físicos
memoria_3d = {}
memoria_2d = {}

print(f"[OK] HUD con Memoria Iniciado. Transmitiendo a {IP_RASPBERRY}")

# ==========================================
# 4. BUCLE PRINCIPAL
# ==========================================
try:
    while True:
        frame = camara.read()
        if frame is None: continue
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        
        # Estas variables son para enviar a la Raspberry
        poses_actuales_3d = {}

        if ids is not None:
            h, w = frame.shape[:2]
            cam_mtx = np.array([[w,0,w/2],[0,w,h/2],[0,0,1]], dtype=np.float32)
            dist_eff = np.zeros((4,1))

            # 1. Leer ArUcos en este milisegundo
            for i in range(len(ids)):
                id_val = int(ids[i][0])
                cx, cy = int(np.mean(corners[i][0][:, 0])), int(np.mean(corners[i][0][:, 1]))
                
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], cam_mtx, dist_eff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]
                    poses_actuales_3d[id_val] = (x, y, z)
                    
                    # Dibujar contorno de los reales
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    
                    # ¡ACTUALIZAR MEMORIA! (Solo de los fijos 1 y 3)
                    if id_val in [1, 3]:
                        memoria_3d[id_val] = (x, y, z)
                        memoria_2d[id_val] = (cx, cy)
                        cv2.putText(frame, f"Ancla {id_val}", (cx-20, cy-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    elif id_val == 0:
                        cv2.putText(frame, "PICKASSO", (cx-30, cy-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        # ==========================================
        # 2. GENERAR RECTÁNGULO DESDE LA MEMORIA
        # ==========================================
        # Ahora usamos la memoria, no importa si en este instante la cámara parpadeó
        if 1 in memoria_3d and 3 in memoria_3d:
            x1, y1, z1 = memoria_3d[1]; x3, y3, z3 = memoria_3d[3]
            
            # Crear Virtuales 3D para mandar a la Raspberry
            poses_actuales_3d[2] = (x3, y1, z1) 
            poses_actuales_3d[4] = (x1, y3, z1) 
            
            # Crear Virtuales 2D para Dibujar en Pantalla
            c1, c3 = memoria_2d[1], memoria_2d[3]
            c2, c4 = (c3[0], c1[1]), (c1[0], c3[1])
            
            # Dibujar los puntos virtuales
            for v_id, coords in zip([2, 4], [c2, c4]):
                cv2.circle(frame, coords, 8, (255, 0, 255), -1)
                cv2.putText(frame, f"Virtual {v_id}", (coords[0]-20, coords[1]-15), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

            # Dibujar el rectángulo (Ruta) uniendo la memoria
            puntos_rect = np.array([c1, c2, c3, c4], np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [puntos_rect], isClosed=True, color=(0, 255, 0), thickness=2)

            # Dibujar centro de la pista y distancia al robot
            centro_pista_x = int((c1[0] + c3[0])/2)
            centro_pista_y = int((c1[1] + c3[1])/2)
            cv2.circle(frame, (centro_pista_x, centro_pista_y), 5, (0, 255, 0), -1)
            
            if 0 in poses_actuales_3d:
                # Telemetría del robot
                rx, ry, _ = poses_actuales_3d[0]
                cx_rob = int(rx) # Aproximación visual
                # Nota: Para la línea naranja exacta, la calculamos si vemos el tag 0 este frame
                # Pero en la terminal mandaremos los datos crudos
                
                # Texto de HUD
                cv2.putText(frame, f"ROBOT X: {rx:.2f}m | Y: {ry:.2f}m", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        # ==========================================
        # 3. TRANSMITIR DATOS UDP (Robot + Físicos + Virtuales)
        # ==========================================
        # Solo mandamos datos si vimos al robot y la pista está armada
        if 0 in poses_actuales_3d and 1 in memoria_3d and 3 in memoria_3d:
            # Asegurarnos de que el Tag 1 y 3 se envíen desde la memoria si no se vieron en este frame exacto
            if 1 not in poses_actuales_3d: poses_actuales_3d[1] = memoria_3d[1]
            if 3 not in poses_actuales_3d: poses_actuales_3d[3] = memoria_3d[3]

            msg_parts = [f"{id_val}:{x:.4f},{y:.4f},{z:.4f}" for id_val, (x, y, z) in poses_actuales_3d.items()]
            sock.sendto("|".join(msg_parts).encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))

        cv2.imshow("HUD Central Pickasso (Memoria Activa)", frame)
        if cv2.waitKey(1) & 0xFF == 27: break # ESC para salir

finally:
    camara.stop()
    cv2.destroyAllWindows()
    sock.close()
    print("[!] Transmisión Finalizada.")
