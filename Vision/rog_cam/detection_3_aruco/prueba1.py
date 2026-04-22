import cv2
import numpy as np
import socket
import math
import threading

# ==========================================
# 0. CLASE ANTI-LATENCIA (EL SECRETO INDUSTRIAL)
# ==========================================
class CamaraUltraRapida:
    def __init__(self, src=0):
        # Inicia la cámara y baja la resolución para vuelo a alta velocidad
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        # Arranca un hilo en el fondo solo para leer el USB sin parar
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
# 1. CONFIGURACIÓN Y RED
# ==========================================
IP_RASPBERRY = "192.168.137.240"  # ¡Verifica tu IP!
PUERTO_UDP = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

marker_size = 0.063
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

half = marker_size / 2.0
obj_points = np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]], dtype=np.float32)

print(f"[OK] Arrancando Motor de Visión Multihilo hacia {IP_RASPBERRY}:{PUERTO_UDP}...")

# Arrancamos la cámara en esteroides (cambia el 0 por tu puerto correcto si es necesario)
camara = CamaraUltraRapida(0).start()

try:
    while True:
        # Tomamos el frame más fresco al instante
        frame = camara.read()
        if frame is None: continue
        
        # Truco 2 para velocidad: Convertir a blanco y negro para detectar ArUcos más rápido
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        
        poses_3d = {}
        centros_2d = {}

        if ids is not None:
            h, w = frame.shape[:2]
            cam_mtx = np.array([[w,0,w/2],[0,w,h/2],[0,0,1]], dtype=np.float32)
            dist_eff = np.zeros((4,1))

            # 1. Encontrar los ArUcos físicos
            for i in range(len(ids)):
                id_val = int(ids[i][0])
                cx, cy = int(np.mean(corners[i][0][:, 0])), int(np.mean(corners[i][0][:, 1]))
                
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], cam_mtx, dist_eff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    poses_3d[id_val] = (tvec[0][0], tvec[1][0], tvec[2][0])
                    centros_2d[id_val] = (cx, cy)
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    
                    if id_val != 0:
                        cv2.putText(frame, f"Esquina {id_val}", (cx-20, cy-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # ==========================================
            # 2. GENERAR EL RECTÁNGULO PERFECTO
            # ==========================================
            # Asumimos que pusiste el Tag 1 y el Tag 3 en diagonal en el piso
            if 1 in poses_3d and 3 in poses_3d:
                # Matemáticas 3D
                x1, y1, z1 = poses_3d[1]; x3, y3, z3 = poses_3d[3]
                poses_3d[2] = (x3, y1, z1) # Generar Virtual 2
                poses_3d[4] = (x1, y3, z1) # Generar Virtual 4
                
                # Matemáticas 2D (Para dibujar)
                c1 = centros_2d[1]; c3 = centros_2d[3]
                c2 = (c3[0], c1[1]) # Generar pixel de Esquina 2
                c4 = (c1[0], c3[1]) # Generar pixel de Esquina 4
                
                centros_2d[2] = c2; centros_2d[4] = c4
                
                # Dibujar las esquinas virtuales
                for v_id in [2, 4]:
                    cv2.circle(frame, centros_2d[v_id], 8, (255, 0, 255), -1)
                    cv2.putText(frame, f"Virtual {v_id}", (centros_2d[v_id][0]-20, centros_2d[v_id][1]-15), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

                # DIBUJAR EL PERÍMETRO VERDE (1->2->3->4->1)
                puntos_rectangulo = np.array([c1, c2, c3, c4], np.int32)
                puntos_rectangulo = puntos_rectangulo.reshape((-1, 1, 2))
                cv2.polylines(frame, [puntos_rectangulo], isClosed=True, color=(0, 255, 0), thickness=2)

            # ==========================================
            # 3. TELEMETRÍA DEL ROBOT (TAG 0)
            # ==========================================
            if 0 in poses_3d:
                rx, ry, rz = poses_3d[0]
                cx, cy = centros_2d[0]
                
                # Resaltar al robot
                cv2.circle(frame, (cx, cy), 15, (0, 165, 255), 2)
                cv2.putText(frame, "PICKASSO", (cx-30, cy-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                
                # Mostrar sus coordenadas absolutas en la pantalla
                cv2.putText(frame, f"Robot Pos: X:{rx:.2f}m | Y:{ry:.2f}m", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

                # Si el rectángulo existe, dibujamos una línea al centro de la pista
                if 1 in poses_3d and 3 in poses_3d:
                    centro_pista_x = int((c1[0] + c3[0]) / 2)
                    centro_pista_y = int((c1[1] + c3[1]) / 2)
                    cv2.line(frame, (cx, cy), (centro_pista_x, centro_pista_y), (0, 165, 255), 1, cv2.LINE_AA)

            # ==========================================
            # 4. TRANSMITIR A LA RASPBERRY
            # ==========================================
            msg_parts = [f"{id_val}:{x:.4f},{y:.4f},{z:.4f}" for id_val, (x, y, z) in poses_3d.items()]
            if msg_parts:
                sock.sendto("|".join(msg_parts).encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))

        cv2.imshow("HUD Pickasso (Cero Lag)", frame)
        
        # Presiona ESC para salir
        if cv2.waitKey(1) & 0xFF == 27: 
            break 

finally:
    camara.stop()
    cv2.destroyAllWindows()
    sock.close()
    print("[!] Sistema Apagado.")
