import cv2
import numpy as np
import math
import threading
import socket
from collections import deque

# ==========================================
# CÁMARA TURBO (CERO LATENCIA)
# ==========================================
class CamaraIP_UltraRapida:
    def __init__(self, url):
        self.stream = cv2.VideoCapture(url)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.stream.isOpened(): raise Exception("Error DroidCam.")
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            if not self.grabbed: self.stop()
            else: (self.grabbed, self.frame) = self.stream.read()

    def read(self): return self.frame
    def stop(self): self.stopped = True; self.stream.release()

# ==========================================
# PROGRAMA PRINCIPAL
# ==========================================
def main():
    DROIDCAM_URL = "http://192.168.137.24:4747/video"
    IP_RASPBERRY = "192.168.137.240"                 
    PUERTO_UDP = 5005

    TOLERANCIA_LLEGADA_M = 0.15 # Ampliado para evitar overshoot
    OFFSET_MESA_M = 0.45 

    print(f"[*] Conectando a Cámara en {DROIDCAM_URL}...")
    try: cap = CamaraIP_UltraRapida(DROIDCAM_URL).start()
    except Exception as e: print(f"[!] {e}"); return

    print(f"[*] Abriendo canal UDP hacia {IP_RASPBERRY}:{PUERTO_UDP}...")
    sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    marker_size = 0.063  
    ESCALA_RADAR = 150  
    CENTRO_RADAR = (200, 200)  
    estela_robot = deque(maxlen=60)  
    memoria_tags = {} 
    
    mision_activa = False
    estado_mision = 0  
    objetivo_actual = None
    nombre_objetivo = ""
    sentido_giro_global = 1 

    print("[*] Buscando archivo de calibración...")
    fs = cv2.FileStorage("parametros_droidcam.yaml", cv2.FILE_STORAGE_READ)
    if fs.isOpened():
        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("dist_coeffs").mat()
        fs.release()
        w, h = 1280, 720 
        map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, camera_matrix, (w, h), cv2.CV_32FC1)
    else:
        w, h = 1280, 720
        focal_length = w * 0.9 
        camera_matrix = np.array([[focal_length, 0, w / 2], [0, focal_length, h / 2], [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((4, 1))
        map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, camera_matrix, (w,h), cv2.CV_32FC1)
    
    zero_dist = np.zeros((4, 1))
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    half_size = marker_size / 2.0
    obj_points = np.array([[-half_size, half_size, 0], [half_size, half_size, 0], 
                           [half_size, -half_size, 0], [-half_size, -half_size, 0]], dtype=np.float32)

    def metros_a_pixeles_radar(x, y):
        return (int(CENTRO_RADAR[0] + (x * ESCALA_RADAR)), int(CENTRO_RADAR[1] - (y * ESCALA_RADAR)))

    ruta_pts_global = []

    while True:
        cv_image_raw = cap.read()
        if cv_image_raw is None: continue

        cam_view = cv2.remap(cv_image_raw, map1, map2, interpolation=cv2.INTER_LINEAR)
        minimapa = np.zeros((400, 400, 3), dtype=np.uint8)
        
        for i in range(0, 400, 50):
            cv2.line(minimapa, (i, 0), (i, 400), (30, 30, 30), 1)
            cv2.line(minimapa, (0, i), (400, i), (30, 30, 30), 1)

        gray = cv2.cvtColor(cam_view, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            for i in range(len(ids)):
                m_id = int(ids[i][0])
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], camera_matrix, zero_dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    cx, cy = int(np.mean(corners[i][0][:, 0])), int(np.mean(corners[i][0][:, 1]))
                    # [NUEVO] Guardamos rvec y tvec completos para calcular el frente del robot
                    memoria_tags[m_id] = {'m_x': tvec[0][0], 'm_y': tvec[1][0], 'm_z': tvec[2][0], 'c_x': cx, 'c_y': cy, 'rvec': rvec, 'tvec': tvec}
                    cv2.aruco.drawDetectedMarkers(cam_view, corners)
                    color = (0, 165, 255) if m_id == 0 else (0, 255, 0)
                    cv2.putText(cam_view, f"Tag {m_id}" if m_id!=0 else "Robot", (cx-40, cy-40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # =========================================================
        # GEOMETRÍA INDEPENDIENTE Y OFFSET DE MESA
        # =========================================================
        if 1 in memoria_tags and 2 in memoria_tags:
            t1x, t1y, t1z = memoria_tags[1]['m_x'], memoria_tags[1]['m_y'], memoria_tags[1]['m_z']
            t2x, t2y, t2z = memoria_tags[2]['m_x'], memoria_tags[2]['m_y'], memoria_tags[2]['m_z']
            
            cz = (t1z + t2z) / 2.0
            
            mesa_pts = [
                (t1x, t1y, t1z),   
                (t2x, t1y, cz),    
                (t2x, t2y, t2z),   
                (t1x, t2y, cz)     
            ]

            centro_mesa_x = (t1x + t2x) / 2.0
            centro_mesa_y = (t1y + t2y) / 2.0

            def expandir_punto(px, py, pz):
                nx = px + OFFSET_MESA_M if px > centro_mesa_x else px - OFFSET_MESA_M
                ny = py + OFFSET_MESA_M if py > centro_mesa_y else py - OFFSET_MESA_M
                return (nx, ny, pz)

            ruta_pts_global = [expandir_punto(*p) for p in mesa_pts]

            pts_radar_mesa = np.array([metros_a_pixeles_radar(p[0], p[1]) for p in mesa_pts], np.int32).reshape((-1, 1, 2))
            cv2.polylines(minimapa, [pts_radar_mesa], isClosed=True, color=(255, 150, 50), thickness=2)
            
            pts_radar_ruta = np.array([metros_a_pixeles_radar(p[0], p[1]) for p in ruta_pts_global], np.int32).reshape((-1, 1, 2))
            cv2.polylines(minimapa, [pts_radar_ruta], isClosed=True, color=(0, 255, 0), thickness=2)

            if camera_matrix is not None:
                pts_3d_mesa = np.array(mesa_pts, dtype=np.float32)
                pts_2d_mesa, _ = cv2.projectPoints(pts_3d_mesa, np.zeros((3,1)), np.zeros((3,1)), camera_matrix, zero_dist)
                cv2.polylines(cam_view, [np.int32(pts_2d_mesa).reshape((-1,1,2))], isClosed=True, color=(255, 150, 50), thickness=2)
                
                pts_3d_ruta = np.array(ruta_pts_global, dtype=np.float32)
                pts_2d_ruta, _ = cv2.projectPoints(pts_3d_ruta, np.zeros((3,1)), np.zeros((3,1)), camera_matrix, zero_dist)
                cv2.polylines(cam_view, [np.int32(pts_2d_ruta).reshape((-1,1,2))], isClosed=True, color=(0, 255, 0), thickness=3)

            ancho_mesa = abs(t1x - t2x)
            largo_mesa = abs(t1y - t2y)
            txt_mesa = f"Mesa: {ancho_mesa:.2f}m x {largo_mesa:.2f}m"
            cv2.putText(cam_view, txt_mesa, (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 150, 50), 2)

        # =========================================================
        # [NUEVO] POSE 6D: FLECHA Y ORIENTACIÓN DEL ROBOT
        # =========================================================
        if 0 in memoria_tags:
            rx, ry = memoria_tags[0]['m_x'], memoria_tags[0]['m_y']
            rvec, tvec = memoria_tags[0]['rvec'], memoria_tags[0]['tvec']
            
            # 1. Convertimos el vector de rotación en una Matriz de Rotación 3x3
            R, _ = cv2.Rodrigues(rvec)
            
            # 2. Vector apuntando hacia el "Frente" (La parte de arriba del tag es -Y)
            vector_frente_local = np.array([[0.0], [-0.15], [0.0]]) # 15 cm al frente
            vector_frente_cam = R @ vector_frente_local
            
            fx = rx + vector_frente_cam[0][0]
            fy = ry + vector_frente_cam[1][0]
            
            # 3. Calculamos el ángulo visual del robot (Yaw Absoluto)
            robot_yaw_visual = math.atan2(vector_frente_cam[1][0], vector_frente_cam[0][0])
            
            # 4. Dibujar Flecha en el Minimapa
            p_robot_radar = metros_a_pixeles_radar(rx, ry)
            p_frente_radar = metros_a_pixeles_radar(fx, fy)
            cv2.arrowedLine(minimapa, p_robot_radar, p_frente_radar, (0, 0, 255), 2, tipLength=0.3)
            
            # 5. Dibujar Flecha en la Cámara 3D
            pt3d = np.array([[0.0, -0.15, 0.0]], dtype=np.float32)
            pt2d, _ = cv2.projectPoints(pt3d, rvec, tvec, camera_matrix, zero_dist)
            px, py = int(pt2d[0][0][0]), int(pt2d[0][0][1])
            cx, cy = memoria_tags[0]['c_x'], memoria_tags[0]['c_y']
            cv2.arrowedLine(cam_view, (cx, cy), (px, py), (0, 0, 255), 4, tipLength=0.3)

            # --- RASTREO Y NAVEGACIÓN ---
            if not estela_robot or estela_robot[-1] != p_robot_radar: estela_robot.append(p_robot_radar)
            for i in range(1, len(estela_robot)):
                cv2.line(minimapa, estela_robot[i-1], estela_robot[i], (0, 100, 255), int(np.interp(i, [0, len(estela_robot)], [1, 3])))

            if mision_activa and len(ruta_pts_global) == 4:
                if estado_mision == 1: objetivo_actual = ruta_pts_global[0]; nombre_objetivo = "1. Est.1"
                elif estado_mision == 2: objetivo_actual = ruta_pts_global[1]; nombre_objetivo = "2. Esq. 1"
                elif estado_mision == 3: objetivo_actual = ruta_pts_global[2]; nombre_objetivo = "3. Est.2"
                elif estado_mision == 4: objetivo_actual = ruta_pts_global[3]; nombre_objetivo = "4. Esq. 2"

                if objetivo_actual:
                    tx, ty, _ = objetivo_actual
                    dx = tx - rx
                    dy = ty - ry
                    dist = math.hypot(dx, dy)
                    
                    # [LA MAGIA] Calculamos el error angular exacto directo en la cámara
                    target_yaw = math.atan2(dy, dx)
                    error_angular_grados = math.degrees(target_yaw - robot_yaw_visual)
                    # Normalizar entre -180 y 180
                    error_angular_grados = (error_angular_grados + 180) % 360 - 180
                    
                    # [NUEVO PAYLOAD] Agregamos el error_angular como el 5to dato
                    mensaje_red = f"{dx},{dy},{dist},{sentido_giro_global},{error_angular_grados:.2f}"
                    sock_udp.sendto(mensaje_red.encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))

                    pt_obj_radar = metros_a_pixeles_radar(tx, ty)
                    cv2.line(minimapa, p_robot_radar, pt_obj_radar, (0, 255, 255), 2)
                    cv2.circle(minimapa, pt_obj_radar, int(TOLERANCIA_LLEGADA_M * ESCALA_RADAR), (0, 255, 100), 1)
                    cv2.putText(minimapa, f"Target: {nombre_objetivo}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    
                    txt_error = f"Error Angular: {error_angular_grados:+.1f} deg"
                    cv2.putText(cam_view, txt_error, (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    if dist <= TOLERANCIA_LLEGADA_M:
                        estado_mision += 1
                        if estado_mision > 4: mision_activa = False

        cv2.rectangle(minimapa, (0, 0), (399, 399), (255, 255, 255), 2)
        cam_view[20:420, 1280-420:1280-20] = minimapa

        texto_estado = f"MISION ACTIVA (Giro {'IZQ' if sentido_giro_global==1 else 'DER'})" if mision_activa else "ESPERANDO ('s' para Iniciar)"
        color_estado = (0, 255, 0) if mision_activa else (0, 0, 255)
        cv2.putText(cam_view, texto_estado, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color_estado, 3)

        cv2.imshow("Dashboard Pickasso", cv2.resize(cam_view, (960, 540)))
        
        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord('q'): break
        elif tecla == ord('r'):
            memoria_tags.clear(); estela_robot.clear(); mision_activa = False; estado_mision = 0
        elif tecla == ord('s') and not mision_activa:
            if len(ruta_pts_global) == 4 and 0 in memoria_tags:
                hx, hy = memoria_tags[0]['m_x'], memoria_tags[0]['m_y']
                t1x, t1y = ruta_pts_global[0][0], ruta_pts_global[0][1]
                t2x, t2y = ruta_pts_global[2][0], ruta_pts_global[2][1]
                
                cross_z = (t1x - hx) * (t2y - hy) - (t1y - hy) * (t2x - hx)
                sentido_giro_global = 1 if cross_z < 0 else -1
                
                mision_activa = True
                estado_mision = 1
            else:
                print("\n[X] Error: Faltan Tags para iniciar.")

    cap.stop(); cv2.destroyAllWindows(); sock_udp.close()

if __name__ == '__main__':
    main()
