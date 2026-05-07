import cv2
import numpy as np
import math
import threading
import socket
import time
from collections import deque

# ==========================================
# CÁMARA TURBO (CERO LATENCIA Y ANTI-DUPLICADOS)
# ==========================================
class CamaraIP_UltraRapida:
    def __init__(self, url):
        self.stream = cv2.VideoCapture(url)
        # Resolución optimizada para cero lag
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.stream.isOpened(): raise Exception("Error DroidCam.")
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
        self.frame_nuevo = True

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            if not self.grabbed: 
                self.stop()
            else: 
                (self.grabbed, self.frame) = self.stream.read()
                self.frame_nuevo = True

    def read(self): 
        if self.frame_nuevo:
            self.frame_nuevo = False
            return self.frame
        return None

    def stop(self): 
        self.stopped = True; self.stream.release()

# ==========================================
# PROGRAMA PRINCIPAL
# ==========================================
def main():
    DROIDCAM_URL = "http://192.168.137.24:4747/video"
    IP_RASPBERRY = "192.168.137.240"                 
    PUERTO_UDP = 5005

    # =================================================================
    # PARÁMETROS CRÍTICOS DE NAVEGACIÓN Y FÍSICA
    # =================================================================
    OFFSET_MESA_M = 0.4 
    LOOKAHEAD_FIJO = 0.20  
    TOLERANCIA_LLEGADA_M = 0.05  
    OFFSET_X_TAG = 0.00    
    OFFSET_Y_TAG = 0.15    

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
    posicion_home = None 

    print("[*] Buscando archivo de calibración...")
    fs = cv2.FileStorage("parametros_droidcam.yaml", cv2.FILE_STORAGE_READ)
    
    if fs.isOpened():
        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("dist_coeffs").mat()
        fs.release()
        print("[OK] Calibración cargada.")
    
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    
    # OPTIMIZACIÓN DE ARUCO PARA VELOCIDAD
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    aruco_params.adaptiveThreshWinSizeStep = 20 
    
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    half_size = marker_size / 2.0
    obj_points = np.array([[-half_size, half_size, 0], [half_size, half_size, 0], 
                           [half_size, -half_size, 0], [-half_size, -half_size, 0]], dtype=np.float32)

    def metros_a_pixeles_radar(x, y):
        return (int(CENTRO_RADAR[0] + (x * ESCALA_RADAR)), int(CENTRO_RADAR[1] - (y * ESCALA_RADAR)))

    ruta_pts_global = []

    while True:
        cv_image_raw = cap.read()
        
        # Bandera de FPS: descansa CPU si no hay frame nuevo
        if cv_image_raw is None: 
            time.sleep(0.005)
            continue

        # Convertir a grises directamente (sin remap)
        gray = cv2.cvtColor(cv_image_raw, cv2.COLOR_BGR2GRAY)
        cam_view = cv_image_raw.copy()
        
        minimapa = np.zeros((400, 400, 3), dtype=np.uint8)
        
        for i in range(0, 400, 50):
            cv2.line(minimapa, (i, 0), (i, 400), (30, 30, 30), 1)
            cv2.line(minimapa, (0, i), (400, i), (30, 30, 30), 1)

        # Detectar ArUcos
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            for i in range(len(ids)):
                m_id = int(ids[i][0])
                # Pasamos dist_coeffs directamente aquí para que ArUco maneje su propia distorsión
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    cx, cy = int(np.mean(corners[i][0][:, 0])), int(np.mean(corners[i][0][:, 1]))
                    memoria_tags[m_id] = {'m_x': tvec[0][0], 'm_y': tvec[1][0], 'm_z': tvec[2][0], 'c_x': cx, 'c_y': cy, 'rvec': rvec, 'tvec': tvec}
                    cv2.aruco.drawDetectedMarkers(cam_view, corners)
                    color = (0, 165, 255) if m_id == 0 else (0, 255, 0)
                    cv2.putText(cam_view, f"Tag {m_id}" if m_id!=0 else "Robot", (cx-40, cy-40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # =========================================================
        # GEOMETRÍA DEL RECTÁNGULO
        # =========================================================
        if 1 in memoria_tags and 2 in memoria_tags:
            t1x, t1y, t1z = memoria_tags[1]['m_x'], memoria_tags[1]['m_y'], memoria_tags[1]['m_z']
            t2x, t2y, t2z = memoria_tags[2]['m_x'], memoria_tags[2]['m_y'], memoria_tags[2]['m_z']
            
            cz = (t1z + t2z) / 2.0
            
            mesa_pts = [(t1x, t1y, t1z), (t2x, t1y, cz), (t2x, t2y, t2z), (t1x, t2y, cz)]
            centro_mesa_x, centro_mesa_y = (t1x + t2x) / 2.0, (t1y + t2y) / 2.0

            def expandir_punto(px, py, pz):
                nx = px + OFFSET_MESA_M if px > centro_mesa_x else px - OFFSET_MESA_M
                ny = py + OFFSET_MESA_M if py > centro_mesa_y else py - OFFSET_MESA_M
                return (nx, ny, pz)

            ruta_pts_global = [expandir_punto(*p) for p in mesa_pts]

            pts_radar_mesa = np.array([metros_a_pixeles_radar(p[0], p[1]) for p in mesa_pts], np.int32).reshape((-1, 1, 2))
            cv2.polylines(minimapa, [pts_radar_mesa], isClosed=True, color=(255, 150, 50), thickness=2)
            
            pts_radar_ruta = np.array([metros_a_pixeles_radar(p[0], p[1]) for p in ruta_pts_global], np.int32).reshape((-1, 1, 2))
            cv2.polylines(minimapa, [pts_radar_ruta], isClosed=True, color=(0, 255, 0), thickness=2)

        # =========================================================
        # NAVEGACIÓN PURE PURSUIT Y ZANAHORIA MÓVIL
        # =========================================================
        if 0 in memoria_tags:
            rx, ry = memoria_tags[0]['m_x'], memoria_tags[0]['m_y']
            rvec, tvec = memoria_tags[0]['rvec'], memoria_tags[0]['tvec']
            
            R, _ = cv2.Rodrigues(rvec)
            
            vector_frente_cam = R @ np.array([[0.0], [-0.15], [0.0]])
            fx, fy = rx + vector_frente_cam[0][0], ry + vector_frente_cam[1][0]
            robot_yaw_visual = math.atan2(vector_frente_cam[1][0], vector_frente_cam[0][0])
            
            vector_offset = R @ np.array([[OFFSET_X_TAG], [OFFSET_Y_TAG], [0.0]])
            centro_carro_x = rx + vector_offset[0][0]
            centro_carro_y = ry + vector_offset[1][0]

            p_centro_radar = metros_a_pixeles_radar(centro_carro_x, centro_carro_y)
            p_frente_radar = metros_a_pixeles_radar(fx, fy)
            cv2.arrowedLine(minimapa, p_centro_radar, p_frente_radar, (0, 0, 255), 2, tipLength=0.3)
            cv2.circle(minimapa, p_centro_radar, 4, (255, 0, 0), -1)

            if not estela_robot or estela_robot[-1] != p_centro_radar: estela_robot.append(p_centro_radar)
            for i in range(1, len(estela_robot)):
                cv2.line(minimapa, estela_robot[i-1], estela_robot[i], (0, 100, 255), int(np.interp(i, [0, len(estela_robot)], [1, 3])))

            if mision_activa and len(ruta_pts_global) == 4 and posicion_home is not None:
                origen_linea = None
                if estado_mision == 1: 
                    origen_linea = posicion_home
                    objetivo_actual = ruta_pts_global[0]; nombre_objetivo = "1. Est.1"
                elif estado_mision == 2: 
                    origen_linea = ruta_pts_global[0]
                    objetivo_actual = ruta_pts_global[1]; nombre_objetivo = "2. Esq. 1"
                elif estado_mision == 3: 
                    origen_linea = ruta_pts_global[1]
                    objetivo_actual = ruta_pts_global[2]; nombre_objetivo = "3. Est.2"
                elif estado_mision == 4: 
                    origen_linea = ruta_pts_global[2]
                    objetivo_actual = ruta_pts_global[3]; nombre_objetivo = "4. Esq. 2"

                if objetivo_actual and origen_linea:
                    tx_fin, ty_fin, _ = objetivo_actual
                    ox, oy, _ = origen_linea
                    
                    dist_meta_real = math.hypot(tx_fin - centro_carro_x, ty_fin - centro_carro_y)
                    
                    if dist_meta_real <= TOLERANCIA_LLEGADA_M:
                        v_lineal = 0.0; omega_ref = 0.0
                        estado_mision += 1
                        if estado_mision > 4: mision_activa = False
                    else:
                        L_linea = math.hypot(tx_fin - ox, ty_fin - oy)
                        if L_linea == 0: L_linea = 0.001
                        
                        ux, uy = (tx_fin - ox) / L_linea, (ty_fin - oy) / L_linea
                        
                        vx, vy = centro_carro_x - ox, centro_carro_y - oy
                        proyeccion = (vx * ux) + (vy * uy)
                        
                        distancia_virtual = min(proyeccion + LOOKAHEAD_FIJO, L_linea)
                        tx_zanahoria = ox + (distancia_virtual * ux)
                        ty_zanahoria = oy + (distancia_virtual * uy)
                        
                        dx_meta = tx_zanahoria - centro_carro_x
                        dy_meta = ty_zanahoria - centro_carro_y
                        dist_zanahoria = math.hypot(dx_meta, dy_meta)
                        
                        angulo_hacia_meta = math.atan2(dy_meta, dx_meta)
                        alpha = angulo_hacia_meta - robot_yaw_visual
                        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
                        
                        v_lineal = 0.2 
                        ld_seguro = max(dist_zanahoria, 0.1) 
                        omega_ref = (2.0 * v_lineal * math.sin(alpha)) / ld_seguro
                        
                        MAX_OMEGA = 1.2 
                        omega_ref = max(-MAX_OMEGA, min(MAX_OMEGA, omega_ref))
                        
                        pt_zanahoria_radar = metros_a_pixeles_radar(tx_zanahoria, ty_zanahoria)
                        cv2.circle(minimapa, pt_zanahoria_radar, 5, (255, 150, 0), -1)

                    mensaje_red = f"{v_lineal:.3f},{omega_ref:.3f},{dist_meta_real:.3f},{estado_mision}"
                    sock_udp.sendto(mensaje_red.encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))

                    pt_obj_radar = metros_a_pixeles_radar(tx_fin, ty_fin)
                    cv2.line(minimapa, p_centro_radar, pt_obj_radar, (255, 0, 255), 2) 
                    cv2.circle(minimapa, pt_obj_radar, int(TOLERANCIA_LLEGADA_M * ESCALA_RADAR), (0, 255, 100), 1)
                    
                    cv2.putText(minimapa, f"Target: {nombre_objetivo}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    cv2.putText(cam_view, f"V: {v_lineal:.2f} m/s", (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.putText(cam_view, f"W: {omega_ref:+.2f} rad/s", (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Limitar la inserción del minimapa para que no se salga de la ventana de 640x480
        cv2.rectangle(minimapa, (0, 0), (399, 399), (255, 255, 255), 2)
        cam_view[20:420, 640-420:640-20] = minimapa  

        texto_estado = "MISION ACTIVA" if mision_activa else "ESPERANDO ('s' para Iniciar)"
        color_estado = (0, 255, 0) if mision_activa else (0, 0, 255)
        cv2.putText(cam_view, texto_estado, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color_estado, 3)
        
        cv2.imshow("Dashboard Pickasso", cam_view)
        
        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord('q'): break
        elif tecla == ord('r'):
            memoria_tags.clear(); estela_robot.clear(); mision_activa = False; estado_mision = 0; posicion_home = None
        elif tecla == ord('s') and not mision_activa:
            if len(ruta_pts_global) == 4 and 0 in memoria_tags:
                posicion_home = (memoria_tags[0]['m_x'], memoria_tags[0]['m_y'], memoria_tags[0]['m_z'])
                mision_activa = True
                estado_mision = 1
            else:
                print("\n[X] Error: Faltan Tags para iniciar.")

    cap.stop(); cv2.destroyAllWindows(); sock_udp.close()

if __name__ == '__main__':
    main()
