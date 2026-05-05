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

    TOLERANCIA_LLEGADA_M = 0.05

    print(f"[*] Conectando a Cámara en {DROIDCAM_URL}...")
    try: cap = CamaraIP_UltraRapida(DROIDCAM_URL).start()
    except Exception as e: print(f"[!] {e}"); return

    print(f"[*] Abriendo canal UDP hacia {IP_RASPBERRY}:{PUERTO_UDP}...")
    sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # --- CONFIGURACIÓN LÓGICA ---
    marker_size = 0.063  
    ESCALA_RADAR = 150  
    CENTRO_RADAR = (200, 200)  
    estela_robot = deque(maxlen=60)  
    memoria_tags = {} 
    
    mision_activa = False
    estado_mision = 0  
    posicion_home = None 
    objetivo_actual = None
    nombre_objetivo = ""
    
    # Variable para guardar el sentido del circuito (1 = Izquierda, -1 = Derecha)
    sentido_giro_global = 1 

    # ==========================================
    # CARGA DE CALIBRACIÓN CHARUCO
    # ==========================================
    camera_matrix = None
    dist_coeffs = None
    map1 = None
    map2 = None

    print("[*] Buscando archivo de calibración...")
    fs = cv2.FileStorage("parametros_droidcam.yaml", cv2.FILE_STORAGE_READ)

    if fs.isOpened():
        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("dist_coeffs").mat()
        fs.release()
        print("[+] ¡Parámetros de lente cargados exitosamente!")
        
        # Pre-calcular mapas de corrección de distorsión (Mejora el rendimiento)
        w, h = 1280, 720 # Resolución fija de CamaraIP_UltraRapida
        map1, map2 = cv2.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, None, camera_matrix, (w, h), cv2.CV_32FC1)
    else:
        print("[-] ALERTA: No se encontró 'parametros_droidcam.yaml'. Se usarán parámetros genéricos.")
        # Parámetros genéricos por si pierdes el archivo
        w, h = 1280, 720
        focal_length = w * 0.9 
        camera_matrix = np.array([[focal_length, 0, w / 2], [0, focal_length, h / 2], [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((4, 1))
        map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, camera_matrix, (w,h), cv2.CV_32FC1)
    # ==========================================
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    half_size = marker_size / 2.0
    obj_points = np.array([[-half_size, half_size, 0], [half_size, half_size, 0], 
                           [half_size, -half_size, 0], [-half_size, -half_size, 0]], dtype=np.float32)

    def metros_a_pixeles_radar(x, y):
        return (int(CENTRO_RADAR[0] + (x * ESCALA_RADAR)), int(CENTRO_RADAR[1] - (y * ESCALA_RADAR)))

    while True:
        cv_image_raw = cap.read()
        if cv_image_raw is None: continue

        # --- APLICAR CORRECCIÓN DE LENTE ---
        # Se utilizan los mapas precalculados basados en el YAML
        cam_view = cv2.remap(cv_image_raw, map1, map2, interpolation=cv2.INTER_LINEAR)
        # -----------------------------------

        minimapa = np.zeros((400, 400, 3), dtype=np.uint8)
        
        for i in range(0, 400, 50):
            cv2.line(minimapa, (i, 0), (i, 400), (30, 30, 30), 1)
            cv2.line(minimapa, (0, i), (400, i), (30, 30, 30), 1)

        gray = cv2.cvtColor(cam_view, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            for i in range(len(ids)):
                m_id = int(ids[i][0])
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success:
                    cx, cy = int(np.mean(corners[i][0][:, 0])), int(np.mean(corners[i][0][:, 1]))
                    memoria_tags[m_id] = {'m_x': tvec[0][0], 'm_y': tvec[1][0], 'c_x': cx, 'c_y': cy}
                    cv2.aruco.drawDetectedMarkers(cam_view, corners)
                    color = (0, 165, 255) if m_id == 0 else (0, 255, 0)
                    cv2.putText(cam_view, f"Tag {m_id}" if m_id!=0 else "Robot", (cx-40, cy-40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        if 1 in memoria_tags and 2 in memoria_tags:
            t1x, t1y = memoria_tags[1]['m_x'], memoria_tags[1]['m_y']
            t2x, t2y = memoria_tags[2]['m_x'], memoria_tags[2]['m_y']
            vx, vy = t2x, t1y 

            if posicion_home:
                hx, hy = posicion_home
                puntos_circ = np.array([metros_a_pixeles_radar(hx, hy), metros_a_pixeles_radar(t1x, t1y), 
                                        metros_a_pixeles_radar(vx, vy), metros_a_pixeles_radar(t2x, t2y)], np.int32).reshape((-1, 1, 2))
                cv2.polylines(minimapa, [puntos_circ], isClosed=True, color=(100, 100, 100), thickness=2)

        if 0 in memoria_tags:
            rx, ry = memoria_tags[0]['m_x'], memoria_tags[0]['m_y']
            p_robot_radar = metros_a_pixeles_radar(rx, ry)
            
            if not estela_robot or estela_robot[-1] != p_robot_radar: estela_robot.append(p_robot_radar)
            for i in range(1, len(estela_robot)):
                cv2.line(minimapa, estela_robot[i-1], estela_robot[i], (0, 100, 255), int(np.interp(i, [0, len(estela_robot)], [1, 3])))
            cv2.circle(minimapa, p_robot_radar, 8, (0, 165, 255), -1)

            if mision_activa and 1 in memoria_tags and 2 in memoria_tags and posicion_home:
                t1x, t1y = memoria_tags[1]['m_x'], memoria_tags[1]['m_y']
                t2x, t2y = memoria_tags[2]['m_x'], memoria_tags[2]['m_y']

                if estado_mision == 1: objetivo_actual = (t1x, t1y); nombre_objetivo = "1. Estacion 1"
                elif estado_mision == 2: objetivo_actual = (t2x, t1y); nombre_objetivo = "2. Esquina Virtual"
                elif estado_mision == 3: objetivo_actual = (t2x, t2y); nombre_objetivo = "3. Estacion 2"
                elif estado_mision == 4: objetivo_actual = posicion_home; nombre_objetivo = "4. Posicion Inicial (HOME)"

                if objetivo_actual:
                    tx, ty = objetivo_actual
                    dx = tx - rx
                    dy = ty - ry
                    dist = math.hypot(dx, dy)
                    
                    mensaje_red = f"{dx},{dy},{dist},{sentido_giro_global}"
                    sock_udp.sendto(mensaje_red.encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))

                    pt_obj_radar = metros_a_pixeles_radar(tx, ty)
                    cv2.line(minimapa, p_robot_radar, pt_obj_radar, (0, 255, 255), 2)
                    cv2.circle(minimapa, pt_obj_radar, int(TOLERANCIA_LLEGADA_M * ESCALA_RADAR), (0, 255, 100), 1)
                    cv2.putText(minimapa, f"Target: {nombre_objetivo}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

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
            memoria_tags.clear(); estela_robot.clear(); mision_activa = False; estado_mision = 0; posicion_home = None
        elif tecla == ord('s') and not mision_activa:
            if 0 in memoria_tags and 1 in memoria_tags and 2 in memoria_tags:
                hx, hy = memoria_tags[0]['m_x'], memoria_tags[0]['m_y']
                t1x, t1y = memoria_tags[1]['m_x'], memoria_tags[1]['m_y']
                t2x, t2y = memoria_tags[2]['m_x'], memoria_tags[2]['m_y']
                
                cross_z = (t1x - hx) * (t2y - hy) - (t1y - hy) * (t2x - hx)
                
                if cross_z < 0:
                    sentido_giro_global = 1 
                    print("\n[!] Detección: El circuito se formará hacia la IZQUIERDA.")
                else:
                    sentido_giro_global = -1 
                    print("\n[!] Detección: El circuito se formará hacia la DERECHA.")
                
                posicion_home = (hx, hy)
                mision_activa = True
                estado_mision = 1
            else:
                print("\n[X] Error: Faltan Tags.")

    cap.stop(); cv2.destroyAllWindows(); sock_udp.close()

if __name__ == '__main__':
    main()
