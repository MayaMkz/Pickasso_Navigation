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
    # --- CONFIGURACIÓN DE RED ---
    DROIDCAM_URL = "http://10.48.97.233:4747/video" 
    IP_RASPBERRY = "192.168.137.240" 
    PUERTO_UDP = 5005

    print(f"[*] Conectando a Cámara en {DROIDCAM_URL}...")
    try: cap = CamaraIP_UltraRapida(DROIDCAM_URL).start()
    except Exception as e: print(f"[!] {e}"); return

    print(f"[*] Abriendo canal de datos hacia Raspberry Pi en {IP_RASPBERRY}:{PUERTO_UDP}...")
    sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # --- CONFIGURACIÓN LÓGICA Y VISUAL ---
    marker_size = 0.063  
    station_threshold = 0.05  # 5 cm para dar por válido el punto y girar al siguiente
    
    ESCALA_RADAR = 150  
    CENTRO_RADAR = (200, 200)  
    estela_robot = deque(maxlen=60)  
    memoria_tags = {} 
    
    mision_activa = False
    estado_mision = 0  
    posicion_home = None
    objetivo_actual = None
    nombre_objetivo = ""

    camera_matrix = None; dist_coeffs = None; map1 = None; map2 = None
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    half_size = marker_size / 2.0
    obj_points = np.array([[-half_size, half_size, 0], [half_size, half_size, 0], 
                           [half_size, -half_size, 0], [-half_size, -half_size, 0]], dtype=np.float32)

    print("[OK] Dashboard Listo.")
    print(" >> Controles: 's' Iniciar Misión | 'r' Reiniciar | 'q' Salir")

    def metros_a_pixeles_radar(x, y):
        return (int(CENTRO_RADAR[0] + (x * ESCALA_RADAR)), int(CENTRO_RADAR[1] - (y * ESCALA_RADAR)))

    while True:
        cv_image_raw = cap.read()
        if cv_image_raw is None: continue

        if camera_matrix is None:
            h, w = cv_image_raw.shape[:2]
            focal_length = w * 0.9 
            camera_matrix = np.array([[focal_length, 0, w / 2], [0, focal_length, h / 2], [0, 0, 1]], dtype=np.float32)
            dist_coeffs = np.zeros((4, 1))
            map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, camera_matrix, (w,h), cv2.CV_32FC1)

        cam_view = cv2.remap(cv_image_raw, map1, map2, interpolation=cv2.INTER_LINEAR)
        minimapa = np.zeros((400, 400, 3), dtype=np.uint8)
        
        for i in range(0, 400, 50):
            cv2.line(minimapa, (i, 0), (i, 400), (30, 30, 30), 1)
            cv2.line(minimapa, (0, i), (400, i), (30, 30, 30), 1)

        gray = cv2.cvtColor(cam_view, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        # 1. ACTUALIZAR MEMORIA Y DIBUJAR TAGS FÍSICOS
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

        # 2. DIBUJAR PISTA EN EL RADAR
        if 1 in memoria_tags and 2 in memoria_tags:
            x1, y1 = memoria_tags[1]['m_x'], memoria_tags[1]['m_y']
            x2, y2 = memoria_tags[2]['m_x'], memoria_tags[2]['m_y']
            
            puntos_circ = np.array([metros_a_pixeles_radar(x1, y1), metros_a_pixeles_radar(x2, y1), 
                                    metros_a_pixeles_radar(x2, y2), metros_a_pixeles_radar(x1, y2)], np.int32).reshape((-1, 1, 2))
            cv2.polylines(minimapa, [puntos_circ], isClosed=True, color=(100, 100, 100), thickness=1)

        # 3. LÓGICA DE SECUENCIA Y NAVEGACIÓN
        if 0 in memoria_tags:
            rx, ry = memoria_tags[0]['m_x'], memoria_tags[0]['m_y']
            p_robot_radar = metros_a_pixeles_radar(rx, ry)
            
            if not estela_robot or estela_robot[-1] != p_robot_radar: estela_robot.append(p_robot_radar)
            for i in range(1, len(estela_robot)):
                cv2.line(minimapa, estela_robot[i-1], estela_robot[i], (0, 100, 255), int(np.interp(i, [0, len(estela_robot)], [1, 3])))

            cv2.circle(minimapa, p_robot_radar, 8, (0, 165, 255), -1)
            cv2.circle(cam_view, (memoria_tags[0]['c_x'], memoria_tags[0]['c_y']), 5, (0, 165, 255), -1)

            # MÁQUINA DE ESTADOS (LA SECUENCIA EXACTA DE TU DIBUJO)
            if mision_activa and 1 in memoria_tags and 2 in memoria_tags:
                if estado_mision == 1:
                    objetivo_actual = (memoria_tags[1]['m_x'], memoria_tags[1]['m_y'])
                    nombre_objetivo = "1. Tag 1"
                elif estado_mision == 2:
                    # Virtual 1 (Cruce de Tag 2 y Tag 1)
                    objetivo_actual = (memoria_tags[2]['m_x'], memoria_tags[1]['m_y'])
                    nombre_objetivo = "2. Esquina Virtual"
                elif estado_mision == 3:
                    objetivo_actual = (memoria_tags[2]['m_x'], memoria_tags[2]['m_y'])
                    nombre_objetivo = "3. Tag 2"
                elif estado_mision == 4:
                    objetivo_actual = posicion_home
                    nombre_objetivo = "4. Regreso a Casa"

                # ENVIAR ÓRDENES
                if objetivo_actual:
                    tx, ty = objetivo_actual
                    dx = tx - rx
                    dy = ty - ry
                    dist = math.hypot(dx, dy)
                    
                    print(f"\r[TX] Target: {nombre_objetivo} | dx: {dx:+.3f} | dy: {dy:+.3f} | Dist: {dist:.3f}m   ", end="")
                    mensaje_red = f"{dx},{dy},{dist}"
                    sock_udp.sendto(mensaje_red.encode('utf-8'), (IP_RASPBERRY, PUERTO_UDP))

                    cv2.line(minimapa, p_robot_radar, metros_a_pixeles_radar(tx, ty), (0, 255, 255), 2)
                    cv2.putText(minimapa, f"Target: {nombre_objetivo}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                    if dist <= station_threshold:
                        print(f"\n[!] {nombre_objetivo} Alcanzado. Girando al siguiente...")
                        estado_mision += 1
                        if estado_mision > 4:
                            mision_activa = False
                            print("\n[★★★] RECTANGULO COMPLETADO [★★★]\n")

        # PICTURE-IN-PICTURE Y VISUALIZACIÓN
        cv2.rectangle(minimapa, (0, 0), (399, 399), (255, 255, 255), 2)
        cam_view[20:420, 1280-420:1280-20] = minimapa

        texto_estado = "MISION EN CURSO" if mision_activa else ("ESPERANDO (Presiona 's')" if 1 in memoria_tags and 2 in memoria_tags else "FALTAN TAGS")
        color_estado = (0, 255, 0) if mision_activa else (0, 0, 255)
        cv2.putText(cam_view, texto_estado, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color_estado, 3)

        ventana_reducida = cv2.resize(cam_view, (960, 540))
        cv2.imshow("Dashboard Pickasso", ventana_reducida)
        
        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord('q'): break
        elif tecla == ord('r'):
            memoria_tags.clear(); estela_robot.clear(); mision_activa = False; estado_mision = 0
            print("\n[!] Memoria reiniciada.")
        elif tecla == ord('s') and not mision_activa:
            if 0 in memoria_tags and 1 in memoria_tags and 2 in memoria_tags:
                posicion_home = (memoria_tags[0]['m_x'], memoria_tags[0]['m_y'])
                mision_activa = True
                estado_mision = 1
                print("\n\n[>>>] INICIANDO RUTEO DE RECTANGULO [>>>]")
            else:
                print("\n[X] Error: La cámara necesita ver el Robot, el Tag 1 y el Tag 2 para arrancar.")

    cap.stop(); cv2.destroyAllWindows(); sock_udp.close()

if __name__ == '__main__':
    main()
