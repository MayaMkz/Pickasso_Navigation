import cv2
import numpy as np
import math
import threading
from collections import deque

# ==========================================
# 0. EL SECRETO DE CERO LATENCIA (MULTITHREADING)
# ==========================================
class CamaraIP_UltraRapida:
    def __init__(self, url):
        self.stream = cv2.VideoCapture(url)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if not self.stream.isOpened():
            raise Exception("No se pudo abrir DroidCam. Revisa la IP.")
            
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        # Arranca el "ladrón de frames" en otro núcleo del procesador
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                # Esto sobreescribe el frame viejo inmediatamente. CERO LAG.
                (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()


def main():
    # --- CONFIGURACIÓN PRINCIPAL ---
    marker_size = 0.063  
    station_threshold = 0.025  
    
    ESCALA_RADAR = 150  
    CENTRO_RADAR = (200, 200)  
    estela_robot = deque(maxlen=60)  
    memoria_tags = {} 
    
    camera_matrix = None
    dist_coeffs = None
    map1, map2 = None, None  # Mapas de aceleración
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    half_size = marker_size / 2.0
    obj_points = np.array([
        [-half_size,  half_size, 0],
        [ half_size,  half_size, 0],
        [ half_size, -half_size, 0],
        [-half_size, -half_size, 0]
    ], dtype=np.float32)

    # --- INICIAR CÁMARA TURBO ---
    droidcam_url = "http://10.48.97.233:4747/video" 
    print(f"[*] Conectando a {droidcam_url} en modo Multihilo...")
    
    try:
        cap = CamaraIP_UltraRapida(droidcam_url).start()
    except Exception as e:
        print(f"[!] {e}")
        return

    print("[OK] HUD Turbo Iniciado. Máxima fluidez lograda.")
    print(" >> Controles: 'r' para Reiniciar | 'q' para Salir")

    def metros_a_pixeles_radar(x_metros, y_metros):
        px = int(CENTRO_RADAR[0] + (x_metros * ESCALA_RADAR))
        py = int(CENTRO_RADAR[1] - (y_metros * ESCALA_RADAR)) 
        return (px, py)

    while True:
        # Tomar el frame más fresco al instante
        cv_image_raw = cap.read()
        if cv_image_raw is None: continue

        # --- OPTIMIZACIÓN: CÁLCULO DE MAPA SOLO 1 VEZ ---
        if camera_matrix is None:
            h, w = cv_image_raw.shape[:2]
            focal_length = w * 0.9 
            camera_matrix = np.array([
                [focal_length, 0, w / 2],
                [0, focal_length, h / 2],
                [0, 0, 1]
            ], dtype=np.float32)
            dist_coeffs = np.zeros((4, 1))
            
            # Pre-calcular mapas de distorsión para velocidad máxima
            map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, camera_matrix, (w,h), cv2.CV_32FC1)

        # --- OPTIMIZACIÓN: REMAP RÁPIDO EN VEZ DE UNDISTORT LENTO ---
        cam_view = cv2.remap(cv_image_raw, map1, map2, interpolation=cv2.INTER_LINEAR)
        
        # Lienzo del Minimapa
        minimapa = np.zeros((400, 400, 3), dtype=np.uint8)
        
        # Cuadrícula fina
        for i in range(0, 400, 50):
            cv2.line(minimapa, (i, 0), (i, 400), (30, 30, 30), 1)
            cv2.line(minimapa, (0, i), (400, i), (30, 30, 30), 1)

        try:
            gray = cv2.cvtColor(cam_view, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None:
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[i][0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if success:
                        cx = int(np.mean(corners[i][0][:, 0]))
                        cy = int(np.mean(corners[i][0][:, 1]))
                        
                        memoria_tags[marker_id] = {
                            'm_x': tvec[0][0], 'm_y': tvec[1][0],
                            'c_x': cx, 'c_y': cy
                        }

                        cv2.aruco.drawDetectedMarkers(cam_view, corners)
                        cv2.drawFrameAxes(cam_view, camera_matrix, dist_coeffs, rvec, tvec, marker_size)
                        
                        etiqueta = "Robot" if marker_id == 0 else f"Tag {marker_id}"
                        color_t = (0, 165, 255) if marker_id == 0 else (0, 255, 0)
                        cv2.putText(cam_view, etiqueta, (cx - 40, cy - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_t, 2)

            # DIBUJAR PISTA
            if 1 in memoria_tags and 2 in memoria_tags:
                x1, y1 = memoria_tags[1]['m_x'], memoria_tags[1]['m_y']
                x2, y2 = memoria_tags[2]['m_x'], memoria_tags[2]['m_y']
                
                p1 = metros_a_pixeles_radar(x1, y1)
                p2 = metros_a_pixeles_radar(x2, y2)
                p3 = metros_a_pixeles_radar(x2, y1)
                p4 = metros_a_pixeles_radar(x1, y2)
                
                puntos_circ = np.array([p1, p3, p2, p4], np.int32).reshape((-1, 1, 2))
                cv2.polylines(minimapa, [puntos_circ], isClosed=True, color=(200, 200, 200), thickness=2)
                
                for pt, nombre in zip([p1, p2], ["1", "2"]):
                    cv2.circle(minimapa, pt, 5, (0, 255, 0), -1)
                    cv2.putText(minimapa, nombre, (pt[0]+8, pt[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

            # DIBUJAR ROBOT Y EVALUAR ALINEACIÓN
            if 0 in memoria_tags:
                rx, ry = memoria_tags[0]['m_x'], memoria_tags[0]['m_y']
                rc_x, rc_y = memoria_tags[0]['c_x'], memoria_tags[0]['c_y']
                
                p_robot_radar = metros_a_pixeles_radar(rx, ry)
                
                if not estela_robot or estela_robot[-1] != p_robot_radar:
                    estela_robot.append(p_robot_radar)
                    
                if len(estela_robot) > 1:
                    for i in range(1, len(estela_robot)):
                        grosor = int(np.interp(i, [0, len(estela_robot)], [1, 3]))
                        cv2.line(minimapa, estela_robot[i-1], estela_robot[i], (0, 100, 255), grosor)

                cv2.circle(minimapa, p_robot_radar, 8, (0, 165, 255), -1)
                cv2.circle(cam_view, (rc_x, rc_y), 5, (0, 165, 255), -1)

                if 1 in memoria_tags:
                    t1_x, t1_y = memoria_tags[1]['m_x'], memoria_tags[1]['m_y']
                    t1c_x, t1c_y = memoria_tags[1]['c_x'], memoria_tags[1]['c_y']
                    
                    p1_radar = metros_a_pixeles_radar(t1_x, t1_y)
                    dist = math.hypot(t1_x - rx, t1_y - ry)
                    
                    if dist <= station_threshold:
                        cv2.circle(minimapa, p_robot_radar, 15, (0, 255, 0), 2)
                        cv2.rectangle(cam_view, (t1c_x - 70, t1c_y - 90), (t1c_x + 70, t1c_y - 60), (0, 0, 255), -1)
                        cv2.putText(cam_view, "REACHED", (t1c_x - 60, t1c_y - 68), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    else:
                        cv2.line(minimapa, p_robot_radar, p1_radar, (0, 0, 255), 1, cv2.LINE_AA)
                        cv2.line(cam_view, (rc_x, rc_y), (t1c_x, t1c_y), (255, 0, 255), 2, cv2.LINE_AA)
                        
                        mid_x, mid_y = int((rc_x + t1c_x)/2), int((rc_y + t1c_y)/2)
                        cv2.putText(cam_view, f"Dist: {dist:.3f}m", (mid_x + 10, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # PICTURE-IN-PICTURE (SOBREPONER)
            cv2.rectangle(minimapa, (0, 0), (399, 399), (255, 255, 255), 2)
            cam_view[20:420, 1280-420:1280-20] = minimapa

            # Reducir el tamaño FINAL de la ventana
            ventana_reducida = cv2.resize(cam_view, (960, 540))

            cv2.imshow("Dashboard Pickasso (Zero Lag)", ventana_reducida)
            
            # --- MANEJO DE TECLADO ---
            tecla = cv2.waitKey(1) & 0xFF
            if tecla == ord('q'):
                print("[*] Cerrando sistema...")
                break
            elif tecla == ord('r'):
                memoria_tags.clear()
                estela_robot.clear()
                print("[!] Memoria reiniciada.")

        except Exception as e:
            print(f"[!] Error procesando vista: {e}")

    cap.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
