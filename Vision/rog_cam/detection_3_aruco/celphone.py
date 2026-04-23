import cv2
import numpy as np
import math
from collections import deque

def main():
    # --- CONFIGURACIÓN PRINCIPAL ---
    marker_size = 0.063  
    station_threshold = 0.025  # Precisión centro a centro (2.5 cm)
    
    # --- CONFIGURACIÓN DEL RADAR VISUAL ---
    ESCALA_RADAR = 300  
    CENTRO_RADAR = (400, 400)  
    estela_robot = deque(maxlen=60)  
    
    # MEMORIA DEL SISTEMA
    # Ahora guarda X, Y (en metros) y px, py (pixeles de la cámara)
    memoria_tags = {} 
    
    camera_matrix = None
    dist_coeffs = None
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

    # --- CONEXIÓN DE DROIDCAM ---
    droidcam_url = "http://10.48.97.233:4747/video" 
    print(f"[*] Enlazando con cámara en: {droidcam_url}")
    cap = cv2.VideoCapture(droidcam_url)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"[!] ERROR: No se pudo abrir DroidCam.")
        return

    print("[OK] Sistema de Doble Monitor Iniciado.")
    print(" >> Controles: 'r' para Reiniciar memoria | 'q' para Salir")

    def metros_a_pixeles_radar(x_metros, y_metros):
        px = int(CENTRO_RADAR[0] + (x_metros * ESCALA_RADAR))
        py = int(CENTRO_RADAR[1] - (y_metros * ESCALA_RADAR)) 
        return (px, py)

    while True:
        cap.grab()
        ret, cv_image_raw = cap.retrieve()
        if not ret: continue

        if camera_matrix is None:
            h, w = cv_image_raw.shape[:2]
            focal_length = w * 0.9 
            camera_matrix = np.array([
                [focal_length, 0, w / 2],
                [0, focal_length, h / 2],
                [0, 0, 1]
            ], dtype=np.float32)
            dist_coeffs = np.zeros((4, 1))

        # Crear lienzos para las dos pantallas
        cam_view = cv2.undistort(cv_image_raw, camera_matrix, dist_coeffs)
        dashboard = np.zeros((800, 800, 3), dtype=np.uint8)
        
        # Cuadrícula del radar
        for i in range(0, 800, 100):
            cv2.line(dashboard, (i, 0), (i, 800), (30, 30, 30), 1)
            cv2.line(dashboard, (0, i), (800, i), (30, 30, 30), 1)

        try:
            gray = cv2.cvtColor(cam_view, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            # 1. ACTUALIZAR MEMORIA Y DIBUJAR MARCADORES FÍSICOS
            if ids is not None:
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[i][0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if success:
                        # Centro en pixeles de la cámara real
                        cx = int(np.mean(corners[i][0][:, 0]))
                        cy = int(np.mean(corners[i][0][:, 1]))
                        
                        # Guardar en memoria: X(m), Y(m), px_camara, py_camara
                        memoria_tags[marker_id] = {
                            'm_x': tvec[0][0], 'm_y': tvec[1][0],
                            'c_x': cx, 'c_y': cy
                        }

                        # Dibujar contornos solo en la vista de la cámara
                        cv2.aruco.drawDetectedMarkers(cam_view, corners)
                        cv2.drawFrameAxes(cam_view, camera_matrix, dist_coeffs, rvec, tvec, marker_size)
                        
                        etiqueta = "Robot" if marker_id == 0 else f"Tag {marker_id}"
                        color_t = (0, 165, 255) if marker_id == 0 else (0, 255, 0)
                        cv2.putText(cam_view, etiqueta, (cx - 40, cy - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_t, 2)

            # ==========================================
            # 2. DIBUJAR LA PISTA EN EL RADAR (Tag 1 y 2)
            # ==========================================
            if 1 in memoria_tags and 2 in memoria_tags:
                x1, y1 = memoria_tags[1]['m_x'], memoria_tags[1]['m_y']
                x2, y2 = memoria_tags[2]['m_x'], memoria_tags[2]['m_y']
                
                # Convertir a pixeles del radar
                p1 = metros_a_pixeles_radar(x1, y1)
                p2 = metros_a_pixeles_radar(x2, y2)
                p3 = metros_a_pixeles_radar(x2, y1)
                p4 = metros_a_pixeles_radar(x1, y2)
                
                # Trazar rectángulo en el radar
                puntos_circ = np.array([p1, p3, p2, p4], np.int32).reshape((-1, 1, 2))
                cv2.polylines(dashboard, [puntos_circ], isClosed=True, color=(200, 200, 200), thickness=2)
                
                for pt, nombre in zip([p1, p2], ["Tag 1", "Tag 2"]):
                    cv2.circle(dashboard, pt, 8, (0, 255, 0), -1)
                    cv2.putText(dashboard, nombre, (pt[0]+15, pt[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # ==========================================
            # 3. DIBUJAR ROBOT Y EVALUAR ALINEACIÓN EN AMBAS PANTALLAS
            # ==========================================
            if 0 in memoria_tags:
                # Datos del Robot
                rx, ry = memoria_tags[0]['m_x'], memoria_tags[0]['m_y']
                rc_x, rc_y = memoria_tags[0]['c_x'], memoria_tags[0]['c_y']
                
                p_robot_radar = metros_a_pixeles_radar(rx, ry)
                
                # --- RADAR: Estela e ícono ---
                if not estela_robot or estela_robot[-1] != p_robot_radar:
                    estela_robot.append(p_robot_radar)
                    
                if len(estela_robot) > 1:
                    for i in range(1, len(estela_robot)):
                        grosor = int(np.interp(i, [0, len(estela_robot)], [1, 4]))
                        cv2.line(dashboard, estela_robot[i-1], estela_robot[i], (0, 100, 255), grosor)

                cv2.circle(dashboard, p_robot_radar, 12, (0, 165, 255), -1)
                cv2.circle(dashboard, p_robot_radar, 16, (0, 165, 255), 1) 
                cv2.putText(dashboard, f"PICKASSO X:{rx:.2f}m Y:{ry:.2f}m", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

                # --- CÁMARA REAL: Punto central de memoria del robot ---
                cv2.circle(cam_view, (rc_x, rc_y), 5, (0, 165, 255), -1)

                # Si tenemos el Tag 1 en memoria, trazar la ruta hacia él
                if 1 in memoria_tags:
                    t1_x, t1_y = memoria_tags[1]['m_x'], memoria_tags[1]['m_y']
                    t1c_x, t1c_y = memoria_tags[1]['c_x'], memoria_tags[1]['c_y']
                    
                    p1_radar = metros_a_pixeles_radar(t1_x, t1_y)
                    dist = math.hypot(t1_x - rx, t1_y - ry)
                    
                    if dist <= station_threshold:
                        # [ALERTA REACHED - EN RADAR]
                        cv2.circle(dashboard, p_robot_radar, 30, (0, 255, 0), 3)
                        cv2.putText(dashboard, "ALINEACION PERFECTA (REACHED)", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 3)
                        
                        # [ALERTA REACHED - EN CAMARA REAL]
                        cv2.rectangle(cam_view, (t1c_x - 70, t1c_y - 90), (t1c_x + 70, t1c_y - 60), (0, 0, 255), -1)
                        cv2.putText(cam_view, "REACHED", (t1c_x - 60, t1c_y - 68), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    else:
                        # [DIBUJAR RUTA - EN RADAR]
                        cv2.line(dashboard, p_robot_radar, p1_radar, (0, 0, 255), 1, cv2.LINE_AA)
                        cv2.putText(dashboard, f"Distancia Centro-Centro: {dist:.3f}m", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        
                        # [DIBUJAR RUTA - EN CAMARA REAL]
                        cv2.line(cam_view, (rc_x, rc_y), (t1c_x, t1c_y), (255, 0, 255), 2, cv2.LINE_AA)
                        cv2.circle(cam_view, (t1c_x, t1c_y), 5, (0, 255, 0), -1)
                        mid_x, mid_y = int((rc_x + t1c_x)/2), int((rc_y + t1c_y)/2)
                        cv2.putText(cam_view, f"Dist: {dist:.3f}m", (mid_x + 10, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # --- MOSTRAR LAS DOS PANTALLAS ---
            cv2.imshow("1. Camara Real (Pickasso)", cam_view)
            cv2.imshow("2. Telemetria Radar", dashboard)
            
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

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
