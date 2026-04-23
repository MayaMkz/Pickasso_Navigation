import cv2
import numpy as np
import math
from collections import deque

def main():
    # --- CONFIGURACIÓN PRINCIPAL ---
    marker_size = 0.063  
    station_threshold = 0.025  # Precisión de centro a centro (2.5 cm)
    
    # --- CONFIGURACIÓN DEL RADAR VISUAL ---
    ESCALA_RADAR = 300  
    CENTRO_RADAR = (400, 400)  
    estela_robot = deque(maxlen=60)  
    
    # [!] NUEVO: LA MEMORIA DEL SISTEMA
    # Aquí guardaremos la última posición conocida de cada cosa
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
    print(f"[*] Enlazando con cámara Samsung en: {droidcam_url}")
    cap = cv2.VideoCapture(droidcam_url)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"[!] ERROR: No se pudo abrir DroidCam.")
        return

    print("[OK] Radar con Memoria Espacial Iniciado.")
    print(" >> Controles: 'r' para Reiniciar memoria | 'q' para Salir")

    def metros_a_pixeles(x_metros, y_metros):
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

        dashboard = np.zeros((800, 800, 3), dtype=np.uint8)
        for i in range(0, 800, 100):
            cv2.line(dashboard, (i, 0), (i, 800), (30, 30, 30), 1)
            cv2.line(dashboard, (0, i), (800, i), (30, 30, 30), 1)

        try:
            gray = cv2.cvtColor(cv_image_raw, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            # 1. ACTUALIZAR MEMORIA CON LO QUE SE VE EN ESTE MILISEGUNDO
            if ids is not None:
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[i][0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if success:
                        # Guardamos X y Y en metros para la memoria
                        memoria_tags[marker_id] = (tvec[0][0], tvec[1][0]) 

            # ==========================================
            # 2. DIBUJAR TODO BASADO EXCLUSIVAMENTE EN LA MEMORIA
            # ==========================================
            
            # DIBUJAR PISTA (Si recordamos dónde están el 1 y el 2)
            if 1 in memoria_tags and 2 in memoria_tags:
                x1, y1 = memoria_tags[1]
                x2, y2 = memoria_tags[2]
                x3, y3 = x2, y1
                x4, y4 = x1, y2
                
                p1 = metros_a_pixeles(x1, y1)
                p2 = metros_a_pixeles(x2, y2)
                p3 = metros_a_pixeles(x3, y3)
                p4 = metros_a_pixeles(x4, y4)
                
                puntos_circuito = np.array([p1, p3, p2, p4], np.int32).reshape((-1, 1, 2))
                cv2.polylines(dashboard, [puntos_circuito], isClosed=True, color=(200, 200, 200), thickness=2)
                
                for pt, nombre in zip([p1, p2], ["Tag 1", "Tag 2"]):
                    cv2.circle(dashboard, pt, 8, (0, 255, 0), -1)
                    cv2.drawMarker(dashboard, pt, (0, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
                    cv2.putText(dashboard, nombre, (pt[0]+15, pt[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # DIBUJAR ROBOT Y EVALUAR ALINEACIÓN (Si recordamos dónde está el 0)
            if 0 in memoria_tags:
                rx, ry = memoria_tags[0]
                p_robot = metros_a_pixeles(rx, ry)
                
                # Actualizar estela solo si el robot se movió
                if not estela_robot or estela_robot[-1] != p_robot:
                    estela_robot.append(p_robot)
                    
                if len(estela_robot) > 1:
                    for i in range(1, len(estela_robot)):
                        grosor = int(np.interp(i, [0, len(estela_robot)], [1, 4]))
                        cv2.line(dashboard, estela_robot[i-1], estela_robot[i], (0, 100, 255), grosor)

                cv2.circle(dashboard, p_robot, 12, (0, 165, 255), -1)
                cv2.circle(dashboard, p_robot, 16, (0, 165, 255), 1) 
                cv2.putText(dashboard, f"PICKASSO X:{rx:.2f}m Y:{ry:.2f}m", 
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

                # Evaluar distancia hacia el Tag 1 guardado en memoria
                if 1 in memoria_tags:
                    p1 = metros_a_pixeles(memoria_tags[1][0], memoria_tags[1][1])
                    dist = math.hypot(memoria_tags[1][0] - rx, memoria_tags[1][1] - ry)
                    
                    if dist <= station_threshold:
                        cv2.circle(dashboard, p_robot, 30, (0, 255, 0), 3)
                        cv2.putText(dashboard, "ALINEACION PERFECTA (REACHED)", 
                                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 3)
                    else:
                        cv2.line(dashboard, p_robot, p1, (0, 0, 255), 1, cv2.LINE_AA)
                        cv2.putText(dashboard, f"Distancia Centro-Centro: {dist:.3f}m", 
                                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Mostrar si hay tags en memoria en la esquina superior derecha
            tags_activos = list(memoria_tags.keys())
            cv2.putText(dashboard, f"Memoria: {tags_activos}", (580, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

            cv2.imshow("Telemetria Pickasso (Memoria Activa)", dashboard)
            
            # --- MANEJO DE TECLADO ---
            tecla = cv2.waitKey(1) & 0xFF
            if tecla == ord('q'):
                print("[*] Cerrando telemetría...")
                break
            elif tecla == ord('r'):
                # EL BOTÓN DE REINICIO
                memoria_tags.clear()
                estela_robot.clear()
                print("[!] Memoria reiniciada. Mueve las etiquetas para trazar nueva ruta.")

        except Exception as e:
            print(f"[!] Error procesando telemetría: {e}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
