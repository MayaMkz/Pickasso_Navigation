import cv2
import numpy as np
import math
from collections import deque

def main():
    # --- CONFIGURACIÓN PRINCIPAL ---
    marker_size = 0.063  
    
    # --- CONFIGURACIÓN DEL RADAR VISUAL ---
    ESCALA_RADAR = 300  # Cuántos pixeles en pantalla equivalen a 1 metro real
    CENTRO_RADAR = (400, 400)  # El centro de nuestra pantalla de 800x800
    estela_robot = deque(maxlen=60)  # Memoria para el rastro del carrito (últimos 60 puntos)
    
    # --- INICIALIZACIÓN DE VISIÓN ---
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
    droidcam_url = "http://10.48.97.233:4747/video" # ¡Asegúrate de que sea tu IP actual!
    print(f"[*] Enlazando con cámara Samsung en: {droidcam_url}")
    cap = cv2.VideoCapture(droidcam_url)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"[!] ERROR: No se pudo abrir DroidCam.")
        return

    print("[OK] Radar de Telemetría Iniciado. Presiona 'q' para salir.")

    # Función auxiliar para convertir Metros Reales a Pixeles de Pantalla
    def metros_a_pixeles(x_metros, y_metros):
        px = int(CENTRO_RADAR[0] + (x_metros * ESCALA_RADAR))
        py = int(CENTRO_RADAR[1] - (y_metros * ESCALA_RADAR)) # Restamos porque en pantalla la Y va hacia abajo
        return (px, py)

    # --- BUCLE PRINCIPAL ---
    while True:
        cap.grab()
        ret, cv_image_raw = cap.retrieve()
        if not ret:
            continue

        if camera_matrix is None:
            h, w = cv_image_raw.shape[:2]
            focal_length = w * 0.9 
            camera_matrix = np.array([
                [focal_length, 0, w / 2],
                [0, focal_length, h / 2],
                [0, 0, 1]
            ], dtype=np.float32)
            dist_coeffs = np.zeros((4, 1))

        # Crear el lienzo negro (Dashboard) de 800x800 pixeles
        dashboard = np.zeros((800, 800, 3), dtype=np.uint8)
        
        # Dibujar una cuadrícula de fondo estilo radar
        for i in range(0, 800, 100):
            cv2.line(dashboard, (i, 0), (i, 800), (30, 30, 30), 1)
            cv2.line(dashboard, (0, i), (800, i), (30, 30, 30), 1)

        try:
            gray = cv2.cvtColor(cv_image_raw, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            poses_3d = {}

            if ids is not None:
                # 1. Obtener coordenadas absolutas (X, Y)
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[i][0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if success:
                        poses_3d[marker_id] = (tvec[0][0], tvec[1][0]) # Solo X y Y

                # 2. DIBUJAR LA PISTA (Si vemos Tag 1 y Tag 2)
                if 1 in poses_3d and 2 in poses_3d:
                    x1, y1 = poses_3d[1]
                    x2, y2 = poses_3d[2]
                    
                    # Calcular esquinas virtuales (3 y 4)
                    x3, y3 = x2, y1
                    x4, y4 = x1, y2
                    
                    # Convertir todo a pixeles
                    p1 = metros_a_pixeles(x1, y1)
                    p2 = metros_a_pixeles(x2, y2)
                    p3 = metros_a_pixeles(x3, y3)
                    p4 = metros_a_pixeles(x4, y4)
                    
                    # Trazar el rectángulo del circuito (Línea Blanca)
                    puntos_circuito = np.array([p1, p3, p2, p4], np.int32).reshape((-1, 1, 2))
                    cv2.polylines(dashboard, [puntos_circuito], isClosed=True, color=(200, 200, 200), thickness=2)
                    
                    # Dibujar Checkpoints
                    for pt, nombre in zip([p1, p2], ["Tag 1", "Tag 2"]):
                        cv2.circle(dashboard, pt, 8, (0, 255, 0), -1)
                        cv2.putText(dashboard, nombre, (pt[0]+15, pt[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # 3. DIBUJAR AL ROBOT (Tag 0)
                if 0 in poses_3d:
                    rx, ry = poses_3d[0]
                    p_robot = metros_a_pixeles(rx, ry)
                    
                    # Guardar en la estela
                    estela_robot.append(p_robot)
                    
                    # Dibujar la estela (Rastro naranja)
                    if len(estela_robot) > 1:
                        for i in range(1, len(estela_robot)):
                            # Hace que la estela se difumine
                            grosor = int(np.interp(i, [0, len(estela_robot)], [1, 4]))
                            cv2.line(dashboard, estela_robot[i-1], estela_robot[i], (0, 100, 255), grosor)

                    # Dibujar ícono del robot
                    cv2.circle(dashboard, p_robot, 12, (0, 165, 255), -1)
                    cv2.circle(dashboard, p_robot, 16, (0, 165, 255), 1) # Anillo exterior
                    
                    # Imprimir telemetría del robot
                    cv2.putText(dashboard, f"PICKASSO X:{rx:.2f}m Y:{ry:.2f}m", 
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

                    # Dibujar línea dinámica de objetivo hacia Tag 1 (Ejemplo)
                    if 1 in poses_3d:
                        p1 = metros_a_pixeles(poses_3d[1][0], poses_3d[1][1])
                        cv2.line(dashboard, p_robot, p1, (0, 0, 255), 1, cv2.LINE_AA) # Línea láser roja
                        
                        dist = math.hypot(poses_3d[1][0] - rx, poses_3d[1][1] - ry)
                        cv2.putText(dashboard, f"Distancia a Tag 1: {dist:.2f}m", 
                                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Mostrar el Dashboard
            cv2.imshow("Telemetria Pickasso (Radar 2D)", dashboard)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[*] Cerrando telemetría...")
                break

        except Exception as e:
            print(f"[!] Error procesando telemetría: {e}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
