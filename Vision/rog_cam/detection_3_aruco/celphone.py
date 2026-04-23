import cv2
import numpy as np
import math

def main():
    # --- CONFIGURACIÓN ---
    marker_size = 0.063  # Tamaño real del ArUco en metros (6.3 cm)
    station_threshold = 0.10  # Distancia para "REACHED" en metros
    
    # Diccionario de significados
    aruco_meanings = {
        0: "Robot",
        1: "Estacion 1",
        2: "Estacion 2"
    }

    # --- INICIALIZACIÓN DE VISIÓN ---
    camera_matrix = None
    dist_coeffs = None
    
    # Configuración del detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # Puntos 3D del marcador
    half_size = marker_size / 2.0
    obj_points = np.array([
        [-half_size,  half_size, 0],
        [ half_size,  half_size, 0],
        [ half_size, -half_size, 0],
        [-half_size, -half_size, 0]
    ], dtype=np.float32)

    # --- CONFIGURACIÓN DE DROIDCAM ---
    droidcam_url = "http://10.48.97.233:4747/video" 
    
    print(f"[*] Conectando a DroidCam en: {droidcam_url}")
    cap = cv2.VideoCapture(droidcam_url)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not cap.isOpened():
        print(f"[!] ERROR: No se pudo abrir la conexión con DroidCam.")
        print("Verifica que la app esté abierta, la IP sea correcta y estés en el mismo Wi-Fi.")
        return

    print("[OK] Iniciando detección. Presiona la tecla 'q' en la ventana de video para salir.")

    # --- BUCLE PRINCIPAL ---
    while True:
        ret, cv_image_raw = cap.read()
        if not ret:
            print("[!] Fallo al capturar imagen de DroidCam.")
            break

        # Generar matriz aproximada en el primer frame
        if camera_matrix is None:
            h, w = cv_image_raw.shape[:2]
            focal_length = w * 0.9 # Ajuste fino para cámaras de celular
            camera_matrix = np.array([
                [focal_length, 0, w / 2],
                [0, focal_length, h / 2],
                [0, 0, 1]
            ], dtype=np.float32)
            dist_coeffs = np.zeros((4, 1))

        try:
            # Corregir distorsión
            cv_image = cv2.undistort(cv_image_raw, camera_matrix, dist_coeffs)
            
            # Detectar tags
            corners, ids, rejected = detector.detectMarkers(cv_image)
            current_poses = {}

            if ids is not None:
                # ==========================================
                # PRIMER PASO: Calcular Posición y Centros 2D
                # ==========================================
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    marker_corners = corners[i][0]

                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, marker_corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if success:
                        x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]
                        
                        # Calcular el centro exacto del ArUco en pixeles
                        cx = int(np.mean(marker_corners[:, 0]))
                        cy = int(np.mean(marker_corners[:, 1]))
                        
                        # Guardamos coordenadas 3D y el centro 2D
                        current_poses[marker_id] = (x, y, z, cx, cy)

                        # Dibujar contornos
                        cv2.aruco.drawDetectedMarkers(cv_image, corners)
                        cv2.drawFrameAxes(cv_image, camera_matrix, dist_coeffs, rvec, tvec, marker_size)
                        
                        # Imprimir el nombre del Tag encima de él
                        meaning = aruco_meanings.get(marker_id, f"Tag {marker_id}")
                        color_texto = (0, 165, 255) if marker_id == 0 else (0, 255, 0)
                        cv2.putText(cv_image, meaning, (cx - 40, cy - 40), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_texto, 2)

                # ==========================================
                # SEGUNDO PASO: Trazar Rutas y Calcular Distancias
                # ==========================================
                id_robot = 0
                
                if id_robot in current_poses:
                    rx, ry, rz, rcx, rcy = current_poses[id_robot]
                    
                    # Dibujar un punto central en el robot
                    cv2.circle(cv_image, (rcx, rcy), 5, (0, 165, 255), -1)
                    
                    for station_id in [1, 2]:
                        if station_id in current_poses:
                            sx, sy, sz, scx, scy = current_poses[station_id]
                            
                            # Diferencias en metros
                            dx = sx - rx
                            dy = sy - ry
                            distancia_total = math.hypot(dx, dy) # Distancia 2D en el plano
                            
                            # --- DIBUJAR LA RUTA (LÍNEA DINÁMICA) ---
                            cv2.line(cv_image, (rcx, rcy), (scx, scy), (255, 0, 255), 2, cv2.LINE_AA)
                            
                            # Dibujar un punto en el destino
                            cv2.circle(cv_image, (scx, scy), 5, (0, 255, 0), -1)
                            
                            # Encontrar el punto medio de la línea para poner la etiqueta de distancia
                            mid_x = int((rcx + scx) / 2)
                            mid_y = int((rcy + scy) / 2)
                            
                            # Fondo semitransparente para leer mejor el texto (opcional, pero se ve muy pro)
                            cv2.putText(cv_image, f"Dist: {distancia_total:.2f}m", (mid_x + 10, mid_y), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                            
                            # Mostrar dx y dy al lado de la estación objetivo
                            cv2.putText(cv_image, f"dx: {dx:.2f} m", (scx + 50, scy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                            cv2.putText(cv_image, f"dy: {dy:.2f} m", (scx + 50, scy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                            # --- LÓGICA DE ALCANCE (REACHED) ---
                            if distancia_total < station_threshold:
                                cv2.rectangle(cv_image, (scx - 60, scy - 90), (scx + 60, scy - 60), (0, 0, 255), -1)
                                cv2.putText(cv_image, "REACHED", (scx - 55, scy - 68), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow("Camara DroidCam - Pruebas Pickasso", cv_image)
            
            # Condición de salida: presionar la tecla 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[*] Saliendo del programa...")
                break

        except Exception as e:
            print(f"[!] Error procesando la imagen: {e}")

    # Limpieza final
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
