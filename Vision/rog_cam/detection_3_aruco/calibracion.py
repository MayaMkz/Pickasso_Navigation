import cv2
import numpy as np

# --- CONFIGURACIÓN DE TU CÁMARA DROIDCAM ---
DROIDCAM_URL = "http://192.168.137.24:4747/video"

# --- CONFIGURACIÓN DEL TABLERO CHARUCO ---
SQUARES_X = 4
SQUARES_Y = 6
SQUARE_LENGTH_M = 0.14  # 14 cm
MARKER_LENGTH_M = 0.10  # 10 cm

# Diccionario confirmado: 4x4
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# 1. Crear el tablero con la sintaxis de OpenCV 4.7+
board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LENGTH_M, MARKER_LENGTH_M, aruco_dict)

# 2. Crear el Detector ChArUco moderno
charuco_detector = cv2.aruco.CharucoDetector(board)

# Listas para almacenar datos
all_charuco_corners = []
all_charuco_ids = []

print("[*] Conectando a DroidCam en tu celular...")
cap = cv2.VideoCapture(DROIDCAM_URL)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

fotos_tomadas = 0
print("\n" + "="*50)
print("  CALIBRADOR CHARUCO (API OpenCV Moderna)")
print("="*50)
print("1. Muestra el tablero a la cámara.")
print("2. Presiona ESPACIO para tomar foto (necesitas mínimo 10, ideal 20).")
print("3. Presiona 'c' para Calcular y Guardar.")
print("4. Presiona 'q' para Salir sin guardar.")

while True:
    ret, frame = cap.read()
    if not ret: 
        cv2.waitKey(100)
        continue

    frame_copia = frame.copy()
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 3. Detección en un solo paso con la nueva API
    charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(gris)

    # Dibujar resultados si se encuentran suficientes esquinas
    if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 3:
        # Dibujar los contornos de los marcadores individuales
        if marker_corners is not None and len(marker_corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame_copia, marker_corners, marker_ids)
            
        # Dibujar los puntos y líneas del tablero ChArUco
        cv2.aruco.drawDetectedCornersCharuco(frame_copia, charuco_corners, charuco_ids, (0, 255, 0))

    # HUD
    listo_para_foto = (charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 3)
    color_texto = (0, 255, 0) if listo_para_foto else (0, 0, 255)
    cv2.putText(frame_copia, f"Fotos guardadas: {fotos_tomadas}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color_texto, 2)
    cv2.imshow('Calibracion ChArUco - Celular Cenital', frame_copia)

    tecla = cv2.waitKey(1) & 0xFF
    
    if tecla == ord(' '): 
        if listo_para_foto:
            all_charuco_corners.append(charuco_corners)
            all_charuco_ids.append(charuco_ids)
            fotos_tomadas += 1
            print(f"[+] Foto {fotos_tomadas} guardada. (Esquinas detectadas: {len(charuco_corners)})")
        else:
            print("[-] No se detectan suficientes esquinas. Ajusta la cámara o iluminación.")
            
    elif tecla == ord('c'):
        if fotos_tomadas >= 10:
            print("\n[!] Emparejando puntos 3D-2D y calculando distorsión...")
            
            # 4. Preparar puntos para la calibración (Nueva metodología de OpenCV)
            objpoints = []
            imgpoints = []
            
            for corners, ids in zip(all_charuco_corners, all_charuco_ids):
                # Extraer los puntos reales vs los puntos de la imagen
                objp, imgp = board.matchImagePoints(corners, ids)
                if objp is not None and imgp is not None and len(objp) > 3:
                    objpoints.append(objp)
                    imgpoints.append(imgp)

            if len(objpoints) > 0:
                # Calibración estándar de cámara
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                    objpoints, imgpoints, gris.shape[::-1], None, None)
                
                nombre_archivo = "parametros_droidcam.yaml"
                fs = cv2.FileStorage(nombre_archivo, cv2.FILE_STORAGE_WRITE)
                fs.write("camera_matrix", mtx)
                fs.write("dist_coeffs", dist)
                fs.release()
                
                print("\n" + "★"*50)
                print("¡CALIBRACIÓN EXITOSA!")
                print(f"Archivo generado: {nombre_archivo}")
                print("★"*50 + "\n")
                break
            else:
                print("\n[X] Error interno al emparejar puntos. Intenta tomar las fotos con mejor iluminación.")
        else:
            print(f"\n[X] Necesitas mínimo 10 fotos. Tienes {fotos_tomadas}.")
            
    elif tecla == ord('q'):
        print("\n[!] Saliendo...")
        break

cap.release()
cv2.destroyAllWindows()
