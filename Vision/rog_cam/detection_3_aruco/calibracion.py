import cv2
import numpy as np

# --- CONFIGURACIÓN DE TU CÁMARA Y TABLERO ---
DROIDCAM_URL = "http://192.168.137.24:4747/video"

# Número de INTERSECCIONES internas (No de cuadros). 
# Si no te lo detecta, prueba cambiándolo a (8, 6)
CHECKERBOARD = (9, 7) 
TAMANO_CUADRO_M = 0.013  # 13 mm pasados a metros

criterio = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Preparar puntos 3D del tablero real con tu medida exacta
puntos_3d_obj = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
puntos_3d_obj[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * TAMANO_CUADRO_M

puntos_3d = [] # Puntos en el mundo real 
puntos_2d = [] # Puntos en la imagen

print("[*] Conectando a la cámara...")
cap = cv2.VideoCapture(DROIDCAM_URL)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

fotos_tomadas = 0
print("\n" + "="*50)
print("  INSTRUCCIONES DE CALIBRACIÓN")
print("="*50)
print("1. Muestra el tablero a la cámara bien estirado (ponlo sobre una tabla plana).")
print("2. Presiona ESPACIO para tomar una foto.")
print("   -> Tip: Toma fotos moviendo el tablero a las esquinas, al centro, y con ligeras inclinaciones.")
print("3. Cuando tengas unas 15-20 fotos, presiona 'c' para Calcular.")
print("4. Presiona 'q' para Salir sin guardar.")

while True:
    ret, frame = cap.read()
    if not ret: continue

    frame_copia = frame.copy()
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Buscar el tablero de ajedrez
    encontrado, esquinas = cv2.findChessboardCorners(gris, CHECKERBOARD, 
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if encontrado:
        esquinas_refinadas = cv2.cornerSubPix(gris, esquinas, (11, 11), (-1, -1), criterio)
        cv2.drawChessboardCorners(frame_copia, CHECKERBOARD, esquinas_refinadas, encontrado)

    # HUD en pantalla
    color_texto = (0, 255, 0) if encontrado else (0, 0, 255)
    cv2.putText(frame_copia, f"Fotos validas: {fotos_tomadas}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color_texto, 2)
    cv2.imshow('Calibrador de Lente Pickasso', frame_copia)

    tecla = cv2.waitKey(1) & 0xFF
    
    if tecla == ord(' '): # Capturar con barra espaciadora
        if encontrado:
            puntos_3d.append(puntos_3d_obj)
            puntos_2d.append(esquinas_refinadas)
            fotos_tomadas += 1
            print(f"[+] Foto {fotos_tomadas} capturada con éxito.")
        else:
            print("[-] No se ven todas las intersecciones. Acerca el tablero o revisa la iluminación.")
            
    elif tecla == ord('c'): # Calcular matriz
        if fotos_tomadas >= 10:
            print("\n[!] Calculando distorsión de la lente... (Espera unos segundos)")
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(puntos_3d, puntos_2d, gris.shape[::-1], None, None)
            
            print("\n" + "★"*50)
            print("¡CALIBRACIÓN EXITOSA! COPIA ESTO EN TU CÓDIGO MAESTRO:")
            print("★"*50)
            print("camera_matrix = np.array(")
            print(np.array2string(mtx, separator=', '))
            print(", dtype=np.float32)")
            print("\ndist_coeffs = np.array(")
            print(np.array2string(dist, separator=', '))
            print(", dtype=np.float32)")
            print("★"*50 + "\n")
            break
        else:
            print(f"[X] Muy pocas fotos ({fotos_tomadas}). Mínimo necesitas 10 buenas.")
            
    elif tecla == ord('q'):
        print("\n[!] Saliendo sin calibrar.")
        break

cap.release()
cv2.destroyAllWindows()
