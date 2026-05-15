import cv2
from ultralytics import YOLO

# --- CONFIGURACIÓN ---
# Cargar el modelo entrenado (Asegúrate de que best.pt esté en la misma carpeta)
modelo = YOLO('C:\\Users\\Jerry\\OneDrive\\Documentos\\ProyectoVision_Ciber\\envciber\\visionYolo\\best.pt')

# Iniciar la cámara
cap = cv2.VideoCapture(2)

# Opcional: Forzar una buena resolución para la ROG EYE S
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print("Iniciando cámara... Presiona la tecla 'q' en la ventana para salir.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error al leer la cámara.")
        break

    # --- PASO 1: HACER LA INFERENCIA ---
    # conf=0.6 significa que solo mostrará detecciones con más del 60% de seguridad
    resultados = modelo(frame, conf=0.8, verbose=False)

    # --- PASO 2: EXTRAER DATOS Y DIBUJAR ---
    # Iteramos sobre cada objeto detectado en el frame
    for r in resultados:
        cajas = r.boxes
        
        for caja in cajas:
            # 1. Obtener coordenadas del Bounding Box (x_min, y_min, x_max, y_max)
            x1, y1, x2, y2 = caja.xyxy[0].int().tolist()
            
            # 2. Obtener el nivel de confianza y el ID de la clase
            confianza = float(caja.conf[0])
            clase_id = int(caja.cls[0])
            
            # 3. Obtener el nombre del color (ej: "cubo_rojo") según tu entrenamiento
            nombre_clase = modelo.names[clase_id]
            
            # --- PREPARACIÓN PARA PICK & PLACE (Calcular el centro) ---
            centro_x = int((x1 + x2) / 2)
            centro_y = int((y1 + y2) / 2)
            
            # --- DIBUJO SOBRE LA IMAGEN ---
            # Dibujar el Bounding Box (Cuadro delimitador)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            
            # Dibujar un punto central (Esto es lo que le enviarás al robot después)
            cv2.circle(frame, (centro_x, centro_y), 4, (0, 0, 255), -1)
            
            # Crear la etiqueta con el nombre y porcentaje de seguridad
            etiqueta = f"{nombre_clase} {confianza*100:.1f}%"
            
            # Poner fondo al texto para que sea legible
            (w, h), _ = cv2.getTextSize(etiqueta, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - 25), (x1 + w, y1), (0, 255, 255), -1)
            
            # Escribir el texto
            cv2.putText(frame, etiqueta, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    # Mostrar la imagen en vivo
    cv2.imshow("YOLOv8 - Deteccion de Cubos", frame)

    # Condición de salida: Presionar la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar los recursos
cap.release()
cv2.destroyAllWindows()