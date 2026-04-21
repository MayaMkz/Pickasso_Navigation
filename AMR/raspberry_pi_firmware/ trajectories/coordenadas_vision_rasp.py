import serial
import time
import math
import sys
import socket
import threading

# ==========================================
# 1. CONFIGURACIÓN DEL ROBOT Y WIFI
# ==========================================
KP_VISION = 40.0  
PUERTO_UDP = 5005
rpm_base = 40.0

# Variables globales donde guardaremos lo que dice la cámara
datos_camara = {"dx": 0.0, "dy": 0.0, "distancia": 999.0, "llegamos": False}

print("Iniciando Cerebro UDP y Médula Espinal (ESP32)...")
try:
    esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1) 
    esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1) 
    time.sleep(2)
    print("[OK] ESP32 conectadas.")
except Exception as e:
    print(f"Error USB: {e}"); exit()

def mandar_orden(placa, motor, direccion, rpm):
    placa.write(f"{motor},{direccion},{round(rpm, 1)}\n".encode('utf-8'))

def frenar_y_limpiar():
    mandar_orden(esp_1, "A", 0, 0); mandar_orden(esp_1, "B", 0, 0)
    mandar_orden(esp_2, "A", 0, 0); mandar_orden(esp_2, "B", 0, 0)
    time.sleep(0.1) 
    esp_1.reset_input_buffer(); esp_1.reset_output_buffer()
    esp_2.reset_input_buffer(); esp_2.reset_output_buffer()

# ==========================================
# 2. EL HILO ESPÍA (RECEPTOR UDP)
# ==========================================
def escuchar_wifi():
    """ Esta función corre en el fondo escuchando la antena WiFi todo el tiempo """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PUERTO_UDP)) # 0.0.0.0 significa "escucha en cualquier IP que tenga la Raspberry"
    
    while True:
        try:
            # Recibe el mensaje de la PC (ej. "1.23,0.05,1.25")
            data, addr = sock.recvfrom(1024) 
            mensaje = data.decode('utf-8').split(',')
            
            if len(mensaje) == 3:
                datos_camara["dx"] = float(mensaje[0])
                datos_camara["dy"] = float(mensaje[1])
                datos_camara["distancia"] = float(mensaje[2])
        except Exception:
            pass

# Iniciamos el hilo espía antes de mover los motores
hilo_udp = threading.Thread(target=escuchar_wifi, daemon=True)
hilo_udp.start()
print(f"[OK] Radio WiFi escuchando en el puerto {PUERTO_UDP}...")

# ==========================================
# 3. BUCLE DE CONTROL DE NAVEGACIÓN
# ==========================================
try:
    while True:
        # Extraemos los datos más recientes del hilo espía
        distancia = datos_camara["distancia"]
        dx = datos_camara["dx"]
        
        # Si la cámara dejó de ver al robot, la distancia se queda en 999
        if distancia == 999.0:
            sys.stdout.write("\r[!] Esperando a que la cámara vea al Robot y la Estación...   ")
            sys.stdout.flush()
            time.sleep(0.5)
            continue

        # 1. ¿Ya llegamos?
        if distancia < 0.10:
            if not datos_camara["llegamos"]:
                print("\n[★★★] ¡META ALCANZADA! Frenando robot.")
                frenar_y_limpiar()
                datos_camara["llegamos"] = True
            time.sleep(0.1)
            continue
            
        datos_camara["llegamos"] = False
        
        # 2. Cálculo de corrección
        # dx te dice qué tan chueco vas. Lo multiplicamos por la Fuerza (KP)
        ajuste = dx * KP_VISION 
        
        rpm_izq = max(10, min(rpm_base - ajuste, 100))
        rpm_der = max(10, min(rpm_base + ajuste, 100))
        
        sys.stdout.write(f"\r[->] Distancia: {distancia:.2f}m | Error X: {dx:.3f} | RPM: {rpm_izq:.0f} / {rpm_der:.0f}   ")
        sys.stdout.flush()
        
        # 3. Mover motores
        mandar_orden(esp_1, "A", 1, rpm_izq); mandar_orden(esp_1, "B", 2, rpm_izq) 
        mandar_orden(esp_2, "A", 1, rpm_der); mandar_orden(esp_2, "B", 2, rpm_der) 
        
        time.sleep(0.05) # Pausa técnica para dejar reaccionar al hardware

except KeyboardInterrupt:
    print("\nParo de Emergencia por Teclado.")
finally:
    frenar_y_limpiar()
    esp_1.close(); esp_2.close()
