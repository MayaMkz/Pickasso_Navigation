"""
cel10.py — Pickasso AGV Holonómico | PC Vision + Pure Pursuit
=============================================================
[CORRECCIÓN CRÍTICA] YAW BLOQUEADO (Mirada al frente fija)
El carro holonómico NO debe cambiar su ángulo objetivo por segmento.
Al presionar 'm', se calcula el ángulo perfecto usando la línea de 
la pista, se bloquea y se mantiene constante toda la misión.
"""

import cv2
import numpy as np
import math
import threading
import socket
import time
from collections import deque

class CamaraIP_UltraRapida:
    def __init__(self, url):
        self.stream = cv2.VideoCapture(url)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.stream.isOpened():
            raise Exception("No se pudo conectar a DroidCam.")
        self.stopped      = False
        self.frame_fresco = None

    def start(self):
        threading.Thread(target=self._update, daemon=True).start()
        while self.frame_fresco is None and not self.stopped:
            time.sleep(0.05)
        print("[OK] Video 1280x720 en vivo.")
        return self

    def _update(self):
        while not self.stopped:
            if self.stream.grab():
                _, img = self.stream.retrieve()
                self.frame_fresco = cv2.resize(img, (1280, 720))
            else:
                self.stop()

    def read(self):
        f = self.frame_fresco
        self.frame_fresco = None
        return f

    def stop(self):
        self.stopped = True
        self.stream.release()

def proyectar(puntos_3d, K, D):
    pts = np.array(puntos_3d, dtype=np.float32).reshape(-1, 1, 3)
    pts2d, _ = cv2.projectPoints(pts, np.zeros((3,1)), np.zeros((3,1)), K, D)
    return [(int(p[0][0]), int(p[0][1])) for p in pts2d]

def main():
    # ══════════════════════════════════════════════════════════════
    #  RED Y CONFIGURACIÓN
    # ══════════════════════════════════════════════════════════════
    DROIDCAM_URL = "http://192.168.137.24:4747/video"
    IP_RASPBERRY = "192.168.137.240"
    PUERTO_UDP   = 5005

    TAG_TIMEOUT_S = 1.25

    ROBOT_TAGS = {
        5  : ( -0.250,  0.140),
        4  : (  0.250,  0.140),
        3  : ( -0.250, -0.140),
        0  : (  0.250, -0.140),
    }
    PRIORIDAD_TAGS = [0, 3, 4, 5]
    tag_robot_activo = None

    OFFSET_MESA_X = 0.495
    OFFSET_MESA_Y = 0.480
    TOLERANCIA_LLEGADA_M = 0.05

    LOOKAHEAD_FIJO   = 0.20
    V_LINEAL         = 0.10
    V_APROX_FACTOR   = 0.60
    MAX_VY           = 0.25

    # ── AJUSTE GLOBAL DE HEADING ──
    # Si al arrancar notas que todo el circuito está rotado unos grados
    # respecto a tu mesa física, ajusta aquí (+/- grados)
    OFFSET_HEADING_GLOBAL_DEG = 0.0
    K_HEADING = 0.8
    OMEGA_HEADING_MAX = 0.5
    
    yaw_bloqueado = 0.0 

    print(f"[*] Conectando a DroidCam...")
    try:
        cap = CamaraIP_UltraRapida(DROIDCAM_URL).start()
    except Exception as e:
        print(f"[ERROR] {e}"); return

    sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    fs = cv2.FileStorage("parametros_droidcam.yaml", cv2.FILE_STORAGE_READ)
    if fs.isOpened():
        K = fs.getNode("camera_matrix").mat()
        D = fs.getNode("dist_coeffs").mat()
        fs.release()
    else:
        f_est = 640 * 0.9
        K = np.array([[f_est,0,320],[0,f_est,240],[0,0,1]], dtype=np.float32)
        D = np.zeros((4,1))

    aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_params.cornerRefinementMethod    = cv2.aruco.CORNER_REFINE_SUBPIX
    aruco_params.adaptiveThreshWinSizeStep = 10
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # Marcador a tamaño físico real
    hs         = 0.087 / 2.0
    obj_points = np.array([[-hs, hs,0],[hs, hs,0],[hs,-hs,0],[-hs,-hs,0]], dtype=np.float32)

    memoria_tags        = {}
    estela_robot        = deque(maxlen=80)
    ruta_pts_global     = []
    mision_activa       = False
    estado_mision       = 0
    objetivo_actual     = None
    nombre_objetivo     = ""
    posicion_home       = None

    MM_SIZE   = 160
    MM_ESCALA = 80

    def mundo_a_mm(mx, my):
        return (int(MM_SIZE/2 + mx*MM_ESCALA), int(MM_SIZE/2 - my*MM_ESCALA))

    while True:
        frame_raw = cap.read()
        if frame_raw is None:
            time.sleep(0.005); continue

        # Evitamos doble distorsión usando la imagen cruda
        cam_view = frame_raw.copy()
        
        mm = np.zeros((MM_SIZE, MM_SIZE, 3), np.uint8)
        for i in range(0, MM_SIZE, 20):
            cv2.line(mm, (i,0),(i,MM_SIZE),(30,30,30),1)
            cv2.line(mm, (0,i),(MM_SIZE,i),(30,30,30),1)

        gray = cv2.cvtColor(cam_view, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        ahora = time.time()

        if ids is not None:
            for i, mid_arr in enumerate(ids):
                mid = int(mid_arr[0])
                ok, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    cx = int(np.mean(corners[i][0][:,0]))
                    cy = int(np.mean(corners[i][0][:,1]))
                    memoria_tags[mid] = {
                        'm_x': tvec[0][0], 'm_y': tvec[1][0], 'm_z': tvec[2][0],
                        'c_x': cx, 'c_y': cy, 'rvec': rvec, 'tvec': tvec, 'ts': ahora}
            cv2.aruco.drawDetectedMarkers(cam_view, corners)

        IDS_ROBOT = set(ROBOT_TAGS.keys())
        for mid, td in memoria_tags.items():
            age   = ahora - td['ts']
            if mid in IDS_ROBOT:
                color = (0, 165, 255) if mid == tag_robot_activo else (180, 100, 0)
                label = f"Robot[{mid}]{'*' if mid==tag_robot_activo else ''}"
            else:
                color = (0, 255, 0)
                label = f"Tag {mid}"
            if age > TAG_TIMEOUT_S: label += f" ??{age:.1f}s"; color = (0, 0, 220)
            cv2.putText(cam_view, label, (td['c_x']-40, td['c_y']-42), cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 2)

        def seleccionar_tag_robot():
            if (tag_robot_activo is not None and tag_robot_activo in memoria_tags and (ahora - memoria_tags[tag_robot_activo]['ts']) < TAG_TIMEOUT_S):
                return tag_robot_activo
            for tid in PRIORIDAD_TAGS:
                if (tid in memoria_tags and (ahora - memoria_tags[tid]['ts']) < TAG_TIMEOUT_S):
                    return tid
            return None

        nuevo_activo = seleccionar_tag_robot()
        if nuevo_activo != tag_robot_activo:
            if nuevo_activo is not None: print(f"\n[TAG] Cambiando tag de robot: {tag_robot_activo} → {nuevo_activo}")
            tag_robot_activo = nuevo_activo

        if 1 in memoria_tags and 2 in memoria_tags:
            t1 = memoria_tags[1]; t2 = memoria_tags[2]
            t1x,t1y,t1z = t1['m_x'],t1['m_y'],t1['m_z']
            t2x,t2y,t2z = t2['m_x'],t2['m_y'],t2['m_z']
            cz  = (t1z+t2z)/2.0
            cmx = (t1x+t2x)/2.0; cmy = (t1y+t2y)/2.0

            mesa_pts = [(t1x,t1y,t1z),(t2x,t1y,cz),(t2x,t2y,t2z),(t1x,t2y,cz)]

            def expandir(px, py, pz):
                nx = px + OFFSET_MESA_X if px > cmx else px - OFFSET_MESA_X
                ny = py + OFFSET_MESA_Y if py > cmy else py - OFFSET_MESA_Y
                return (nx, ny, pz)

            ruta_pts_global = [expandir(*p) for p in mesa_pts]

            for pts3, col3 in [(mesa_pts,(255,150,50)),(ruta_pts_global,(0,255,0))]:
                p2d = proyectar(pts3, K, D)
                cv2.polylines(cam_view, [np.array(p2d,np.int32).reshape(-1,1,2)], True, col3, 2)
            for pts3, col3 in [(mesa_pts,(255,150,50)),(ruta_pts_global,(0,255,0))]:
                cv2.polylines(mm, [np.array([mundo_a_mm(p[0],p[1]) for p in pts3],np.int32).reshape(-1,1,2)], True, col3, 1)

        if tag_robot_activo is not None:
            tag  = memoria_tags[tag_robot_activo]
            off_x, off_y = ROBOT_TAGS[tag_robot_activo]
            rx_t = tag['m_x']; ry_t = tag['m_y']
            rvec = tag['rvec']; tvec = tag['tvec']
            R, _ = cv2.Rodrigues(rvec)

            off_cam = R @ np.array([[off_x],[off_y],[0.0]], dtype=np.float64)
            cx_m    = rx_t + off_cam[0][0]
            cy_m    = ry_t + off_cam[1][0]

            vec_nariz = R @ np.array([[0.0],[-0.15],[0.0]])
            robot_yaw = math.atan2(vec_nariz[1][0], vec_nariz[0][0])

            p_cen = proyectar([(cx_m, cy_m, tag['m_z'])], K, D)[0]
            p_nar = proyectar([(cx_m+vec_nariz[0][0], cy_m+vec_nariz[1][0], tag['m_z'])], K, D)[0]
            cv2.arrowedLine(cam_view, p_cen, p_nar, (0,0,255), 3, tipLength=0.4)
            cv2.circle(cam_view, p_cen, 5, (255,255,0), -1)

            if abs(off_x) + abs(off_y) > 0.01:
                p_tag = proyectar([(rx_t, ry_t, tag['m_z'])], K, D)[0]
                cv2.circle(cam_view, p_tag, 4, (0, 165, 255), -1)
                cv2.line(cam_view, p_tag, p_cen, (100, 100, 255), 1)

            p_mm = mundo_a_mm(cx_m, cy_m)
            if not estela_robot or estela_robot[-1] != p_mm: estela_robot.append(p_mm)
            for i in range(1, len(estela_robot)):
                cv2.line(mm, estela_robot[i-1], estela_robot[i], (0,120,255), int(np.interp(i,[0,len(estela_robot)],[1,3])))

            if mision_activa and len(ruta_pts_global)==4 and posicion_home is not None:

                if   estado_mision==1: origen=posicion_home;      objetivo_actual=ruta_pts_global[0]; nombre_objetivo="1.Est.1"
                elif estado_mision==2: origen=ruta_pts_global[0]; objetivo_actual=ruta_pts_global[1]; nombre_objetivo="2.Seg.2"
                elif estado_mision==3: origen=ruta_pts_global[1]; objetivo_actual=ruta_pts_global[2]; nombre_objetivo="3.Est.2"
                elif estado_mision==4: origen=ruta_pts_global[2]; objetivo_actual=ruta_pts_global[3]; nombre_objetivo="4.Seg.4"
                else: objetivo_actual = None

                if objetivo_actual:
                    tx_fin,ty_fin,tz_fin = objetivo_actual
                    ox,oy,_              = origen
                    dist_real = math.hypot(tx_fin-cx_m, ty_fin-cy_m)

                    # ── [CORRECCIÓN YAW] Usar el ángulo bloqueado para orientar el carro siempre al frente ──
                    error_heading = robot_yaw - yaw_bloqueado
                    error_heading = (error_heading + math.pi) % (2 * math.pi) - math.pi

                    omega_heading = -K_HEADING * error_heading
                    omega_heading = max(-OMEGA_HEADING_MAX, min(OMEGA_HEADING_MAX, omega_heading))

                    # Dibujar línea cyan que indica hacia donde intenta mirar el robot
                    largo_flecha = 0.18
                    p_yaw_obj = proyectar(
                        [(cx_m + largo_flecha * math.cos(yaw_bloqueado),
                          cy_m + largo_flecha * math.sin(yaw_bloqueado),
                          tag['m_z'])], K, D)[0]
                    cv2.arrowedLine(cam_view, p_cen, p_yaw_obj, (255, 255, 0), 2, tipLength=0.3)
                    
                    if dist_real <= TOLERANCIA_LLEGADA_M:
                        vx_cmd = 0.0; vy_cmd = 0.0; omega_heading = 0.0
                        estado_mision += 1
                        print(f"[✓] Waypoint capturado → estado {estado_mision}")
                        if estado_mision > 4: mision_activa = False

                    else:
                        L_linea = math.hypot(tx_fin-ox, ty_fin-oy)
                        if L_linea < 1e-4: L_linea = 1e-4
                        ux, uy  = (tx_fin-ox)/L_linea, (ty_fin-oy)/L_linea
                        vx_proj, vy_proj = cx_m-ox, cy_m-oy
                        proj    = vx_proj*ux + vy_proj*uy
                        d_virt  = max(0.0, min(proj+LOOKAHEAD_FIJO, L_linea))
                        tx_zan  = ox + d_virt*ux
                        ty_zan  = oy + d_virt*uy

                        dx_w  = tx_zan - cx_m
                        dy_w  = ty_zan - cy_m
                        d_zan = max(math.hypot(dx_w, dy_w), 1e-4)

                        cos_y  = math.cos(robot_yaw)
                        sin_y  = math.sin(robot_yaw)
                        vx_dir =  (dx_w/d_zan)*cos_y + (dy_w/d_zan)*sin_y
                        vy_dir = -(dx_w/d_zan)*sin_y + (dy_w/d_zan)*cos_y

                        v = V_LINEAL * (V_APROX_FACTOR if dist_real < 0.25 else 1.0)
                        vx_cmd = v * vx_dir
                        vy_cmd = max(-MAX_VY, min(MAX_VY, v * vy_dir))

                        p_zan = proyectar([(tx_zan,ty_zan,tz_fin)],K,D)[0]
                        cv2.circle(cam_view, p_zan, 8, (0,200,255), -1)
                        cv2.line(cam_view, p_cen, p_zan, (0,200,255), 2)

                    p_obj = proyectar([(tx_fin,ty_fin,tz_fin)],K,D)[0]
                    cv2.circle(cam_view, p_obj, 10, (0,255,100), 2)
                    
                    paquete = f"{vx_cmd:.4f},{vy_cmd:.4f},{dist_real:.4f},{estado_mision},{omega_heading:.4f}"
                    sock_udp.sendto(paquete.encode(), (IP_RASPBERRY, PUERTO_UDP))

                    cv2.putText(cam_view, f"Target : {nombre_objetivo}", (8,96), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,0,255), 2)
                    cv2.putText(cam_view, f"vx={vx_cmd:+.3f} vy={vy_cmd:+.3f}", (8,122), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,255), 2)
                    cv2.putText(cam_view, f"Hdg Err: {math.degrees(error_heading):+.1f} deg", (8,148), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200,200,200), 2)

        estado_txt = f"MISION | {nombre_objetivo}" if mision_activa else "ESPERANDO — 's' iniciar  'r' reset  'q' salir"
        cv2.putText(cam_view, estado_txt, (8,36), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,0) if mision_activa else (0,80,255), 2)
        cv2.rectangle(mm, (0,0), (MM_SIZE-1,MM_SIZE-1), (200,200,200), 1)
        cam_view[8:8+MM_SIZE, 480:640] = mm

        cv2.imshow("Pickasso Holonómico — Dashboard", cam_view)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'): break
        elif k == ord('r'):
            memoria_tags.clear(); estela_robot.clear()
            mision_activa=False; estado_mision=0; posicion_home=None
        elif k == ord('m') and not mision_activa:
            if len(ruta_pts_global)==4 and tag_robot_activo is not None:
                tag  = memoria_tags[tag_robot_activo]
                off_x, off_y = ROBOT_TAGS[tag_robot_activo]
                R_s, _ = cv2.Rodrigues(tag['rvec'])
                off    = R_s @ np.array([[off_x],[off_y],[0.0]])
                posicion_home = (tag['m_x']+off[0][0], tag['m_y']+off[1][0], tag['m_z'])
                
                # ── ALINEACIÓN PERFECTA CON LA LÍNEA VERDE ──
                # Ignoramos si el carro está chueco físicamente.
                # Calculamos el ángulo exacto de la línea de la pista (Punto 3 al 0)
                dy_pista = ruta_pts_global[0][1] - ruta_pts_global[3][1]
                dx_pista = ruta_pts_global[0][0] - ruta_pts_global[3][0]
                
                # Fijamos la vista al frente para toda la misión
                yaw_bloqueado = math.atan2(dy_pista, dx_pista) + math.radians(OFFSET_HEADING_GLOBAL_DEG)
                
                mision_activa  = True
                estado_mision  = 1
                print(f"[►] Misión iniciada. Tag: {tag_robot_activo} | Yaw Bloqueado: {math.degrees(yaw_bloqueado):.1f}°")

    cap.stop(); cv2.destroyAllWindows(); sock_udp.close()

if __name__ == '__main__':
    main()
