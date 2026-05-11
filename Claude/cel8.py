"""
cel8.py — Pickasso AGV Holonómico | PC Vision + Pure Pursuit
=============================================================
CAMBIOS respecto a cel7.py
---------------------------
[1] ROBOT HOLONÓMICO — el carro ya no gira en las esquinas.
    Se elimina todo lo relacionado con omega (Stanley heading/cross).

[2] PURE PURSUIT → (vx, vy) en frame del robot
    El algoritmo calcula el vector (dx, dy) al punto lookahead en
    coordenadas del mundo, luego lo rota al frame del robot usando
    robot_yaw. Así el Raspberry sabe cuánto ir adelante y cuánto lateral.

[3] PAQUETE UDP NUEVO: "vx,vy,dist,estado_mision"
    vx  → velocidad en el eje frontal del robot (m/s)
    vy  → velocidad lateral del robot (m/s, + = derecha)
    dist → distancia al waypoint actual (m)
    estado_mision → 1-4 (la PC lo incrementa al capturar waypoint)

[4] SIMPLIFICACIÓN — se eliminan K_STANLEY_HEADING, K_STANLEY_CROSS,
    OMEGA_MAX, V_MIN_FACTOR, SENTIDO_GIRO_GLOBAL.
    Ya no se necesitan porque no hay giro de robot.

PARÁMETROS QUE DEBES TOCAR — ver sección [TUNING] abajo.
"""

import cv2
import numpy as np
import math
import threading
import socket
import time
from collections import deque


# ─────────────────────────────────────────────────────────────────────
# CÁMARA TURBO (grab/retrieve sin lag) — sin cambios
# ─────────────────────────────────────────────────────────────────────
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
        print("[OK] Video 640×480 en vivo.")
        return self

    def _update(self):
        while not self.stopped:
            if self.stream.grab():
                _, img = self.stream.retrieve()
                self.frame_fresco = cv2.resize(img, (640, 480))
            else:
                self.stop()

    def read(self):
        f = self.frame_fresco
        self.frame_fresco = None
        return f

    def stop(self):
        self.stopped = True
        self.stream.release()


# ─────────────────────────────────────────────────────────────────────
# PROYECCIÓN 3D → 2D (con perspectiva real de la cámara)
# ─────────────────────────────────────────────────────────────────────
def proyectar(puntos_3d, K, D):
    pts = np.array(puntos_3d, dtype=np.float32).reshape(-1, 1, 3)
    pts2d, _ = cv2.projectPoints(pts, np.zeros((3,1)), np.zeros((3,1)), K, D)
    return [(int(p[0][0]), int(p[0][1])) for p in pts2d]


# ─────────────────────────────────────────────────────────────────────
# PROGRAMA PRINCIPAL
# ─────────────────────────────────────────────────────────────────────
def main():

    # ══════════════════════════════════════════════════════════════
    #  RED
    # ══════════════════════════════════════════════════════════════
    DROIDCAM_URL = "http://192.168.137.24:4747/video"
    IP_RASPBERRY = "192.168.137.240"
    PUERTO_UDP   = 5005

    # ══════════════════════════════════════════════════════════════
    #  [TUNING A] GEOMETRÍA DEL ROBOT  ← medir con cinta
    # ══════════════════════════════════════════════════════════════
    # OFFSET_X_TAG : desplazamiento lateral del tag desde el centroide.
    #   + = tag está a la DERECHA del centroide del carro.
    OFFSET_X_TAG = 0.00   # m

    # OFFSET_Y_TAG : desplazamiento longitudinal.
    #   + = tag está DETRÁS del centroide (en frame cenital, nariz = -Y).
    OFFSET_Y_TAG = 0.00   # m  ← MEDIR

    # ══════════════════════════════════════════════════════════════
    #  [TUNING B] GEOMETRÍA DE LA RUTA
    # ══════════════════════════════════════════════════════════════
    # OFFSET_MESA_M : expansión del rectángulo virtual.
    #   Debe ser ≥ ancho_carro/2 + 0.10 m para que las llantas no rocen.
    OFFSET_MESA_M = 0.45   # m

    # TOLERANCIA_LLEGADA_M : radio para capturar un waypoint.
    #   Con Mecanum puedes reducir un poco porque el carro entra lateral.
    TOLERANCIA_LLEGADA_M = 0.05   # m

    # ══════════════════════════════════════════════════════════════
    #  [TUNING C] CONTROLADOR DE VELOCIDAD
    # ══════════════════════════════════════════════════════════════
    # LOOKAHEAD_FIJO : distancia de la zanahoria por delante del carro (m).
    #   Grande → curvas suaves, puede cortar esquinas.
    #   Pequeño → más preciso, puede oscilar.
    #   Rango útil: 0.10 – 0.30 m
    LOOKAHEAD_FIJO = 0.20   # m

    # V_LINEAL : velocidad de crucero total del vector (m/s).
    #   Sube de a 0.02 una vez que el control sea estable.
    V_LINEAL = 0.15   # m/s

    # V_APROX_FACTOR : fracción de V_LINEAL al entrar en zona de captura.
    #   El carro frena cuando dist < 0.25 m del waypoint.
    V_APROX_FACTOR = 0.60   # sin unidades

    # MAX_VY : techo de velocidad lateral (m/s).
    #   Limita maniobras laterales bruscas causadas por ruido de visión.
    MAX_VY = 0.25   # m/s

    # ══════════════════════════════════════════════════════════════
    #  INICIALIZACIÓN
    # ══════════════════════════════════════════════════════════════
    print(f"[*] Conectando a DroidCam...")
    try:
        cap = CamaraIP_UltraRapida(DROIDCAM_URL).start()
    except Exception as e:
        print(f"[ERROR] {e}"); return

    sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[*] UDP → {IP_RASPBERRY}:{PUERTO_UDP}")

    # ── Calibración ────────────────────────────────────────────
    fs = cv2.FileStorage("parametros_droidcam.yaml", cv2.FILE_STORAGE_READ)
    if fs.isOpened():
        K = fs.getNode("camera_matrix").mat()
        D = fs.getNode("dist_coeffs").mat()
        fs.release()
        print("[OK] Calibración cargada.")
    else:
        print("[WARN] Sin calibración — usando focal estimada.")
        f_est = 640 * 0.9
        K = np.array([[f_est,0,320],[0,f_est,240],[0,0,1]], dtype=np.float32)
        D = np.zeros((4,1))

    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, K, (640,480), cv2.CV_32FC1)

    # ── ArUco ───────────────────────────────────────────────────
    aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_params.cornerRefinementMethod    = cv2.aruco.CORNER_REFINE_SUBPIX
    aruco_params.adaptiveThreshWinSizeStep = 10
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    hs         = 0.087 / 2.0
    obj_points = np.array([[-hs, hs,0],[hs, hs,0],[hs,-hs,0],[-hs,-hs,0]], dtype=np.float32)

    # ── Estado ──────────────────────────────────────────────────
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

    # ════════════════════════════════════════════════════════════
    # BUCLE PRINCIPAL
    # ════════════════════════════════════════════════════════════
    while True:
        frame_raw = cap.read()
        if frame_raw is None:
            time.sleep(0.005); continue

        cam_view = cv2.remap(frame_raw, map1, map2, cv2.INTER_LINEAR)

        mm = np.zeros((MM_SIZE, MM_SIZE, 3), np.uint8)
        for i in range(0, MM_SIZE, 20):
            cv2.line(mm, (i,0),(i,MM_SIZE),(30,30,30),1)
            cv2.line(mm, (0,i),(MM_SIZE,i),(30,30,30),1)

        # ── Detección ArUco ──────────────────────────────────
        gray = cv2.cvtColor(cam_view, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        ahora = time.time()

        if ids is not None:
            for i, mid_arr in enumerate(ids):
                mid = int(mid_arr[0])
                ok, rvec, tvec = cv2.solvePnP(
                    obj_points, corners[i][0], K, D,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    cx = int(np.mean(corners[i][0][:,0]))
                    cy = int(np.mean(corners[i][0][:,1]))
                    memoria_tags[mid] = {
                        'm_x': tvec[0][0], 'm_y': tvec[1][0], 'm_z': tvec[2][0],
                        'c_x': cx, 'c_y': cy, 'rvec': rvec, 'tvec': tvec, 'ts': ahora}
            cv2.aruco.drawDetectedMarkers(cam_view, corners)

        for mid, td in memoria_tags.items():
            age   = ahora - td['ts']
            color = (0,165,255) if mid==0 else (0,255,0)
            label = "Robot" if mid==0 else f"Tag {mid}"
            if age > 0.5: label += f" ??{age:.1f}s"; color = (0,0,220)
            cv2.putText(cam_view, label, (td['c_x']-40, td['c_y']-42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # ── Rectángulo de ruta ────────────────────────────────
        if 1 in memoria_tags and 2 in memoria_tags:
            t1 = memoria_tags[1]; t2 = memoria_tags[2]
            t1x,t1y,t1z = t1['m_x'],t1['m_y'],t1['m_z']
            t2x,t2y,t2z = t2['m_x'],t2['m_y'],t2['m_z']
            cz  = (t1z+t2z)/2.0
            cmx = (t1x+t2x)/2.0; cmy = (t1y+t2y)/2.0

            mesa_pts = [(t1x,t1y,t1z),(t2x,t1y,cz),(t2x,t2y,t2z),(t1x,t2y,cz)]

            def expandir(px,py,pz):
                nx = px+OFFSET_MESA_M if px>cmx else px-OFFSET_MESA_M
                ny = py+OFFSET_MESA_M if py>cmy else py-OFFSET_MESA_M
                return (nx,ny,pz)

            ruta_pts_global = [expandir(*p) for p in mesa_pts]

            # Dibujar con proyección real
            for pts3, col3 in [(mesa_pts,(255,150,50)),(ruta_pts_global,(0,255,0))]:
                p2d = proyectar(pts3, K, D)
                cv2.polylines(cam_view, [np.array(p2d,np.int32).reshape(-1,1,2)], True, col3, 2)

            # Minimap
            for pts3, col3 in [(mesa_pts,(255,150,50)),(ruta_pts_global,(0,255,0))]:
                cv2.polylines(mm,
                    [np.array([mundo_a_mm(p[0],p[1]) for p in pts3],np.int32).reshape(-1,1,2)],
                    True, col3, 1)

            cv2.putText(cam_view,
                f"Mesa: {abs(t1x-t2x):.2f}x{abs(t1y-t2y):.2f} m",
                (8,68), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,150,50), 2)

        # ── Robot: centroide, orientación y Pure Pursuit ──────
        if 0 in memoria_tags and (ahora - memoria_tags[0]['ts']) < 0.8:
            tag  = memoria_tags[0]
            rx_t = tag['m_x']; ry_t = tag['m_y']
            rvec = tag['rvec']; tvec = tag['tvec']
            R, _ = cv2.Rodrigues(rvec)

            # Corrección de centroide
            off_cam  = R @ np.array([[OFFSET_X_TAG],[OFFSET_Y_TAG],[0.0]], dtype=np.float64)
            cx_m     = rx_t + off_cam[0][0]
            cy_m     = ry_t + off_cam[1][0]

            # Orientación del robot (ángulo de la nariz en coords mundo)
            vec_nariz = R @ np.array([[0.0],[-0.15],[0.0]])
            robot_yaw = math.atan2(vec_nariz[1][0], vec_nariz[0][0])

            # Proyectar centroide y nariz
            p_cen = proyectar([(cx_m, cy_m, tag['m_z'])], K, D)[0]
            p_nar = proyectar([(cx_m+vec_nariz[0][0], cy_m+vec_nariz[1][0], tag['m_z'])], K, D)[0]
            cv2.arrowedLine(cam_view, p_cen, p_nar, (0,0,255), 3, tipLength=0.4)
            cv2.circle(cam_view, p_cen, 5, (255,255,0), -1)

            if abs(OFFSET_X_TAG)+abs(OFFSET_Y_TAG) > 0.01:
                p_tag = proyectar([(rx_t,ry_t,tag['m_z'])],K,D)[0]
                cv2.circle(cam_view, p_tag, 4, (0,165,255), -1)
                cv2.line(cam_view, p_tag, p_cen, (100,100,255), 1)

            p_mm = mundo_a_mm(cx_m, cy_m)
            if not estela_robot or estela_robot[-1] != p_mm: estela_robot.append(p_mm)
            for i in range(1, len(estela_robot)):
                cv2.line(mm, estela_robot[i-1], estela_robot[i], (0,120,255),
                         int(np.interp(i,[0,len(estela_robot)],[1,3])))

            # ══════════════════════════════════════════════════
            # PURE PURSUIT HOLONÓMICO → (vx_robot, vy_robot)
            # ══════════════════════════════════════════════════
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

                    if dist_real <= TOLERANCIA_LLEGADA_M:
                        vx_cmd = 0.0; vy_cmd = 0.0
                        estado_mision += 1
                        print(f"[✓] Waypoint capturado → estado {estado_mision}")
                        if estado_mision > 4:
                            mision_activa = False
                            print("[✓] Misión completada.")

                    else:
                        # ── Calcular zanahoria (lookahead point) ──────────
                        L_linea = math.hypot(tx_fin-ox, ty_fin-oy)
                        if L_linea < 1e-4: L_linea = 1e-4
                        ux, uy  = (tx_fin-ox)/L_linea, (ty_fin-oy)/L_linea
                        vx_proj, vy_proj = cx_m-ox, cy_m-oy
                        proj    = vx_proj*ux + vy_proj*uy
                        d_virt  = max(0.0, min(proj+LOOKAHEAD_FIJO, L_linea))
                        tx_zan  = ox + d_virt*ux
                        ty_zan  = oy + d_virt*uy

                        # ── Vector mundo: del centroide al lookahead ──────
                        dx_w = tx_zan - cx_m
                        dy_w = ty_zan - cy_m
                        d_zan = math.hypot(dx_w, dy_w)
                        if d_zan < 1e-4: d_zan = 1e-4

                        # ── [NUEVO] Rotar al frame del robot ──────────────
                        # El robot tiene yaw = robot_yaw respecto al mundo.
                        # Para ir en dirección (dx_w, dy_w) en el mundo,
                        # necesita aplicar (vx_r, vy_r) en su propio frame:
                        #   vx_r =  dx_w * cos(yaw) + dy_w * sin(yaw)
                        #   vy_r = -dx_w * sin(yaw) + dy_w * cos(yaw)
                        cos_y = math.cos(robot_yaw)
                        sin_y = math.sin(robot_yaw)
                        vx_dir =  (dx_w/d_zan)*cos_y + (dy_w/d_zan)*sin_y
                        vy_dir = -(dx_w/d_zan)*sin_y + (dy_w/d_zan)*cos_y

                        # ── Escalar al módulo de velocidad deseado ────────
                        v = V_LINEAL
                        if dist_real < 0.25:
                            v *= V_APROX_FACTOR

                        vx_cmd = v * vx_dir
                        vy_cmd = v * vy_dir
                        vy_cmd = max(-MAX_VY, min(MAX_VY, vy_cmd))

                        # Zanahoria en imagen
                        p_zan = proyectar([(tx_zan,ty_zan,tz_fin)],K,D)[0]
                        cv2.circle(cam_view, p_zan, 8, (0,200,255), -1)
                        cv2.line(cam_view, p_cen, p_zan, (0,200,255), 2)

                    # Waypoint final en imagen
                    p_obj = proyectar([(tx_fin,ty_fin,tz_fin)],K,D)[0]
                    cv2.circle(cam_view, p_obj, 10, (0,255,100), 2)

                    # ── UDP: "vx,vy,dist,estado_mision" ───────────────
                    paquete = f"{vx_cmd:.4f},{vy_cmd:.4f},{dist_real:.4f},{estado_mision}"
                    sock_udp.sendto(paquete.encode(), (IP_RASPBERRY, PUERTO_UDP))

                    # HUD
                    cv2.putText(cam_view, f"Target : {nombre_objetivo}",
                                (8,96),  cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,0,255), 2)
                    cv2.putText(cam_view, f"vx={vx_cmd:+.3f} m/s  vy={vy_cmd:+.3f} m/s",
                                (8,122), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,255), 2)
                    cv2.putText(cam_view, f"dist waypoint: {dist_real:.3f} m",
                                (8,148), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 2)

        # ── UI Global ────────────────────────────────────────
        estado_txt = (f"MISION | {nombre_objetivo}"
                      if mision_activa else "ESPERANDO — 's' iniciar  'r' reset  'q' salir")
        cv2.putText(cam_view, estado_txt, (8,36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0,255,0) if mision_activa else (0,80,255), 2)

        cv2.rectangle(mm, (0,0), (MM_SIZE-1,MM_SIZE-1), (200,200,200), 1)
        cam_view[8:8+MM_SIZE, 480:640] = mm

        cv2.imshow("Pickasso Holonómico — Dashboard", cam_view)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('r'):
            memoria_tags.clear(); estela_robot.clear()
            mision_activa=False; estado_mision=0; posicion_home=None
            print("[R] Reset.")
        elif k == ord('s') and not mision_activa:
            if len(ruta_pts_global)==4 and 0 in memoria_tags:
                tag = memoria_tags[0]
                R_s,_ = cv2.Rodrigues(tag['rvec'])
                off   = R_s @ np.array([[OFFSET_X_TAG],[OFFSET_Y_TAG],[0.0]])
                posicion_home = (tag['m_x']+off[0][0], tag['m_y']+off[1][0], tag['m_z'])
                mision_activa  = True
                estado_mision  = 1
                print("[►] Misión holonómica iniciada.")
            else:
                print("[✗] Faltan tags 0, 1 y 2.")

    cap.stop(); cv2.destroyAllWindows(); sock_udp.close()


if __name__ == '__main__':
    main()
