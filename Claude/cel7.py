"""
cel7.py — Pickasso AGV | PC Vision + Pure Pursuit
===================================================
BUGS CORREGIDOS respecto a cel6(2).py
--------------------------------------
[BUG 1] Rectángulos mal dibujados (causa principal del error visual)
        ANTES : metros_a_pixeles_radar() — proyección lineal que ignora Z.
                Con cámara cenital los puntos se desplazan del tag real.
        AHORA : cv2.projectPoints() con la matriz intrínseca real.
                Los rectángulos se dibujan exactamente donde están en el mundo.

[BUG 2] Centroide incorrecto al presionar 's'
        ANTES : posicion_home = tvec del tag (no el centroide del carro).
        AHORA : posicion_home = centroide corregido con OFFSET_X/Y_TAG.

[BUG 3] Radar overlay dibujado sobre la imagen con escala incorrecta
        ANTES : la "zanahoria" y líneas usaban coords radar sobre cam_view.
        AHORA : todo se proyecta con projectPoints, radar sólo es minimap.

[BUG 4] Doble Distorsión y Escala ArUco
        Se eliminó cv2.remap previo al solvePnP para no aplastar el 3D.
        Se ajustó el marcador ArUco a su tamaño físico real (8.7 cm).

[MEJORA] Minimap compacto en esquina superior derecha (no ocupa el frame).

PARÁMETROS DE SINTONIZACIÓN — busca la sección [TUNING] abajo.
"""

import cv2
import numpy as np
import math
import threading
import socket
import time
from collections import deque


# ─────────────────────────────────────────────────────────────────────
# CÁMARA TURBO  (grab/retrieve sin lag)
# ─────────────────────────────────────────────────────────────────────
class CamaraIP_UltraRapida:
    def __init__(self, url):
        self.stream = cv2.VideoCapture(url)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.stream.isOpened():
            raise Exception("No se pudo conectar a DroidCam. Revisa la IP.")
        self.stopped       = False
        self.frame_fresco  = None

    def start(self):
        threading.Thread(target=self._update, daemon=True).start()
        while self.frame_fresco is None and not self.stopped:
            time.sleep(0.05)
        print("[OK] Video en vivo recibido (640×480).")
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
# FUNCIÓN AUXILIAR: proyectar puntos 3D (coord cámara) → 2D imagen
# ─────────────────────────────────────────────────────────────────────
def proyectar(puntos_3d, K, D):
    """
    puntos_3d : lista de tuplas (x, y, z) en coordenadas de cámara [m]
    Devuelve  : lista de tuplas (px, py) en píxeles de imagen.

    NOTA: usamos rvec=0 tvec=0 porque los puntos ya están en el
    frame de la cámara (vienen directamente del tvec de solvePnP).
    """
    pts = np.array(puntos_3d, dtype=np.float32).reshape(-1, 1, 3)
    pts2d, _ = cv2.projectPoints(pts,
                                  np.zeros((3, 1)),
                                  np.zeros((3, 1)),
                                  K, D)
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
    #  [TUNING A] GEOMETRÍA DEL ROBOT  ← medir con cinta métrica
    # ══════════════════════════════════════════════════════════════
    # OFFSET_X_TAG  : desplazamiento lateral del tag respecto al centroide.
    OFFSET_X_TAG = 0.00   # m

    # OFFSET_Y_TAG  : desplazamiento longitudinal del tag respecto al centroide.
    OFFSET_Y_TAG = 0.15   # m

    # ══════════════════════════════════════════════════════════════
    #  [TUNING B] GEOMETRÍA DE LA RUTA
    # ══════════════════════════════════════════════════════════════
    # OFFSET_MESA_M : expansión del rectángulo alrededor de la mesa.
    OFFSET_MESA_M = 0.40   # m

    # TOLERANCIA_LLEGADA_M : radio de captura del waypoint.
    TOLERANCIA_LLEGADA_M = 0.05   # m

    # ══════════════════════════════════════════════════════════════
    #  [TUNING C] PURE PURSUIT
    # ══════════════════════════════════════════════════════════════
    LOOKAHEAD_FIJO = 0.20   # m
    V_LINEAL = 0.08   # m/s
    V_APROX_FACTOR = 0.60   # sin unidades  (0.5 = 50% de velocidad)
    MAX_OMEGA = 1.2   # rad/s

    # ══════════════════════════════════════════════════════════════
    #  INICIALIZACIÓN
    # ══════════════════════════════════════════════════════════════
    print(f"[*] Conectando a DroidCam ({DROIDCAM_URL})...")
    try:
        cap = CamaraIP_UltraRapida(DROIDCAM_URL).start()
    except Exception as e:
        print(f"[ERROR] {e}"); return

    sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[*] UDP → {IP_RASPBERRY}:{PUERTO_UDP}")

    # ── Calibración ─────────────────────────────────────────────
    print("[*] Cargando calibración (640×480)...")
    fs = cv2.FileStorage("parametros_droidcam.yaml", cv2.FILE_STORAGE_READ)
    if fs.isOpened():
        K = fs.getNode("camera_matrix").mat()
        D = fs.getNode("dist_coeffs").mat()
        fs.release()
        print("[OK] Calibración cargada.")
    else:
        print("[WARN] Sin calibración — usando focal estimada.")
        f_est = 640 * 0.9
        K = np.array([[f_est, 0, 320],
                      [0, f_est, 240],
                      [0,     0,   1]], dtype=np.float32)
        D = np.zeros((4, 1))

    # ── ArUco ───────────────────────────────────────────────────
    aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_params.cornerRefinementMethod    = cv2.aruco.CORNER_REFINE_SUBPIX
    aruco_params.adaptiveThreshWinSizeStep = 10
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # ---> AQUÍ ESTÁ LA CORRECCIÓN DE TAMAÑO (8.7 cm) <---
    hs         = 0.087 / 2.0
    obj_points = np.array([[-hs,  hs, 0], [ hs,  hs, 0],
                            [ hs, -hs, 0], [-hs, -hs, 0]], dtype=np.float32)

    # ── Estado de navegación ────────────────────────────────────
    memoria_tags         = {}
    estela_robot         = deque(maxlen=80)
    ruta_pts_global      = []
    mision_activa        = False
    estado_mision        = 0
    objetivo_actual      = None
    nombre_objetivo      = ""
    posicion_home        = None
    sentido_giro_global  = 1

    # Minimap (esquina superior derecha, 160×160 px)
    MM_SIZE   = 160
    MM_ESCALA = 80   # px/m

    def mundo_a_mm(mx, my):
        """Coordenadas de mundo → píxeles del minimap."""
        return (int(MM_SIZE/2 + mx * MM_ESCALA),
                int(MM_SIZE/2 - my * MM_ESCALA))

    # ════════════════════════════════════════════════════════════
    # BUCLE PRINCIPAL
    # ════════════════════════════════════════════════════════════
    while True:
        frame_raw = cap.read()
        if frame_raw is None:
            time.sleep(0.005)
            continue

        # ---> AQUÍ ESTÁ LA CORRECCIÓN DE DISTORSIÓN (Imagen cruda) <---
        cam_view = frame_raw.copy()

        # Minimap limpio
        mm = np.zeros((MM_SIZE, MM_SIZE, 3), np.uint8)
        for i in range(0, MM_SIZE, 20):
            cv2.line(mm, (i, 0), (i, MM_SIZE), (30, 30, 30), 1)
            cv2.line(mm, (0, i), (MM_SIZE, i), (30, 30, 30), 1)

        # ── Detección de tags ────────────────────────────────────
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
                    cx = int(np.mean(corners[i][0][:, 0]))
                    cy = int(np.mean(corners[i][0][:, 1]))
                    memoria_tags[mid] = {
                        'm_x': tvec[0][0], 'm_y': tvec[1][0], 'm_z': tvec[2][0],
                        'c_x': cx, 'c_y': cy, 'rvec': rvec, 'tvec': tvec,
                        'ts': ahora}
            cv2.aruco.drawDetectedMarkers(cam_view, corners)

        # Etiquetas con indicador de tag perdido
        for mid, td in memoria_tags.items():
            age = ahora - td['ts']
            color = (0, 165, 255) if mid == 0 else (0, 255, 0)
            label = ("Robot" if mid == 0 else f"Tag {mid}")
            if age > 0.5:
                label += f" ??{age:.1f}s"
                color = (0, 0, 220)
            cv2.putText(cam_view, label,
                        (td['c_x'] - 40, td['c_y'] - 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # ── Geometría del rectángulo ─────────────────────────────
        if 1 in memoria_tags and 2 in memoria_tags:
            t1 = memoria_tags[1]
            t2 = memoria_tags[2]
            t1x, t1y, t1z = t1['m_x'], t1['m_y'], t1['m_z']
            t2x, t2y, t2z = t2['m_x'], t2['m_y'], t2['m_z']
            cz = (t1z + t2z) / 2.0

            mesa_pts = [(t1x, t1y, t1z), (t2x, t1y, cz),
                        (t2x, t2y, t2z), (t1x, t2y, cz)]
            cmx = (t1x + t2x) / 2.0
            cmy = (t1y + t2y) / 2.0

            def expandir(px, py, pz):
                nx = px + OFFSET_MESA_M if px > cmx else px - OFFSET_MESA_M
                ny = py + OFFSET_MESA_M if py > cmy else py - OFFSET_MESA_M
                return (nx, ny, pz)

            ruta_pts_global = [expandir(*p) for p in mesa_pts]

            # ── [FIX] Dibujar con proyección real (no radar lineal) ──
            # Mesa (naranja)
            p2d_mesa = proyectar(mesa_pts, K, D)
            cv2.polylines(cam_view,
                          [np.array(p2d_mesa, np.int32).reshape(-1, 1, 2)],
                          True, (255, 150, 50), 2)

            # Ruta con offset (verde)
            p2d_ruta = proyectar(ruta_pts_global, K, D)
            cv2.polylines(cam_view,
                          [np.array(p2d_ruta, np.int32).reshape(-1, 1, 2)],
                          True, (0, 255, 0), 2)

            # Minimap: mesa y ruta
            cv2.polylines(mm,
                [np.array([mundo_a_mm(p[0], p[1]) for p in mesa_pts],
                           np.int32).reshape(-1, 1, 2)],
                True, (255, 150, 50), 1)
            cv2.polylines(mm,
                [np.array([mundo_a_mm(p[0], p[1]) for p in ruta_pts_global],
                           np.int32).reshape(-1, 1, 2)],
                True, (0, 255, 0), 1)

            # Dimensiones de la mesa
            cv2.putText(cam_view,
                        f"Mesa: {abs(t1x-t2x):.2f}x{abs(t1y-t2y):.2f} m",
                        (8, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 150, 50), 2)

        # ── Robot: centroide y orientación ──────────────────────
        if 0 in memoria_tags and (ahora - memoria_tags[0]['ts']) < 0.8:
            tag = memoria_tags[0]
            rx_tag = tag['m_x']
            ry_tag = tag['m_y']
            rvec   = tag['rvec']
            tvec   = tag['tvec']

            R, _ = cv2.Rodrigues(rvec)

            # ── [FIX] Corrección de centroide ──────────────────
            # El vector offset está en el frame del tag.
            # Frame del tag en cámara cenital:
            #   +X = derecha del robot, +Y = atrás del robot (nariz = -Y)
            offset_tag_frame = np.array([
                [OFFSET_X_TAG ],
                [OFFSET_Y_TAG ],
                [0.0          ]], dtype=np.float64)
            offset_cam = R @ offset_tag_frame
            # Centroide real del carro en coords de cámara
            cx_m = rx_tag + offset_cam[0][0]
            cy_m = ry_tag + offset_cam[1][0]

            # Orientación (vector nariz en coords de cámara)
            vec_nariz = R @ np.array([[0.0], [-0.15], [0.0]])
            robot_yaw = math.atan2(vec_nariz[1][0], vec_nariz[0][0])

            # ── [FIX] Proyectar centroide y nariz a píxeles reales ──
            p_cen_2d = proyectar([(cx_m, cy_m, tag['m_z'])], K, D)[0]
            p_nar_2d = proyectar(
                [(cx_m + vec_nariz[0][0], cy_m + vec_nariz[1][0], tag['m_z'])], K, D)[0]

            cv2.arrowedLine(cam_view, p_cen_2d, p_nar_2d, (0, 0, 255), 3, tipLength=0.4)
            cv2.circle(cam_view, p_cen_2d, 5, (255, 255, 0), -1)

            # Si el tag no está en el centroide, mostrar también el tag
            if abs(OFFSET_X_TAG) + abs(OFFSET_Y_TAG) > 0.01:
                p_tag_2d = proyectar([(rx_tag, ry_tag, tag['m_z'])], K, D)[0]
                cv2.circle(cam_view, p_tag_2d, 4, (0, 165, 255), -1)
                cv2.line(cam_view, p_tag_2d, p_cen_2d, (100, 100, 255), 1)

            # Estela en minimap
            p_mm = mundo_a_mm(cx_m, cy_m)
            if not estela_robot or estela_robot[-1] != p_mm:
                estela_robot.append(p_mm)
            for i in range(1, len(estela_robot)):
                cv2.line(mm, estela_robot[i-1], estela_robot[i],
                         (0, 120, 255),
                         int(np.interp(i, [0, len(estela_robot)], [1, 3])))

            # ══════════════════════════════════════════════════
            # PURE PURSUIT → (v, omega)
            # ══════════════════════════════════════════════════
            if mision_activa and len(ruta_pts_global) == 4 and posicion_home is not None:

                if   estado_mision == 1: origen = posicion_home;      objetivo_actual = ruta_pts_global[0]; nombre_objetivo = "1.Est.1"
                elif estado_mision == 2: origen = ruta_pts_global[0]; objetivo_actual = ruta_pts_global[1]; nombre_objetivo = "2.Esq.1"
                elif estado_mision == 3: origen = ruta_pts_global[1]; objetivo_actual = ruta_pts_global[2]; nombre_objetivo = "3.Est.2"
                elif estado_mision == 4: origen = ruta_pts_global[2]; objetivo_actual = ruta_pts_global[3]; nombre_objetivo = "4.Esq.2"
                else: objetivo_actual = None

                if objetivo_actual:
                    tx_fin, ty_fin, tz_fin = objetivo_actual
                    ox, oy, oz = origen

                    # Distancia al waypoint real (desde centroide)
                    dist_real = math.hypot(tx_fin - cx_m, ty_fin - cy_m)

                    if dist_real <= TOLERANCIA_LLEGADA_M:
                        # Waypoint capturado
                        v_cmd    = 0.0
                        omega_cmd = 0.0
                        estado_mision += 1
                        print(f"[✓] Waypoint {estado_mision - 1} capturado → pasando a estado {estado_mision}")
                        if estado_mision > 4:
                            mision_activa = False
                            print("[✓] Misión completada.")

                    else:
                        # ── Pure Pursuit: calcular zanahoria ──────
                        L_linea = math.hypot(tx_fin - ox, ty_fin - oy)
                        if L_linea < 1e-4: L_linea = 1e-4

                        ux, uy = (tx_fin - ox) / L_linea, (ty_fin - oy) / L_linea
                        vx, vy = cx_m - ox, cy_m - oy
                        proj   = vx * ux + vy * uy

                        d_virtual  = min(proj + LOOKAHEAD_FIJO, L_linea)
                        d_virtual  = max(d_virtual, 0.0)
                        tx_zan     = ox + d_virtual * ux
                        ty_zan     = oy + d_virtual * uy

                        dx_zan     = tx_zan - cx_m
                        dy_zan     = ty_zan - cy_m
                        ld         = max(math.hypot(dx_zan, dy_zan), 0.05)

                        alpha      = math.atan2(dy_zan, dx_zan) - robot_yaw
                        alpha      = (alpha + math.pi) % (2 * math.pi) - math.pi

                        v_cmd      = V_LINEAL
                        if dist_real < 0.25:
                            v_cmd *= V_APROX_FACTOR

                        omega_cmd  = (2.0 * v_cmd * math.sin(alpha)) / ld
                        omega_cmd  = max(-MAX_OMEGA, min(MAX_OMEGA, omega_cmd))

                        # ── [FIX] Zanahoria en imagen (proyección real) ──
                        p_zan_2d = proyectar([(tx_zan, ty_zan, tz_fin)], K, D)[0]
                        cv2.circle(cam_view, p_zan_2d, 8, (0, 200, 255), -1)
                        cv2.line(cam_view, p_cen_2d, p_zan_2d, (0, 200, 255), 2)

                    # ── Waypoint final en imagen ─────────────────
                    p_obj_2d = proyectar([(tx_fin, ty_fin, tz_fin)], K, D)[0]
                    cv2.circle(cam_view, p_obj_2d, 10, (0, 255, 100), 2)
                    cv2.line(cam_view, p_cen_2d, p_obj_2d, (255, 0, 255), 1)

                    # Minimap: waypoint
                    p_obj_mm = mundo_a_mm(tx_fin, ty_fin)
                    cv2.circle(mm, p_obj_mm, 4, (0, 255, 100), -1)
                    cv2.line(mm, mundo_a_mm(cx_m, cy_m), p_obj_mm, (255, 0, 255), 1)

                    # ── UDP: "v,omega,dist,estado_mision" ────────
                    paquete = f"{v_cmd:.4f},{omega_cmd:.4f},{dist_real:.4f},{estado_mision}"
                    sock_udp.sendto(paquete.encode(), (IP_RASPBERRY, PUERTO_UDP))

                    # ── HUD de telemetría ─────────────────────────
                    cv2.putText(cam_view, f"Target : {nombre_objetivo}",
                                (8, 98),  cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)
                    cv2.putText(cam_view, f"v={v_cmd:.3f} m/s  w={omega_cmd:+.3f} rad/s",
                                (8, 124), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                    cv2.putText(cam_view, f"Dist waypoint: {dist_real:.3f} m",
                                (8, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)

        # ── UI Global ────────────────────────────────────────────
        estado_txt = (f"MISION | {nombre_objetivo}"
                      if mision_activa else "ESPERANDO — 's' iniciar  'r' reset  'q' salir")
        cv2.putText(cam_view, estado_txt, (8, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 255, 0) if mision_activa else (0, 80, 255), 2)

        # Insertar minimap (esquina superior derecha con borde)
        cv2.rectangle(mm, (0, 0), (MM_SIZE-1, MM_SIZE-1), (200, 200, 200), 1)
        cam_view[8: 8 + MM_SIZE, 640 - 8 - MM_SIZE: 640 - 8] = mm

        cv2.imshow("Pickasso — Dashboard", cam_view)

        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord('q'):
            break
        elif tecla == ord('r'):
            memoria_tags.clear(); estela_robot.clear()
            mision_activa = False; estado_mision = 0
            posicion_home = None; objetivo_actual = None
            print("[R] Reset.")
        elif tecla == ord('s') and not mision_activa:
            if len(ruta_pts_global) == 4 and 0 in memoria_tags:
                tag = memoria_tags[0]
                R_s, _ = cv2.Rodrigues(tag['rvec'])
                off = R_s @ np.array([[OFFSET_X_TAG], [OFFSET_Y_TAG], [0.0]])
                # ── [FIX] posicion_home = centroide, no tvec del tag ──
                posicion_home = (
                    tag['m_x'] + off[0][0],
                    tag['m_y'] + off[1][0],
                    tag['m_z'])

                # Calcular sentido de giro (producto cruz)
                t1x, t1y = ruta_pts_global[0][0], ruta_pts_global[0][1]
                t2x, t2y = ruta_pts_global[2][0], ruta_pts_global[2][1]
                hx, hy   = posicion_home[0], posicion_home[1]
                cross_z  = (t1x - hx) * (t2y - hy) - (t1y - hy) * (t2x - hx)
                sentido_giro_global = 1 if cross_z < 0 else -1

                mision_activa = True
                estado_mision = 1
                print(f"[►] Misión iniciada — sentido {'IZQ' if sentido_giro_global==1 else 'DER'}")
            else:
                print("[✗] Faltan tags 0, 1 y 2.")

    cap.stop()
    cv2.destroyAllWindows()
    sock_udp.close()


if __name__ == '__main__':
    main()
