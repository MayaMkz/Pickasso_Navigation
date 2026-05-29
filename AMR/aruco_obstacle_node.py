"""
aruco_obstacle_node.py — Pickasso AMR | Obstáculo por ArUco ID 10
=================================================================
Cámara CENITAL FIJA (DroidCam en techo/poste) apuntando al área de trabajo.

Con cámara cenital, solvePnP devuelve tvec en el frame de la cámara donde:
  tvec[0] → X en metros desde el eje óptico (lateral)
  tvec[1] → Y en metros desde el eje óptico (longitudinal)
  tvec[2] → Z = altura de la cámara al marcador (constante, no se usa)

Para pasar a coordenadas del mapa solo se necesita UN offset fijo:
  (CAM_ORIGIN_X, CAM_ORIGIN_Y) = posición del eje óptico de la cámara
                                   en el sistema de coordenadas del mapa.

Esto se calibra UNA VEZ poniendo el marcador en una posición conocida del mapa
y ajustando CAM_ORIGIN_X / CAM_ORIGIN_Y hasta que coincida.

NO necesita /amr_pose para ubicar obstáculos — la cámara fija ya conoce
las coordenadas absolutas del área de trabajo.

Rescatado de Cel8.py:
  - CamaraIP_UltraRapida (grab/retrieve, sin lag de buffer)
  - calibración desde parametros_droidcam.yaml + fallback focal estimada
  - cv2.remap para undistort
  - DICT_4X4_50, CORNER_REFINE_SUBPIX, adaptiveThreshWinSizeStep=10
  - solvePnP con SOLVEPNP_IPPE_SQUARE y obj_points de 4 esquinas
  - tamaño de marcador: 0.063 m (6.3 cm lado)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import threading
import time


# ─────────────────────────────────────────────────────────────────────────────
# Cámara rápida — idéntica a Cel8.py
# ─────────────────────────────────────────────────────────────────────────────
class CamaraIP_UltraRapida:
    def __init__(self, url):
        self.stream = cv2.VideoCapture(url)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.stream.isOpened():
            raise Exception(f"No se pudo conectar a DroidCam: {url}")
        self.stopped      = False
        self.frame_fresco = None
        self._lock        = threading.Lock()

    def start(self):
        threading.Thread(target=self._update, daemon=True).start()
        t0 = time.time()
        while self.frame_fresco is None and not self.stopped:
            if time.time() - t0 > 8.0:
                raise Exception("Timeout esperando primer frame de DroidCam.")
            time.sleep(0.05)
        print("[OK] DroidCam 640×480 en vivo.")
        return self

    def _update(self):
        while not self.stopped:
            if self.stream.grab():
                _, img = self.stream.retrieve()
                with self._lock:
                    self.frame_fresco = cv2.resize(img, (640, 480))
            else:
                self.stopped = True

    def read(self):
        with self._lock:
            f = self.frame_fresco
            self.frame_fresco = None
        return f

    def stop(self):
        self.stopped = True
        self.stream.release()


# ─────────────────────────────────────────────────────────────────────────────
# Nodo ROS 2
# ─────────────────────────────────────────────────────────────────────────────
class ArucoObstacleNode(Node):

    # ══════════════════════════════════════════════════════════════════════
    #  RED — igual que Cel8.py
    # ══════════════════════════════════════════════════════════════════════
    DROIDCAM_URL  = "http://192.168.137.24:4747/video"
    CALIB_FILE    = "parametros_droidcam.yaml"

    # ══════════════════════════════════════════════════════════════════════
    #  MARCADOR
    # ══════════════════════════════════════════════════════════════════════
    TARGET_ID     = 10       # ID que se trata como obstáculo
    MARKER_SIDE_M = 0.063    # lado físico del marcador en metros (igual que Cel8)

    # ══════════════════════════════════════════════════════════════════════
    #  CALIBRACIÓN DE ORIGEN — LO ÚNICO QUE DEBES AJUSTAR
    # ══════════════════════════════════════════════════════════════════════
    #
    #  CAM_ORIGIN_X, CAM_ORIGIN_Y:
    #    Posición del eje óptico de la cámara en el sistema de coordenadas
    #    del mapa (config.py). Normalmente es el centro físico del área
    #    de trabajo si la cámara está centrada, o la esquina si está en un extremo.
    #
    #  Cómo calibrar:
    #    1. Pon el marcador ID 10 en una posición conocida del mapa,
    #       por ejemplo en (1.50, 1.90) — el centro del área.
    #    2. Corre el nodo y observa el log "[CRUDO]".
    #    3. Ajusta CAM_ORIGIN_X = 1.50 - crudo_x
    #                CAM_ORIGIN_Y = 1.90 - crudo_y
    #    4. Verifica que "[MAPA]" reporte (1.50, 1.90). Listo.
    #
    #  Valor inicial: centro del área de trabajo de config.py
    #    AREA_WIDTH=3.0 → centro X=1.50
    #    AREA_HEIGHT=3.8 → centro Y=1.90
    CAM_ORIGIN_X  = 1.50    # m — ajustar con el procedimiento de arriba
    CAM_ORIGIN_Y  = 1.90    # m — ajustar con el procedimiento de arriba

    #  CAM_ROTATION_RAD:
    #    Ángulo de rotación de la cámara respecto al mapa (en radianes).
    #    0.0  = cámara alineada con los ejes del mapa (+X derecha, +Y arriba)
    #    Ajustar si el celular/cámara no está perfectamente alineado con el área.
    CAM_ROTATION_RAD = 0.0  # rad — ajustar si la imagen está rotada

    #  FLIP_X, FLIP_Y:
    #    Dependiendo de cómo esté montado el celular, puede que los ejes
    #    de la cámara estén invertidos respecto al mapa.
    #    Cambiar a -1.0 si el obstáculo aparece en el lado opuesto al real.
    FLIP_X = 1.0   # 1.0 normal | -1.0 invertir eje X
    FLIP_Y = 1.0   # 1.0 normal | -1.0 invertir eje Y

    # ══════════════════════════════════════════════════════════════════════
    #  ANTI-FLOOD
    # ══════════════════════════════════════════════════════════════════════
    DETECTION_HZ  = 10.0    # Hz de detección
    COOLDOWN_S    = 2.0     # segundos mínimos entre publicaciones del mismo punto
    MIN_DIST_NUEVA = 0.15   # (m) para considerar un obstáculo como "nuevo"

    def __init__(self):
        super().__init__('aruco_obstacle_node')

        self.obstacle_pub = self.create_publisher(PointStamped, '/aruco_obstacle', 10)

        # ── Calibración — idéntico a Cel8.py ─────────────────────────────
        fs = cv2.FileStorage(self.CALIB_FILE, cv2.FILE_STORAGE_READ)
        if fs.isOpened():
            self.K = fs.getNode("camera_matrix").mat()
            self.D = fs.getNode("dist_coeffs").mat()
            fs.release()
            self.get_logger().info(f"[OK] Calibración cargada desde {self.CALIB_FILE}")
        else:
            self.get_logger().warn(
                f"[WARN] {self.CALIB_FILE} no encontrado — usando focal estimada."
            )
            f_est  = 640 * 0.9
            self.K = np.array([[f_est,0,320],[0,f_est,240],[0,0,1]], dtype=np.float32)
            self.D = np.zeros((4, 1), dtype=np.float32)

        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, None, self.K, (640, 480), cv2.CV_32FC1
        )

        # ── ArUco — idéntico a Cel8.py ────────────────────────────────────
        aruco_dict   = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        aruco_params = aruco.DetectorParameters()
        aruco_params.cornerRefinementMethod    = aruco.CORNER_REFINE_SUBPIX
        aruco_params.adaptiveThreshWinSizeStep = 10
        self.detector = aruco.ArucoDetector(aruco_dict, aruco_params)

        hs = self.MARKER_SIDE_M / 2.0
        self.obj_points = np.array(
            [[-hs, hs, 0], [hs, hs, 0], [hs, -hs, 0], [-hs, -hs, 0]],
            dtype=np.float32
        )

        # ── Cámara ───────────────────────────────────────────────────────
        self.get_logger().info(f"Conectando a DroidCam: {self.DROIDCAM_URL}")
        try:
            self.cap = CamaraIP_UltraRapida(self.DROIDCAM_URL).start()
            self.get_logger().info(
                f"[OK] Buscando ArUco ID {self.TARGET_ID}...\n"
                f"     Origen cámara en mapa: ({self.CAM_ORIGIN_X}, {self.CAM_ORIGIN_Y}) m\n"
                f"     Rotación cámara: {math.degrees(self.CAM_ROTATION_RAD):.1f}°"
            )
        except Exception as e:
            self.get_logger().error(f"[ERROR cámara] {e}")
            self.cap = None

        self._last_pub_time = 0.0
        self._last_pub_pos  = None

        self.create_timer(1.0 / self.DETECTION_HZ, self._loop)
        self.get_logger().info("ArUco Obstacle Node listo.")

    # ──────────────────────────────────────────────────────────────────────
    # Loop de detección
    # ──────────────────────────────────────────────────────────────────────

    def _loop(self):
        if self.cap is None or self.cap.stopped:
            return

        frame_raw = self.cap.read()
        if frame_raw is None:
            return

        frame = cv2.remap(frame_raw, self.map1, self.map2, cv2.INTER_LINEAR)
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return

        for i, mid_arr in enumerate(ids):
            mid = int(mid_arr[0])
            if mid != self.TARGET_ID:
                continue

            # ── solvePnP — idéntico a Cel8.py ────────────────────────────
            ok, rvec, tvec = cv2.solvePnP(
                self.obj_points,
                corners[i][0],
                self.K,
                self.D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok:
                continue

            # Con cámara cenital:
            #   tvec[0] = X en frame cámara (lateral en la imagen)
            #   tvec[1] = Y en frame cámara (vertical en la imagen)
            #   tvec[2] = Z = distancia cámara-marcador (altura, constante)
            cam_x = float(tvec[0][0]) * self.FLIP_X
            cam_y = float(tvec[1][0]) * self.FLIP_Y

            self.get_logger().info(
                f"[CRUDO] ArUco ID {mid} | cam_x={cam_x:.3f} m  cam_y={cam_y:.3f} m  "
                f"altura={float(tvec[2][0]):.3f} m"
            )

            # ── Transformar al frame del mapa ─────────────────────────────
            wx, wy = self._cam_to_map(cam_x, cam_y)

            self.get_logger().info(
                f"[MAPA]  ArUco ID {mid} | X={wx:.3f} m  Y={wy:.3f} m"
            )

            # ── Cooldown ──────────────────────────────────────────────────
            now = time.time()
            if self._last_pub_pos is not None:
                d = math.hypot(wx - self._last_pub_pos[0], wy - self._last_pub_pos[1])
                if d < self.MIN_DIST_NUEVA and (now - self._last_pub_time) < self.COOLDOWN_S:
                    break

            # ── Publicar ──────────────────────────────────────────────────
            msg = PointStamped()
            msg.header.frame_id = "map"
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.point.x = wx
            msg.point.y = wy
            msg.point.z = 0.0
            self.obstacle_pub.publish(msg)

            self._last_pub_time = now
            self._last_pub_pos  = (wx, wy)

            self.get_logger().info(
                f"[✓] Obstáculo publicado → /aruco_obstacle  X={wx:.3f}  Y={wy:.3f}"
            )
            break

    # ──────────────────────────────────────────────────────────────────────
    # Transformación cámara cenital → mapa
    # ──────────────────────────────────────────────────────────────────────

    def _cam_to_map(self, cam_x: float, cam_y: float) -> tuple:
        """
        Cámara cenital fija: la transformación es solo rotación + traslación.

        El eje óptico de la cámara (0,0) corresponde a (CAM_ORIGIN_X, CAM_ORIGIN_Y)
        en el mapa. Si la cámara está rotada respecto al mapa, se aplica
        CAM_ROTATION_RAD antes de sumar el origen.

        Para una cámara perfectamente alineada (CAM_ROTATION_RAD=0):
            map_x = CAM_ORIGIN_X + cam_x
            map_y = CAM_ORIGIN_Y + cam_y
        """
        cos_r = math.cos(self.CAM_ROTATION_RAD)
        sin_r = math.sin(self.CAM_ROTATION_RAD)

        rotated_x = cos_r * cam_x - sin_r * cam_y
        rotated_y = sin_r * cam_x + cos_r * cam_y

        map_x = self.CAM_ORIGIN_X + rotated_x
        map_y = self.CAM_ORIGIN_Y + rotated_y

        return (map_x, map_y)

    # ──────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.cap is not None:
            self.cap.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
