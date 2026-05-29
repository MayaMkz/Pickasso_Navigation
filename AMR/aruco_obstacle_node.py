"""
aruco_obstacle_node.py — Pickasso AMR | Obstáculo por ArUco ID 10
=================================================================
Detecta el ArUco marker ID 10 con la cámara DroidCam y publica
su posición en coordenadas del mapa como PointStamped en /aruco_obstacle.

El planner_node escucha /aruco_obstacle igual que /clicked_point (RViz),
lo agrega al grid y recalcula la ruta automáticamente.

Rescatado de Cel8.py:
  - Conexión DroidCam con CamaraIP_UltraRapida (grab/retrieve, sin lag)
  - Calibración cargada desde parametros_droidcam.yaml (mismo nombre que Cel8)
  - Tamaño del marcador: hs = 0.063/2.0  → marcador de 6.3 cm
  - Diccionario: DICT_4X4_50
  - DetectorParameters con CORNER_REFINE_SUBPIX + adaptiveThreshWinSizeStep=10
  - Pose estimada con solvePnP SOLVEPNP_IPPE_SQUARE (mismo método que Cel8)

Dependencias:
    pip install opencv-contrib-python rclpy
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, PointStamped
from std_msgs.msg import Header

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import threading
import time


# ─────────────────────────────────────────────────────────────────────────────
# Clase de cámara rápida — idéntica a Cel8.py para evitar lag de buffer
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

    # ── Configuración ── (mismos valores que Cel8.py) ─────────────────────
    DROIDCAM_URL    = "http://192.168.137.24:4747/video"   # igual que Cel8
    CALIB_FILE      = "parametros_droidcam.yaml"           # igual que Cel8
    TARGET_ID       = 10           # ID que se trata como obstáculo
    MARKER_SIDE_M   = 0.063        # tamaño físico del marcador (m) — igual que Cel8 (hs*2)
    DETECTION_HZ    = 10.0         # frecuencia de detección
    COOLDOWN_S      = 2.0          # segundos entre publicaciones del mismo obstáculo
    MIN_DIST_NUEVA  = 0.15         # (m) mínimo para considerar obstáculo "nuevo"
    # ──────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('aruco_obstacle_node')

        # Publisher
        self.obstacle_pub = self.create_publisher(
            PointStamped, '/aruco_obstacle', 10
        )

        # Pose del robot (para proyectar al mapa)
        self.robot_pose = None
        self.create_subscription(Pose2D, '/amr_pose', self._pose_cb, 10)

        # ── Calibración (igual que Cel8.py) ──────────────────────────────
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

        # obj_points idéntico a Cel8.py
        hs = self.MARKER_SIDE_M / 2.0
        self.obj_points = np.array(
            [[-hs, hs, 0], [hs, hs, 0], [hs, -hs, 0], [-hs, -hs, 0]],
            dtype=np.float32
        )

        # ── Cámara DroidCam ───────────────────────────────────────────────
        self.get_logger().info(f"Conectando a DroidCam: {self.DROIDCAM_URL}")
        try:
            self.cap = CamaraIP_UltraRapida(self.DROIDCAM_URL).start()
            self.get_logger().info("[OK] DroidCam conectada. Buscando ArUco ID 10...")
        except Exception as e:
            self.get_logger().error(f"[ERROR cámara] {e}")
            self.cap = None

        # Anti-flood
        self._last_pub_time = 0.0
        self._last_pub_pos  = None

        # Timer de detección
        self.create_timer(1.0 / self.DETECTION_HZ, self._loop)

        self.get_logger().info("ArUco Obstacle Node listo.")

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────

    def _pose_cb(self, msg):
        self.robot_pose = msg

    # ──────────────────────────────────────────────────────────────────────
    # Loop principal de detección
    # ──────────────────────────────────────────────────────────────────────

    def _loop(self):
        if self.cap is None or self.cap.stopped:
            return

        frame_raw = self.cap.read()
        if frame_raw is None:
            return

        # Undistort — igual que Cel8.py usa cv2.remap
        frame = cv2.remap(frame_raw, self.map1, self.map2, cv2.INTER_LINEAR)
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return

        for i, mid_arr in enumerate(ids):
            mid = int(mid_arr[0])
            if mid != self.TARGET_ID:
                continue

            # ── Estimar pose con solvePnP — idéntico a Cel8.py ──────────
            ok, rvec, tvec = cv2.solvePnP(
                self.obj_points,
                corners[i][0],
                self.K,
                self.D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok:
                continue

            # tvec está en el frame de la cámara:
            #   tvec[0] → lateral  (X cámara, + = derecha)
            #   tvec[1] → vertical (Y cámara, + = abajo)
            #   tvec[2] → profundidad (Z cámara, + = lejos)
            dist_m    = float(tvec[2][0])   # distancia frontal al marcador
            lateral_m = float(tvec[0][0])   # desplazamiento lateral

            self.get_logger().info(
                f"ArUco ID {mid} | dist={dist_m:.2f} m  lateral={lateral_m:.2f} m"
            )

            # ── Proyectar al frame del mapa ──────────────────────────────
            world_pos = self._cam_to_world(dist_m, lateral_m)
            if world_pos is None:
                self.get_logger().warn(
                    "Pose del robot no disponible todavía — obstáculo no publicado."
                )
                return

            wx, wy = world_pos

            # ── Cooldown: evitar spam del mismo obstáculo ────────────────
            now = time.time()
            if self._last_pub_pos is not None:
                d_ant = math.hypot(wx - self._last_pub_pos[0],
                                   wy - self._last_pub_pos[1])
                if d_ant < self.MIN_DIST_NUEVA and (now - self._last_pub_time) < self.COOLDOWN_S:
                    return

            # ── Publicar ─────────────────────────────────────────────────
            msg            = PointStamped()
            msg.header     = Header()
            msg.header.frame_id = "map"
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.point.x    = wx
            msg.point.y    = wy
            msg.point.z    = 0.0
            self.obstacle_pub.publish(msg)

            self._last_pub_time = now
            self._last_pub_pos  = (wx, wy)

            self.get_logger().info(
                f"[✓] Obstáculo ArUco ID {mid} → mapa X={wx:.3f} m  Y={wy:.3f} m"
            )
            break  # un obstáculo por frame es suficiente

    # ──────────────────────────────────────────────────────────────────────
    # Proyección cámara → coordenadas mapa
    # ──────────────────────────────────────────────────────────────────────

    def _cam_to_world(self, dist_m: float, lateral_m: float):
        """
        Convierte (distancia frontal, offset lateral) en coordenadas absolutas
        del mapa usando la pose actual del robot.

        Se asume cámara mirando hacia adelante del robot
        (misma dirección que el eje de avance principal del AMR).
        Si tu cámara está rotada, ajusta cam_offset_angle_rad.
        """
        if self.robot_pose is None:
            return None

        rx, ry    = self.robot_pose.x, self.robot_pose.y
        rtheta    = self.robot_pose.theta

        # Posición relativa del obstáculo en frame del robot
        obs_x_robot =  lateral_m   # lateral: + = derecha del robot
        obs_y_robot =  dist_m      # frontal: + = adelante del robot

        # Rotar al frame del mapa con la orientación actual del robot
        cos_t = math.cos(rtheta)
        sin_t = math.sin(rtheta)

        wx = rx + cos_t * obs_x_robot - sin_t * obs_y_robot
        wy = ry + sin_t * obs_x_robot + cos_t * obs_y_robot

        return (wx, wy)

    # ──────────────────────────────────────────────────────────────────────
    # Cleanup
    # ──────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.cap is not None:
            self.cap.stop()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
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
