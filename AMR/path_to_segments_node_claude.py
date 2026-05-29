import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from std_msgs.msg import Bool

import socket
import time
import threading


class PathToSegmentsNode(Node):
    """
    Convierte el path planeado en comandos UDP ortogonales (F/B/R/L/S)
    hacia la Raspberry Pi.

    Mejoras:
    - Recepción de ruta nueva durante ejecución activa → frena y replantea.
    - Acumulación correcta de segmentos ortogonales (sin diagonales).
    - Thread separado para ejecución bloqueante (time.sleep) sin bloquear el spin.
    """

    def __init__(self):
        super().__init__('path_to_segments_node')

        self.raspberry_ip = "10.42.0.234"
        self.raspberry_port = 5006

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )

        self.start_sub = self.create_subscription(
            Bool,
            '/start_mission',
            self.start_callback,
            10
        )

        self.path_points = []
        self.ready = False

        self.min_segment = 0.05       # metros mínimos para enviar un segmento
        self.speed_estimate = 0.13    # m/s estimados del robot físico

        # Control de ejecución en thread
        self._executing = False
        self._abort = False
        self._lock = threading.Lock()

        self.get_logger().info("Path to Segments Node Started")
        self.get_logger().info("Esperando /planned_path y /start_mission...")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def path_callback(self, msg):
        new_points = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            new_points.append((x, y))

        # Ruta vacía = freno de emergencia
        if len(new_points) == 0:
            self.get_logger().warn("¡Ruta vacía recibida! Frenando.")
            self._abort_execution()
            self.send_stop()
            self.path_points = []
            self.ready = False
            return

        with self._lock:
            self.path_points = new_points
            self.ready = True

        # Si ya está ejecutando una ruta anterior, abortar y re-ejecutar con la nueva
        if self._executing:
            self.get_logger().info(
                "Nueva ruta recibida durante ejecución. Abortando ruta anterior y replanificando..."
            )
            self._abort_execution()
            # Dar un tick para que el thread anterior termine
            time.sleep(0.1)
            self._launch_execution()
        else:
            self.get_logger().info(
                f"Nueva ruta recibida con {len(self.path_points)} puntos. "
                "Esperando /start_mission..."
            )

    def start_callback(self, msg):
        if not msg.data:
            return

        if not self.ready or len(self.path_points) < 2:
            self.get_logger().warn("No hay ruta válida todavía.")
            return

        if self._executing:
            self.get_logger().warn("Ya hay una ejecución en curso, ignorando start.")
            return

        self.get_logger().info("Iniciando ejecución de ruta...")
        self._launch_execution()

    # ------------------------------------------------------------------
    # Control de ejecución en thread
    # ------------------------------------------------------------------

    def _launch_execution(self):
        self._abort = False
        self._executing = True
        t = threading.Thread(target=self._execute_thread, daemon=True)
        t.start()

    def _abort_execution(self):
        self._abort = True
        # Dar tiempo al thread para salir del sleep
        time.sleep(0.05)

    def _execute_thread(self):
        try:
            with self._lock:
                points = list(self.path_points)
            self._execute_path(points)
        finally:
            self._executing = False

    # ------------------------------------------------------------------
    # Comunicación UDP
    # ------------------------------------------------------------------

    def send_segment(self, command, distance):
        """Envía un segmento y espera el tiempo estimado de recorrido."""
        if distance < self.min_segment:
            return

        message = f"{command},{distance:.3f}"
        self.sock.sendto(
            message.encode("utf-8"),
            (self.raspberry_ip, self.raspberry_port)
        )
        self.get_logger().info(f"  → Enviado: {message}")

        wait_time = (distance / self.speed_estimate) + 0.8

        # Espera interrumpible por abort
        elapsed = 0.0
        step = 0.05
        while elapsed < wait_time:
            if self._abort:
                return
            time.sleep(step)
            elapsed += step

    def send_stop(self):
        self.sock.sendto(
            "S,0".encode("utf-8"),
            (self.raspberry_ip, self.raspberry_port)
        )
        self.get_logger().info("  → Enviado: S,0")

    # ------------------------------------------------------------------
    # Ejecución del path
    # ------------------------------------------------------------------

    def _execute_path(self, points):
        """
        Recorre el path ortogonal acumulando desplazamientos en X e Y por separado.
        Envía primero el movimiento en Y (F/B) y luego en X (R/L) al cambiar de dirección,
        exactamente como el robot físico los necesita.
        """
        accumulated_x = 0.0
        accumulated_y = 0.0

        for i in range(len(points) - 1):
            if self._abort:
                self.get_logger().info("Ejecución abortada.")
                self.send_stop()
                return

            x1, y1 = points[i]
            x2, y2 = points[i + 1]

            dx = round(x2 - x1, 6)
            dy = round(y2 - y1, 6)

            # Si el segmento es diagonal (no debería ocurrir con A* ortogonal),
            # lo descomponemos en Y primero, luego X.
            if abs(dy) > 1e-4 and abs(dx) > 1e-4:
                accumulated_y += dy
                self._flush_y(accumulated_y)
                accumulated_y = 0.0
                accumulated_x += dx
                self._flush_x(accumulated_x)
                accumulated_x = 0.0
                continue

            accumulated_x += dx
            accumulated_y += dy

            # Enviar acumulados cuando superen el mínimo
            if abs(accumulated_y) >= self.min_segment:
                self._flush_y(accumulated_y)
                accumulated_y = 0.0
                if self._abort:
                    self.send_stop()
                    return

            if abs(accumulated_x) >= self.min_segment:
                self._flush_x(accumulated_x)
                accumulated_x = 0.0
                if self._abort:
                    self.send_stop()
                    return

        # Residuos al final
        if not self._abort and abs(accumulated_y) >= self.min_segment:
            self._flush_y(accumulated_y)
        if not self._abort and abs(accumulated_x) >= self.min_segment:
            self._flush_x(accumulated_x)

        if not self._abort:
            self.send_stop()
            self.get_logger().info("✓ Ruta completada.")

    def _flush_y(self, accumulated_y):
        if abs(accumulated_y) < self.min_segment:
            return
        if accumulated_y > 0:
            self.send_segment("F", abs(accumulated_y))
        else:
            self.send_segment("B", abs(accumulated_y))

    def _flush_x(self, accumulated_x):
        if abs(accumulated_x) < self.min_segment:
            return
        if accumulated_x > 0:
            self.send_segment("R", abs(accumulated_x))
        else:
            self.send_segment("L", abs(accumulated_x))


def main(args=None):
    rclpy.init(args=args)
    node = PathToSegmentsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
