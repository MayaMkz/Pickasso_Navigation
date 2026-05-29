import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose2D, PointStamped
from .config import *
import heapq
import math


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.goal_sub = self.create_subscription(Pose2D, '/goal_pose', self.goal_callback, 10)
        self.pose_sub = self.create_subscription(Pose2D, '/amr_pose', self.pose_callback, 10)

        self.dynamic_obstacles = []
        self.click_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.click_callback, 10
        )

        # Suscripción al tópico de obstáculos ArUco (publicado por aruco_obstacle_node)
        self.aruco_sub = self.create_subscription(
            PointStamped, '/aruco_obstacle', self.aruco_callback, 10
        )

        self.width_cells = int(AREA_WIDTH / RESOLUTION)
        self.height_cells = int(AREA_HEIGHT / RESOLUTION)
        self.grid = self.create_grid()

        self.current_pose = None
        self.goal_pose = None

        self.get_logger().info("Planner Node Started (Ortogonal + Ruta Alterna)")

    # ------------------------------------------------------------------
    # Callbacks de pose y goal
    # ------------------------------------------------------------------

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, msg):
        self.goal_pose = msg
        if self.current_pose is None:
            self.get_logger().warn("Cannot plan: /amr_pose not received yet")
            return
        self.plan_and_publish()

    # ------------------------------------------------------------------
    # Callbacks de obstáculos
    # ------------------------------------------------------------------

    def _register_obstacle(self, ox, oy, label=""):
        """Agrega un obstáculo al grid con el margen propio del robot."""
        self.dynamic_obstacles.append((ox, oy))
        self.get_logger().info(f"Obstáculo {label}en X:{ox:.2f}, Y:{oy:.2f}")

        # Reconstruir grid completo + todos los obstáculos dinámicos
        self.grid = self.create_grid()
        for dox, doy in self.dynamic_obstacles:
            sx = 0.10 + ROBOT_LENGTH + 2.0 * SAFETY_MARGIN
            sy = 0.10 + ROBOT_WIDTH + 2.0 * SAFETY_MARGIN
            self._inflate_obstacle(dox, doy, sx, sy)

        if self.goal_pose is not None and self.current_pose is not None:
            self.plan_and_publish()

    def click_callback(self, msg):
        self._register_obstacle(msg.point.x, msg.point.y, label="(RViz clic) ")

    def aruco_callback(self, msg):
        """Recibe obstáculos detectados por la cámara (ArUco ID 10)."""
        self._register_obstacle(msg.point.x, msg.point.y, label="(ArUco ID10) ")

    # ------------------------------------------------------------------
    # Grid
    # ------------------------------------------------------------------

    def _inflate_obstacle(self, cx, cy, sx, sy):
        min_x = int((cx - sx / 2.0) / RESOLUTION)
        max_x = int((cx + sx / 2.0) / RESOLUTION)
        min_y = int((cy - sy / 2.0) / RESOLUTION)
        max_y = int((cy + sy / 2.0) / RESOLUTION)
        for y in range(max(0, min_y), min(self.height_cells, max_y + 1)):
            for x in range(max(0, min_x), min(self.width_cells, max_x + 1)):
                self.grid[y][x] = 1

    def create_grid(self):
        grid = [[0] * self.width_cells for _ in range(self.height_cells)]

        # Borde exterior
        for y in range(self.height_cells):
            for x in range(self.width_cells):
                if x == 0 or x == self.width_cells - 1 or y == 0 or y == self.height_cells - 1:
                    grid[y][x] = 1

        # Estaciones y mesa
        for station in STATIONS:
            cx, cy = station["center"]
            sx, sy = station["size"]
            margin = TABLE_SAFETY_MARGIN if station["name"] == "table" else SAFETY_MARGIN
            sx_inf = sx + ROBOT_LENGTH + 2.0 * margin
            sy_inf = sy + ROBOT_WIDTH + 2.0 * margin
            min_x = int((cx - sx_inf / 2.0) / RESOLUTION)
            max_x = int((cx + sx_inf / 2.0) / RESOLUTION)
            min_y = int((cy - sy_inf / 2.0) / RESOLUTION)
            max_y = int((cy + sy_inf / 2.0) / RESOLUTION)
            for y in range(max(0, min_y), min(self.height_cells, max_y + 1)):
                for x in range(max(0, min_x), min(self.width_cells, max_x + 1)):
                    grid[y][x] = 1

        return grid

    # ------------------------------------------------------------------
    # A* ORTOGONAL (sin diagonales)
    # El robot holonómico se mueve en +X, -X, +Y, -Y solamente.
    # Esto garantiza que path_to_segments_node genere comandos F/B/R/L puros,
    # sin acumulaciones diagonales que confundan al robot físico.
    # ------------------------------------------------------------------

    def heuristic(self, a, b):
        # Manhattan es la heurística perfecta para movimiento ortogonal
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def astar_ortogonal(self, start, goal):
        """A* con solo 4 vecinos ortogonales."""
        neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                if nx < 0 or ny < 0 or nx >= self.width_cells or ny >= self.height_cells:
                    continue
                if self.grid[ny][nx] == 1:
                    continue

                neighbor = (nx, ny)
                tentative_g = g_score[current] + 1  # coste uniforme ortogonal

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return []  # Sin ruta

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    # ------------------------------------------------------------------
    # Búsqueda de celda libre más cercana (para start/goal bloqueados)
    # ------------------------------------------------------------------

    def find_nearest_free_cell(self, gx, gy, max_radius=15):
        """BFS desde (gx, gy) para encontrar la celda libre más cercana."""
        visited = set()
        queue = [(gx, gy)]
        visited.add((gx, gy))

        while queue:
            cx, cy = queue.pop(0)
            if self.grid[cy][cx] == 0:
                return (cx, cy)
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) not in visited:
                    if 0 <= nx < self.width_cells and 0 <= ny < self.height_cells:
                        if abs(nx - gx) <= max_radius and abs(ny - gy) <= max_radius:
                            visited.add((nx, ny))
                            queue.append((nx, ny))
        return None

    # ------------------------------------------------------------------
    # Coordinación de planificación
    # ------------------------------------------------------------------

    def world_to_grid(self, x, y):
        return int(x / RESOLUTION), int(y / RESOLUTION)

    def grid_to_world(self, gx, gy):
        return gx * RESOLUTION, gy * RESOLUTION

    def is_valid_cell(self, gx, gy):
        if gx < 0 or gy < 0 or gx >= self.width_cells or gy >= self.height_cells:
            return False
        return self.grid[gy][gx] == 0

    def plan_and_publish(self):
        sx, sy = self.world_to_grid(self.current_pose.x, self.current_pose.y)
        gx, gy = self.world_to_grid(self.goal_pose.x, self.goal_pose.y)

        # --- Reubicar start si está en obstáculo ---
        if not self.is_valid_cell(sx, sy):
            self.get_logger().warn("Start dentro de obstáculo, buscando celda libre cercana...")
            free = self.find_nearest_free_cell(sx, sy)
            if free is None:
                self.get_logger().error("No se encontró celda libre cerca del start. Abortando.")
                self._publish_empty_path()
                return
            sx, sy = free
            self.get_logger().info(f"Start reubicado a celda libre ({sx},{sy})")

        # --- Reubicar goal si está en obstáculo ---
        if not self.is_valid_cell(gx, gy):
            self.get_logger().warn("Goal dentro de obstáculo, buscando celda libre cercana...")
            free = self.find_nearest_free_cell(gx, gy)
            if free is None:
                self.get_logger().error("No se encontró celda libre cerca del goal. Abortando.")
                self._publish_empty_path()
                return
            gx, gy = free
            self.get_logger().info(f"Goal reubicado a celda libre ({gx},{gy})")

        # --- Planificar con A* ortogonal ---
        grid_path = self.astar_ortogonal((sx, sy), (gx, gy))

        if len(grid_path) == 0:
            self.get_logger().warn(
                "¡Ruta completamente bloqueada! Intentando ruta alterna con margen reducido..."
            )
            # Intento 2: reducir el margen de seguridad de obstáculos dinámicos temporalmente
            grid_path = self._try_reduced_margin_plan((sx, sy), (gx, gy))

        if len(grid_path) == 0:
            self.get_logger().error(
                "No se encontró ninguna ruta válida. Cancelando misión."
            )
            self._publish_empty_path()
            return

        self._publish_path(grid_path)

    def _try_reduced_margin_plan(self, start, goal):
        """Replanifica con margen reducido al 50% en obstáculos dinámicos."""
        backup_grid = [row[:] for row in self.grid]  # copia

        # Reconstruir grid con margen reducido solo para dinámicos
        self.grid = self.create_grid()
        for dox, doy in self.dynamic_obstacles:
            sx = 0.10 + ROBOT_LENGTH * 0.5  # margen reducido
            sy = 0.10 + ROBOT_WIDTH * 0.5
            self._inflate_obstacle(dox, doy, sx, sy)

        grid_path = self.astar_ortogonal(start, goal)

        # Restaurar grid original
        self.grid = backup_grid

        if len(grid_path) > 0:
            self.get_logger().info("Ruta alterna encontrada con margen reducido.")
        return grid_path

    def _publish_empty_path(self):
        empty = Path()
        empty.header.frame_id = "map"
        empty.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty)

    def _publish_path(self, grid_path):
        """
        Comprime el path ortogonal: solo guarda los vértices donde cambia la dirección.
        Esto reduce el número de puntos y hace más eficiente path_to_segments_node.
        """
        compressed = self._compress_ortogonal_path(grid_path)

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for gx, gy in compressed:
            x, y = self.grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f"Ruta publicada: {len(grid_path)} celdas → {len(compressed)} segmentos ortogonales"
        )

    def _compress_ortogonal_path(self, path):
        """
        Dado un path ortogonal celda por celda, devuelve solo los puntos
        de inicio, cada cambio de dirección y el punto final.
        """
        if len(path) <= 2:
            return path

        compressed = [path[0]]
        prev_dir = None

        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            curr_dir = (dx, dy)

            if curr_dir != prev_dir:
                # Cambio de dirección: guardar el punto anterior como vértice
                if i > 1:
                    compressed.append(path[i - 1])
                prev_dir = curr_dir

        compressed.append(path[-1])
        return compressed


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
