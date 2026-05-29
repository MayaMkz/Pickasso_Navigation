¡Tienes toda la razón! Es un excelente detalle. Lo que está pasando es que el algoritmo sabía que no podía atravesar el obstáculo, pero nadie le dijo explícitamente que "las paredes" del mapa también son un límite físico para el chasis.

Por otro lado, el problema de que ignore el segundo obstáculo está directamente ligado a que sigues usando el `path_to_segments_node`. Como te comenté, ese programa usa la instrucción `time.sleep()`. Cuando el programa entra en "sleep" para esperar a que el carrito termine de avanzar un segmento, **el nodo se congela por completo (se vuelve sordo y ciego)**. Por lo tanto, aunque el cerebro (`planner_node`) calcule una ruta nueva para evadir el segundo obstáculo, tu carrito no lo escucha hasta que termina su movimiento actual.

Además, cambiar esto es vital para evitar los jalones bruscos y proteger la estabilidad de tu brazo xArm5.

Aquí tienes la solución definitiva para ambos problemas y cómo adaptar tus terminales.

### 1. Limitar el área de trabajo (Modificación a `planner_node.py`)

Vamos a crear un "muro invisible" alrededor de todo el mapa. Reemplaza **todo** el contenido de tu archivo `planner_node.py` por este. He añadido un bloque en la función `create_grid` que "infla" los bordes del mapa usando las dimensiones exactas de tu robot, garantizando que nunca intente salirse:

```python
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

        self.width_cells = int(AREA_WIDTH / RESOLUTION)
        self.height_cells = int(AREA_HEIGHT / RESOLUTION)
        self.grid = self.create_grid()
        self.current_pose = None
        self.goal_pose = None

        self.get_logger().info("Planner Node Started")

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, msg):
        self.goal_pose = msg
        if self.current_pose is None:
            self.get_logger().warn("Cannot plan: /amr_pose not received yet")
            return
        self.plan_and_publish()
        
    def click_callback(self, msg):
        ox, oy = msg.point.x, msg.point.y
        self.dynamic_obstacles.append((ox, oy))
        self.get_logger().info(f"Obstáculo logístico en X:{ox:.2f}, Y:{oy:.2f}.")

        self.grid = self.create_grid()

        for dox, doy in self.dynamic_obstacles:
            sx = 0.30 + ROBOT_LENGTH + 2.0 * SAFETY_MARGIN
            sy = 0.30 + ROBOT_WIDTH + 2.0 * SAFETY_MARGIN
            min_x = int((dox - sx / 2.0) / RESOLUTION)
            max_x = int((dox + sx / 2.0) / RESOLUTION)
            min_y = int((doy - sy / 2.0) / RESOLUTION)
            max_y = int((doy + sy / 2.0) / RESOLUTION)

            for y in range(max(0, min_y), min(self.height_cells, max_y)):
                for x in range(max(0, min_x), min(self.width_cells, max_x)):
                    if 0 <= y < self.height_cells and 0 <= x < self.width_cells:
                        self.grid[y][x] = 1

        if self.goal_pose is not None and self.current_pose is not None:
            self.plan_and_publish()

    def world_to_grid(self, x, y):
        return int(x / RESOLUTION), int(y / RESOLUTION)

    def grid_to_world(self, gx, gy):
        return gx * RESOLUTION, gy * RESOLUTION

    def create_grid(self):
        grid = [[0 for _ in range(self.width_cells)] for _ in range(self.height_cells)]

        # --- NUEVO: MUROS INVISIBLES EN LOS BORDES ---
        margin_x_cells = int((ROBOT_LENGTH / 2.0 + SAFETY_MARGIN) / RESOLUTION)
        margin_y_cells = int((ROBOT_WIDTH / 2.0 + SAFETY_MARGIN) / RESOLUTION)

        for y in range(self.height_cells):
            for x in range(self.width_cells):
                if (x <= margin_x_cells or x >= self.width_cells - margin_x_cells or
                    y <= margin_y_cells or y >= self.height_cells - margin_y_cells):
                    grid[y][x] = 1
        # ---------------------------------------------

        for station in STATIONS:
            cx, cy = station["center"]
            sx, sy = station["size"]
            margin = TABLE_SAFETY_MARGIN if station["name"] == "table" else SAFETY_MARGIN

            sx = sx + ROBOT_LENGTH + 2.0 * margin
            sy = sy + ROBOT_WIDTH + 2.0 * margin
            min_x = int((cx - sx / 2.0) / RESOLUTION)
            max_x = int((cx + sx / 2.0) / RESOLUTION)
            min_y = int((cy - sy / 2.0) / RESOLUTION)
            max_y = int((cy + sy / 2.0) / RESOLUTION)

            for y in range(max(0, min_y), min(self.height_cells, max_y)):
                for x in range(max(0, min_x), min(self.width_cells, max_x)):
                    if 0 <= y < self.height_cells and 0 <= x < self.width_cells:
                        grid[y][x] = 1
        return grid

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def astar(self, start, goal):
        neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                if nx < 0 or ny < 0 or nx >= self.width_cells or ny >= self.height_cells:
                    continue
                if self.grid[ny][nx] == 1:
                    continue

                neighbor = (nx, ny)
                move_cost = math.sqrt(dx ** 2 + dy ** 2)
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
        return []

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def is_valid_cell(self, cell):
        gx, gy = cell
        if gx < 0 or gy < 0 or gx >= self.width_cells or gy >= self.height_cells:
            return False
        return self.grid[gy][gx] == 0

    def plan_and_publish(self):
        start = self.world_to_grid(self.current_pose.x, self.current_pose.y)
        goal = self.world_to_grid(self.goal_pose.x, self.goal_pose.y)

        if not self.is_valid_cell(start):
            self.get_logger().warn("Start position is inside inflated obstacle")
            return
        if not self.is_valid_cell(goal):
            self.get_logger().warn("Goal position is inside inflated obstacle")
            return

        grid_path = self.astar(start, goal)
        if len(grid_path) == 0:
            self.get_logger().warn("No path found")
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for gx, gy in grid_path:
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


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 2. Tus nuevos comandos (Despídete del `path_to_segments`)

Para solucionar definitivamente lo del segundo obstáculo y los jalones de los motores, vamos a reemplazar los comandos de tu Terminal 2 y 3. Al usar el controlador continuo, el Pickasso no entrará en estado de `sleep`, por lo que **escuchará y evadirá todos los obstáculos que agregues dinámicamente**, uno tras otro, sin detenerse.

Asegúrate de compilar primero con `colcon build` estando en la carpeta `~/x_arm`.

**Terminal 1 (Tu launch habitual - Visualización y simulador):**

```bash
cd ~/x_arm/
source install/setup.bash
ros2 launch pickasso_amr_2d simulation.launch.py

```

**Terminal 2 (El nuevo conductor, que ajusta en tiempo real):**

```bash
cd ~/x_arm/
source install/setup.bash
ros2 run pickasso_amr_2d path_follower_node

```

*(Ojo: Ya no usamos el de segments)*.

**Terminal 3 (El puente hacia los motores físicos del carrito):**

```bash
cd ~/x_arm/
source install/setup.bash
ros2 run pickasso_amr_2d cmd_vel_udp_bridge_node

```

**Terminal 4 (Para detonar el viaje):**
Aquí es donde usarás el tópico o el nodo de prueba. Puedes usar tu comando pub de ROS o el script de prueba que tienes diseñado para que publique el destino automáticamente:

```bash
cd ~/x_arm/
source install/setup.bash
ros2 run pickasso_amr_2d test_goal_node

```

¡Haz la prueba colocando 3 o 4 puntos rápidos con la herramienta "Publish Point" en RViz mientras el robot avanza! Verás cómo la línea verde de la trayectoria serpentea para evadirlos sin tocar jamás los bordes de la cuadrícula.
