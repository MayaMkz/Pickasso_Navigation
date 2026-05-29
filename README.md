¡Entendido! Vamos a dejar listos los programas con los cambios integrados para que todo funcione a la perfección.

Antes de pasar al código, quiero hacer una pequeña pero importante aclaración técnica sobre tu archivo `setup.py` para evitar confusiones. Mencionas que ahí tienes definido que se abran ciertos comandos para evitar ingresar en varias terminales. En realidad, el bloque `console_scripts` del `setup.py` sirve para "registrar" o dar de alta los nodos en ROS 2, permitiéndote ejecutarlos con el comando `ros2 run pickasso_amr_2d <nombre_del_nodo>`. Sin embargo, el `setup.py` por sí solo no ejecuta múltiples nodos al mismo tiempo. Para abrir todo con un solo comando y evitar las múltiples terminales, se utiliza un archivo "Launch" (que veo que tienes configurado en la carpeta `launch/` de tu paquete).

Aquí tienes los códigos completos ya actualizados, y más abajo los pasos exactos usando el nombre correcto de tu paquete (`pickasso_amr_2d`).

---

### **1. Código Completo: `map_node.py**`

*Reemplaza todo el contenido de tu `map_node.py` actual con este:*

```python
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PointStamped

from .config import *


class MapNode(Node):

    def __init__(self):
        super().__init__('map_node')

        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )

        # Suscripción para recibir los clics desde RViz
        self.dynamic_obstacles = []
        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.click_callback,
            10
        )

        self.width_cells = int(AREA_WIDTH / RESOLUTION)
        self.height_cells = int(AREA_HEIGHT / RESOLUTION)

        self.timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info("Map Node Started. Listening for /clicked_point...")

    def click_callback(self, msg):
        # Guardar las coordenadas del clic de RViz
        self.dynamic_obstacles.append((msg.point.x, msg.point.y))
        self.get_logger().info(f"Obstáculo visual añadido en X:{msg.point.x:.2f}, Y:{msg.point.y:.2f}")
        # Forzar una actualización inmediata del mapa
        self.publish_map()

    def publish_map(self):
        grid = OccupancyGrid()

        grid.header.frame_id = "map"
        grid.header.stamp = self.get_clock().now().to_msg()

        grid.info.resolution = RESOLUTION
        grid.info.width = self.width_cells
        grid.info.height = self.height_cells
        grid.info.origin = Pose()

        data = [0] * (self.width_cells * self.height_cells)

        # 1. Dibujar estaciones estáticas
        for station in STATIONS:
            cx, cy = station["center"]
            sx, sy = station["size"]

            min_x = int((cx - sx / 2.0) / RESOLUTION)
            max_x = int((cx + sx / 2.0) / RESOLUTION)
            min_y = int((cy - sy / 2.0) / RESOLUTION)
            max_y = int((cy + sy / 2.0) / RESOLUTION)

            for y in range(max(0, min_y), min(self.height_cells, max_y)):
                for x in range(max(0, min_x), min(self.width_cells, max_x)):
                    data[y * self.width_cells + x] = 100

        # 2. Dibujar obstáculos dinámicos (clics)
        for ox, oy in self.dynamic_obstacles:
            sx, sy = 0.30, 0.30 # Simulamos cajas de 30x30 cm
            min_x = int((ox - sx / 2.0) / RESOLUTION)
            max_x = int((ox + sx / 2.0) / RESOLUTION)
            min_y = int((oy - sy / 2.0) / RESOLUTION)
            max_y = int((oy + sy / 2.0) / RESOLUTION)

            for y in range(max(0, min_y), min(self.height_cells, max_y)):
                for x in range(max(0, min_x), min(self.width_cells, max_x)):
                    # Evitar errores si se hace clic fuera del mapa
                    if 0 <= y < self.height_cells and 0 <= x < self.width_cells:
                        data[y * self.width_cells + x] = 100

        grid.data = data
        self.publisher.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = MapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```

---

### **2. Código Completo: `planner_node.py**`

*Reemplaza todo el contenido de tu `planner_node.py` actual con este:*

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
        
        # Suscripción para recibir los clics de RViz
        self.dynamic_obstacles = []
        self.click_sub = self.create_subscription(
            PointStamped, 
            '/clicked_point', 
            self.click_callback, 
            10
        )

        self.width_cells = int(AREA_WIDTH / RESOLUTION)
        self.height_cells = int(AREA_HEIGHT / RESOLUTION)

        self.grid = self.create_grid()

        self.current_pose = None
        self.goal_pose = None

        self.get_logger().info("Planner Node Started")
        self.get_logger().info("Waiting for /goal_pose and /clicked_point...")

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
        self.get_logger().info(f"Obstáculo logístico recibido en X:{ox:.2f}, Y:{oy:.2f}. Actualizando y replanificando...")

        # 1. Reconstruir la malla original limpia
        self.grid = self.create_grid()

        # 2. Añadir los obstáculos dinámicos con su respectivo inflado
        for dox, doy in self.dynamic_obstacles:
            # Tamaño base (0.30m) + dimensiones del robot y márgenes
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

        # 3. Si el robot estaba yendo a un destino, replanificar automáticamente
        if self.goal_pose is not None and self.current_pose is not None:
            self.plan_and_publish()

    def world_to_grid(self, x, y):
        return int(x / RESOLUTION), int(y / RESOLUTION)

    def grid_to_world(self, gx, gy):
        return gx * RESOLUTION, gy * RESOLUTION

    def create_grid(self):
        grid = [[0 for _ in range(self.width_cells)] for _ in range(self.height_cells)]

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
        self.get_logger().info(f"Path published with {len(path_msg.poses)} points")


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

---

### **3. Instrucciones de Ejecución Paso a Paso**

**Paso 1: Recompilar los cambios**
Como modificaste archivos `.py`, debes reconstruir tu espacio de trabajo. Ve a la carpeta raíz de tu workspace (ej. `~/ros2_ws`) y ejecuta:

```bash
colcon build --packages-select pickasso_amr_2d
source install/setup.bash

```

**Paso 2: Lanzar el sistema**
Como mencionamos, si no tienes un archivo `.launch.py` configurado todavía, tendrás que usar varias terminales. Abre cada una, recuerda ejecutar `source install/setup.bash` en todas, y usa los siguientes comandos con el nombre de tu paquete (`pickasso_amr_2d`):

* **Terminal 1 (Lógica, Navegación y Comunicación con el carrito físico):**
Puedes encadenarlos así para ejecutarlos en una sola terminal (aunque se mezclen los logs):
```bash
ros2 run pickasso_amr_2d map_node & ros2 run pickasso_amr_2d planner_node & ros2 run pickasso_amr_2d cmd_vel_udp_bridge_node & ros2 run pickasso_amr_2d trajectory_follower_node

```


*(Nota: Reemplacé el simulador por tu puente UDP y utilicé `trajectory_follower_node` que tienes en tu setup.py para tener un control continuo).*
* **Terminal 2 (Visualización):**
```bash
ros2 run pickasso_amr_2d robot_marker_node & ros2 run pickasso_amr_2d station_marker_node & rviz2

```


* **Terminal 3 (El Detonador de la Prueba):**
```bash
ros2 run pickasso_amr_2d test_goal_node

```



**Paso 3: Probar el obstáculo dinámico en RViz**

1. En RViz, asegúrate de tener las pantallas de **Map**, **Path** y **Markers** activas.
2. Ejecuta el comando en la Terminal 3 para que el robot trace su ruta y empiece a enviar comandos a la placa física.
3. En el menú superior de RViz, busca el botón **"Publish Point"**.
4. Haz clic directamente sobre la línea de la trayectoria verde en el mapa.
5. ¡Deberías ver aparecer un cuadro negro instantáneamente, la trayectoria actualizarse sola rodeándolo, y tu carrito físico realizar el ajuste en tiempo real!
