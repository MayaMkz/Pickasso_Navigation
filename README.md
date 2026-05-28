# Pickasso_Navigation
¡Manos a la obra! Para implementar esta funcionalidad de manera limpia y sin tener que escribir comandos largos en la terminal cada vez, vamos a aprovechar una herramienta que ya viene integrada en RViz2: el botón **"Publish Point"**.

Al usar este botón y hacer clic en el mapa, RViz publica automáticamente las coordenadas $X$ e $Y$ en un tópico llamado `/clicked_point`. Vamos a modificar tu código para que tanto el mapa visual como el cerebro del Pickasso escuchen ese clic, coloquen un obstáculo (imaginemos una caja de $30 \times 30$ cm) y recalculen la ruta al instante.

Esto es ideal para probar la evasión dinámica sin arriesgar la estructura física ni el brazo xArm5 en pruebas iniciales.

---

### **1. Modificaciones en el Código**

Abre tus archivos y añade los siguientes bloques de código.

#### **A. Modificar `map_node.py` (Para la visualización)**

Necesitamos que el mapa escuche los clics y pinte las celdas de negro.

1. **En las importaciones (arriba del todo), añade:**
```python
from geometry_msgs.msg import PointStamped

```


2. **Dentro del `__init__`, debajo de tus publicadores, añade:**
```python
self.dynamic_obstacles = []
self.click_sub = self.create_subscription(
    PointStamped,
    '/clicked_point',
    self.click_callback,
    10
)

```


3. **Crea el nuevo método para guardar el clic:**
```python
def click_callback(self, msg):
    self.dynamic_obstacles.append((msg.point.x, msg.point.y))
    self.get_logger().info(f"Obstáculo visual en X:{msg.point.x:.2f}, Y:{msg.point.y:.2f}")

```


4. **Dentro de `publish_map(self)`, justo antes de la línea `grid.data = data`, añade este bloque:**
```python
    # Dibujar obstáculos dinámicos
    for ox, oy in self.dynamic_obstacles:
        sx, sy = 0.30, 0.30 # Asumimos cajas de 30x30 cm
        min_x = int((ox - sx / 2.0) / RESOLUTION)
        max_x = int((ox + sx / 2.0) / RESOLUTION)
        min_y = int((oy - sy / 2.0) / RESOLUTION)
        max_y = int((oy + sy / 2.0) / RESOLUTION)

        for y in range(max(0, min_y), min(self.height_cells, max_y)):
            for x in range(max(0, min_x), min(self.width_cells, max_x)):
                data[y * self.width_cells + x] = 100

```



#### **B. Modificar `planner_node.py` (El Cerebro / A*)**

El planificador también debe enterarse del obstáculo para inflarlo con su margen de seguridad y buscar una nueva ruta.

1. **En las importaciones, añade:**
```python
from geometry_msgs.msg import PointStamped

```


2. **Dentro del `__init__`, añade:**
```python
self.dynamic_obstacles = []
self.click_sub = self.create_subscription(
    PointStamped,
    '/clicked_point',
    self.click_callback,
    10
)

```


3. **Crea el método de respuesta al clic:**
```python
def click_callback(self, msg):
    ox, oy = msg.point.x, msg.point.y
    self.dynamic_obstacles.append((ox, oy))
    self.get_logger().info("Actualizando malla de colisión...")

    # 1. Reconstruir la cuadrícula limpia
    self.grid = self.create_grid()

    # 2. Añadir e inflar los obstáculos dinámicos
    for dox, doy in self.dynamic_obstacles:
        # Tamaño base (0.30) + el tamaño del robot para evitar rozes
        sx = 0.30 + ROBOT_LENGTH + 2.0 * SAFETY_MARGIN
        sy = 0.30 + ROBOT_WIDTH + 2.0 * SAFETY_MARGIN

        min_x = int((dox - sx / 2.0) / RESOLUTION)
        max_x = int((dox + sx / 2.0) / RESOLUTION)
        min_y = int((doy - sy / 2.0) / RESOLUTION)
        max_y = int((doy + sy / 2.0) / RESOLUTION)

        for y in range(max(0, min_y), min(self.height_cells, max_y)):
            for x in range(max(0, min_x), min(self.width_cells, max_x)):
                # Evitar desbordamientos en la lista
                if 0 <= y < self.height_cells and 0 <= x < self.width_cells:
                    self.grid[y][x] = 1

    # 3. Forzar replanificación automática si ya estábamos en movimiento
    if self.goal_pose is not None and self.current_pose is not None:
        self.plan_and_publish()

```



---

### **2. Comandos de Ejecución (Arquitectura Completa)**

Una vez que guardes los cambios y compiles tu espacio de trabajo (`colcon build`), abre **4 terminales**. Asegúrate de hacer el `source install/setup.bash` en todas.

Para que los movimientos sean suaves y seguros, usaremos el seguidor de trayectorias continuo en lugar de los movimientos por segmentos.

**Terminal 1: El Core Logístico**
Aquí arrancamos la inteligencia del sistema y estimamos la posición (con hardware real, `robot_sim_node` sería reemplazado por la odometría de tus encoders).

```bash
ros2 run amr_navigation map_node &
ros2 run amr_navigation robot_sim_node &
ros2 run amr_navigation planner_node &
ros2 run amr_navigation path_follower_node

```

**Terminal 2: El Puente Físico (Conexión al AMR)**
Esto envía los comandos de velocidad suave (control continuo) directo a la Raspberry Pi por red.

```bash
ros2 run amr_navigation cmd_vel_udp_bridge_node

```

**Terminal 3: Entorno Gráfico**
Lanzamos los marcadores estéticos y abrimos la interfaz de usuario.

```bash
ros2 run amr_navigation robot_marker_node &
ros2 run amr_navigation station_marker_node &
rviz2

```

**Terminal 4: Despacho de Órdenes**
Para enviar al robot a trabajar.

```bash
ros2 run amr_navigation test_goal_node

```

---

### **3. Cómo probarlo en RViz2**

1. Cuando RViz2 se abra, asegúrate de tener añadidos los visualizadores de **Map** (suscrito a `/map`), **MarkerArray** (suscrito a `/station_markers`), **Marker** (suscrito a `/amr_marker`) y **Path** (suscrito a `/planned_path`).
2. Ve a la Terminal 4 y corre el nodo de prueba. Verás la línea trazada hacia la estación de "Pickup" y tu robot (el prisma azul) comenzará a moverse, enviando al mismo tiempo datos físicos a la Raspberry Pi.
3. En la barra superior de RViz2, haz clic en la herramienta **"Publish Point"**.
4. Haz clic en algún lugar del mapa justo en frente de la línea de la trayectoria del robot.
5. Verás aparecer un bloque negro (el obstáculo) y el planificador trazará inmediatamente una nueva curva para rodearlo, haciendo que tu robot físico también gire suavemente.

Con esta implementación, tienes una base muy sólida para la navegación logística. ¿Tienes en mente integrar la cámara cenital para que alimente automáticamente esta lista de obstáculos usando YOLOv8 en un futuro, o por ahora la detección de entorno se mantendrá manual?
