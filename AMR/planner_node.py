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

