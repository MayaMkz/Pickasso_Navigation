import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import math
import threading
import time

# ================================================================
#  CONFIGURACIÓN DE LLEGADA
# ================================================================
OFFSET_LLEGADA = {
    'pick':    0.20,   # 20cm antes del ArUco de pick
    'classif': 0.30,   # 30cm antes del ArUco de classif
    'home':    0.0,    # home llega exacto
}

# Coordenada fija de seguridad para Home (por si no hay ArUco ahí)
HOME_ESTATICO = {'x': 2.55, 'y': 0.50, 'yaw': 0.0}
YAW_FIJO = 1.57

def euler_a_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

class NodoComandante(Node):
    def __init__(self):
        super().__init__('nodo_comandante_logistico')
        self.pub_bandera = self.create_publisher(String, '/bandera_estacion', 10)
        self.sub_estado = self.create_subscription(String, '/estado_brazo', self.estado_cb, 10)
        self.estado_brazo = None
        
        # Inicializamos TF Listener aquí para que use el reloj del nodo
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def estado_cb(self, msg):
        self.estado_brazo = msg.data.strip()

    def publicar_bandera(self, estacion):
        msg = String()
        msg.data = estacion
        self.pub_bandera.publish(msg)
        self.get_logger().info(f"BANDERA ENVIADA AL BRAZO: '{estacion.upper()}'")

    def esperar_confirmacion_brazo(self, estado_esperado):
        self.estado_brazo = None
        while rclpy.ok() and self.estado_brazo != estado_esperado:
            time.sleep(0.1)
        self.estado_brazo = None 

    def leer_tf_estacion(self, nombre):
        if nombre == 'home':
            return HOME_ESTATICO['x'], HOME_ESTATICO['y'], HOME_ESTATICO['yaw']
            
        frame = f'estacion_{nombre}'
        try:
            trans = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time(), timeout=Duration(seconds=1.5))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return x, y, yaw
        except Exception:
            return None, None, None

def aplicar_offset(x_aruco, y_aruco, yaw_aruco, offset_m):
    dx = -math.sin(yaw_aruco) * offset_m
    dy =  math.cos(yaw_aruco) * offset_m
    return x_aruco + dx, y_aruco + dy

def navegar_a_dinamico(navigator, nodo, destino):
    print(f"\n[Nav2] Calculando ruta hacia '{destino.upper()}'...")
    
    # 1. Leer ArUco
    x_aruco, y_aruco, yaw_aruco = nodo.leer_tf_estacion(destino)
    
    if x_aruco is None:
        print(f"ERROR: No se detecta el ArUco de la estación '{destino}'.")
        return False

    # 2. Aplicar Offset
    offset = OFFSET_LLEGADA.get(destino, 0.0)
    if destino != 'home' and offset > 0.0 and yaw_aruco is not None:
        x_goal, y_goal = aplicar_offset(x_aruco, y_aruco, yaw_aruco, offset)
        print(f" ArUco({x_aruco:.2f},{y_aruco:.2f}) → Llegada calculada({x_goal:.2f},{y_goal:.2f}) [Offset: {offset}m]")
    else:
        x_goal, y_goal = x_aruco, y_aruco
        print(f" Coordenada final ({x_goal:.2f},{y_goal:.2f})")

    # 3. Crear Pose y Navegar
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(x_goal)
    goal_pose.pose.position.y = float(y_goal)
    
    yaw_final = YAW_FIJO if destino != 'home' else HOME_ESTATICO['yaw']
    qx, qy, qz, qw = euler_a_quaternion(yaw_final)
    goal_pose.pose.orientation.x = qx
    goal_pose.pose.orientation.y = qy
    goal_pose.pose.orientation.z = qz
    goal_pose.pose.orientation.w = qw

    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f" Viajando... Distancia restante: {feedback.distance_remaining:.2f} m", end='\r')

    resultado = navigator.getResult()
    print("\n")
    if resultado == TaskResult.SUCCEEDED:
        print(f"[Nav2] ¡Pickasso llegó a {destino.upper()}!")
        return True
    else:
        print(f"[Nav2] ERROR en la ruta hacia {destino.upper()}.")
        return False

def main():
    rclpy.init()
    
    nodo_comunicaciones = NodoComandante()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nodo_comunicaciones)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    navigator = BasicNavigator()

    print("\n" + "═"*50)
    print(" SISTEMA MAESTRO DE LOGÍSTICA PICKASSO")
    print("═"*50)
    
    while rclpy.ok():
        input("\n>> PRESIONA ENTER para arrancar un nuevo Lote de Producción... ")
        print("\nNOTA: Ve a la terminal del Brazo (Pick & Place) y escribe el color a recolectar primero.")
        input(">> Presiona ENTER cuando el brazo ya esté esperando el carrito... ")

        # 1. Viaje a Pick
        if navegar_a_dinamico(navigator, nodo_comunicaciones, 'pick'):
            nodo_comunicaciones.publicar_bandera('pick')
            
            print("[Comandante] Esperando a que el xArm5 termine de cargar el carro...")
            nodo_comunicaciones.esperar_confirmacion_brazo('pick_completado')
            print("[Comandante] ¡Carga confirmada por el brazo!")

            # 2. VIA-POINT: Viaje a Home antes de ir a Classif
            print("\n[Comandante] Iniciando maniobra de seguridad (Via-Point a Home)...")
            if navegar_a_dinamico(navigator, nodo_comunicaciones, 'home'):
                
                # 3. Viaje a Place (Classif)
                if navegar_a_dinamico(navigator, nodo_comunicaciones, 'classif'):
                    nodo_comunicaciones.publicar_bandera('place')
                    
                    print("[Comandante] Esperando a que el xArm5 termine de descargar...")
                    nodo_comunicaciones.esperar_confirmacion_brazo('place_completado')
                    print("[Comandante] ¡Descarga confirmada por el brazo!")

                    # 4. Retorno a Home final
                    navegar_a_dinamico(navigator, nodo_comunicaciones, 'home')
                    print("[Comandante] Lote finalizado con éxito. Pickasso en reposo.")

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
