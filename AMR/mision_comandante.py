import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math
import threading
import time

def euler_a_quaternion(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw

class NodoComandante(Node):
    def __init__(self):
        super().__init__('nodo_comandante_logistico')
        self.pub_bandera = self.create_publisher(String, '/bandera_estacion', 10)
        self.sub_estado = self.create_subscription(String, '/estado_brazo', self.estado_cb, 10)
        self.estado_brazo = None

    def estado_cb(self, msg):
        self.estado_brazo = msg.data.strip()

    def publicar_bandera(self, estacion):
        msg = String()
        msg.data = estacion
        self.pub_bandera.publish(msg)
        self.get_logger().info(f"🚩 BANDERA ENVIADA AL BRAZO: '{estacion.upper()}'")

    def esperar_confirmacion_brazo(self, estado_esperado):
        self.estado_brazo = None
        while rclpy.ok() and self.estado_brazo != estado_esperado:
            time.sleep(0.1)
        self.estado_brazo = None # Reseteamos para la siguiente lectura

def navegar_a(navigator, destino, estaciones):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    goal_pose.pose.position.x = estaciones[destino]['x']
    goal_pose.pose.position.y = estaciones[destino]['y']
    
    qx, qy, qz, qw = euler_a_quaternion(estaciones[destino]['yaw'])
    goal_pose.pose.orientation.x = qx
    goal_pose.pose.orientation.y = qy
    goal_pose.pose.orientation.z = qz
    goal_pose.pose.orientation.w = qw

    print(f"\n🚚 [Nav2] Pickasso conduciendo hacia la estación: {destino.upper()}...")
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"   -> Distancia restante: {feedback.distance_remaining:.2f} m", end='\r')

    resultado = navigator.getResult()
    print("\n")
    if resultado == TaskResult.SUCCEEDED:
        print(f"✅ [Nav2] ¡Destino {destino.upper()} alcanzado!")
        return True
    else:
        print(f"❌ [Nav2] ERROR: No se pudo llegar a {destino.upper()}.")
        return False

def main():
    rclpy.init()
    
    # Iniciamos el nodo de comunicaciones en un hilo separado para que no bloquee a Nav2
    nodo_comunicaciones = NodoComandante()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nodo_comunicaciones)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    navigator = BasicNavigator()

    estaciones = {
        'home':    {'x': 2.55, 'y': 0.50, 'yaw': 0.0},
        'pick':    {'x': 2.45, 'y': 2.85, 'yaw': 1.57}, 
        'classif': {'x': 1.00, 'y': 0.50, 'yaw': 3.14}  
    }

    print("\n" + "═"*50)
    print(" 🧠 SISTEMA MAESTRO DE LOGÍSTICA PICKASSO")
    print("═"*50)
    
    while rclpy.ok():
        input("\n>> PRESIONA ENTER para arrancar un nuevo Lote de Producción... ")
        print("\n⏳ NOTA: Ve a la terminal del Brazo (Pick & Place) y escribe el color a recolectar primero.")
        input(">> Presiona ENTER cuando el brazo ya esté esperando el carrito... ")

        # 1. Viaje a Pick
        if navegar_a(navigator, 'pick', estaciones):
            # Avisarle al brazo que ya llegamos
            nodo_comunicaciones.publicar_bandera('pick')
            
            # Esperar a que el brazo termine su trabajo
            print("⏳ [Comandante] Esperando a que el xArm5 termine de cargar el carro...")
            nodo_comunicaciones.esperar_confirmacion_brazo('pick_completado')
            print("📦 [Comandante] ¡Carga confirmada por el brazo!")

            # 2. Viaje a Place (Classif)
            if navegar_a(navigator, 'classif', estaciones):
                nodo_comunicaciones.publicar_bandera('place')
                
                print("⏳ [Comandante] Esperando a que el xArm5 termine de descargar...")
                nodo_comunicaciones.esperar_confirmacion_brazo('place_completado')
                print("📦 [Comandante] ¡Descarga confirmada por el brazo!")

                # 3. Viaje a Home
                navegar_a(navigator, 'home', estaciones)
                print("🏁 [Comandante] Lote finalizado con éxito. Pickasso en reposo.")

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
