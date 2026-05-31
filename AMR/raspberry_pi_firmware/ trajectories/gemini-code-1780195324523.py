import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class PickassoBaseNode(Node):
    def __init__(self):
        super().__init__('pickasso_base_node')
        
        # --- PARÁMETROS FÍSICOS ---
        self.DIAMETRO_LLANTA_MM = 152.0
        self.CIRCUNFERENCIA_M = (self.DIAMETRO_LLANTA_MM * math.pi) / 1000.0
        self.RPM_MAX_FISICO = 100.0

        # --- INTENTO DE CONEXIÓN SERIAL ---
        # Si no tienes el carrito conectado hoy, no fallará, solo avisará.
        self.hardware_conectado = False
        try:
            self.esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
            self.esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.05)
            self.hardware_conectado = True
            self.get_logger().info("[OK] ESP32 Conectadas por USB.")
        except Exception as e:
            self.get_logger().warn(f"[OFFLINE MODE] No se detectaron las ESP32: {e}")
            self.get_logger().warn("El nodo correrá simulando los cálculos matemáticos.")

        # --- SUSCRIPCIÓN A NAV2 ---
        # Escuchamos las velocidades que calcula Nav2
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("Nodo Pickasso Base iniciado. Esperando comandos de velocidad...")

    def mandar_orden(self, placa, motor, direccion, rpm):
        comando = f"{motor},{direccion},{rpm:.1f}\n"
        if self.hardware_conectado:
            try:
                placa.write(comando.encode("utf-8"))
            except Exception as e:
                self.get_logger().error(f"Error escribiendo en serial: {e}")
        else:
            # En modo offline, simplemente imprimimos qué haría el robot
            pass 

    def ms_a_rpm(self, v_ms):
        return (v_ms / self.CIRCUNFERENCIA_M) * 60.0

    def cmd_vel_callback(self, msg):
        """
        Esta función se ejecuta cada vez que Nav2 publica un movimiento.
        """
        vx = msg.linear.x   # Velocidad frontal (m/s)
        vy = msg.linear.y   # Velocidad lateral (m/s)
        omega = msg.angular.z # Velocidad de rotación (rad/s)

        # Factor de escala para la rotación (ajustable cuando tengas el chasis)
        radio_giro_aprox = 0.15 
        omega_ms = omega * radio_giro_aprox

        # --- CINEMÁTICA MECANUM ---
        v_FL = vx + vy - omega_ms
        v_BL = vx - vy - omega_ms
        v_FR = vx - vy + omega_ms
        v_BR = vx + vy + omega_ms

        # Conversión a RPM
        rpm_FL = min(self.RPM_MAX_FISICO, abs(self.ms_a_rpm(v_FL)))
        rpm_BL = min(self.RPM_MAX_FISICO, abs(self.ms_a_rpm(v_BL)))
        rpm_FR = min(self.RPM_MAX_FISICO, abs(self.ms_a_rpm(v_FR)))
        rpm_BR = min(self.RPM_MAX_FISICO, abs(self.ms_a_rpm(v_BR)))

        # --- DIRECCIONES ESP1 (Lado Izquierdo) ---
        dir_FL = 2 if v_FL >= 0 else 1
        dir_BL = 2 if v_BL >= 0 else 1
        
        # --- DIRECCIONES ESP2 (Lado Derecho) ---
        dir_FR = 1 if v_FR >= 0 else 2
        dir_BR = 1 if v_BR >= 0 else 2

        # --- ENVIAR A ESP32 ---
        self.mandar_orden(self.esp_1, "A", dir_FL, rpm_FL)
        self.mandar_orden(self.esp_1, "B", dir_BL, rpm_BL)
        self.mandar_orden(self.esp_2, "A", dir_FR, rpm_FR)
        self.mandar_orden(self.esp_2, "B", dir_BR, rpm_BR)

        # Imprimimos en consola para que veas qué está calculando hoy en tu PC
        self.get_logger().info(f"CMD -> vx: {vx:+.2f}, vy: {vy:+.2f}, w: {omega:+.2f} | FL:{rpm_FL:.0f} BL:{rpm_BL:.0f} FR:{rpm_FR:.0f} BR:{rpm_BR:.0f}")

def main(args=None):
    rclpy.init(args=args)
    nodo = PickassoBaseNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Apagando nodo base...")
        if nodo.hardware_conectado:
            # Frenar motores al salir
            for p in (nodo.esp_1, nodo.esp_2):
                nodo.mandar_orden(p, "A", 0, 0)
                nodo.mandar_orden(p, "B", 0, 0)
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()