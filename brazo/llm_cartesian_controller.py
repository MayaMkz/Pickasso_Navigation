#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from xarm_msgs.srv import PlanPose, PlanExec


class LLMCartesianController(Node):
    def __init__(self):
        super().__init__('llm_cartesian_controller')

        # Servicios del xArm
        self.cliente_plan = self.create_client(PlanPose, '/xarm_pose_plan')
        self.cliente_exec = self.create_client(PlanExec, '/xarm_exec_plan')

        while not self.cliente_plan.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /xarm_pose_plan...')

        while not self.cliente_exec.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /xarm_exec_plan...')

        # Suscriptor al comando interpretado por Gemini
        self.subscription = self.create_subscription(
            String,
            '/robot_action',
            self.robot_action_callback,
            10
        )

        # Posición actual estimada del efector final
        self.current_x = 0.4
        self.current_y = -0.2
        self.current_z = 0.112

        # Paso por defecto en metros
        self.default_step = 0.05

        # Límites cartesianos (convertidos de mm a m)
        self.y_min = -0.450
        self.y_max =  0.450
        self.z_min =  0.110
        self.z_max =  0.540

        self.get_logger().info('Nodo LLM Cartesian Controller iniciado.')
        self.get_logger().info(
            f'Posición inicial: X={self.current_x}, Y={self.current_y}, Z={self.current_z}'
        )
        self.get_logger().info(
            f'Límites: Y=[{self.y_min}, {self.y_max}] m, '
            f'Z=[{self.z_min}, {self.z_max}] m'
        )

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def mover_a_coordenada(self, x, y, z):
        request = PlanPose.Request()

        request.target.position.x = x
        request.target.position.y = y
        request.target.position.z = z

        request.target.orientation.x = 1.0
        request.target.orientation.y = 0.0
        request.target.orientation.z = 0.0
        request.target.orientation.w = 0.0

        self.get_logger().info(f'Calculando ruta hacia: X={x}, Y={y}, Z={z}...')

        future = self.cliente_plan.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response is None:
            self.get_logger().error('No hubo respuesta del servicio de planificación.')
            return False

        if response.success:
            self.get_logger().info('Ruta encontrada. Ejecutando movimiento...')

            exec_req = PlanExec.Request()
            exec_req.wait = True

            exec_future = self.cliente_exec.call_async(exec_req)
            rclpy.spin_until_future_complete(self, exec_future)

            self.get_logger().info('Movimiento completado.')
            return True
        else:
            self.get_logger().error('No se pudo encontrar una ruta válida.')
            return False

    def robot_action_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'JSON inválido recibido: {msg.data}')
            return

        command = data.get('command')
        direction = data.get('direction')
        distance = data.get('distance_m')

        self.get_logger().info(f'Comando recibido: {data}')

        if command == 'INVALID':
            self.get_logger().warn(f'Comando inválido ignorado: {data.get("reason")}')
            return

        if command != 'MOVE':
            self.get_logger().warn(f'Comando no soportado aún: {command}')
            return

        if direction not in ['UP', 'DOWN', 'LEFT', 'RIGHT']:
            self.get_logger().warn(f'Dirección no soportada: {direction}')
            return

        step = self.default_step if distance is None else float(distance)

        target_x = self.current_x
        target_y = self.current_y
        target_z = self.current_z

        # Mapeo de direcciones cartesianas
        if direction == 'UP':
            target_z += step
        elif direction == 'DOWN':
            target_z -= step
        elif direction == 'LEFT':
            target_y += step
        elif direction == 'RIGHT':
            target_y -= step

        # Aplicar límites
        limited_y = self.clamp(target_y, self.y_min, self.y_max)
        limited_z = self.clamp(target_z, self.z_min, self.z_max)

        if limited_y != target_y or limited_z != target_z:
            self.get_logger().warn(
                f'Objetivo fuera de límites. '
                f'Se ajustó de Y={target_y}, Z={target_z} '
                f'a Y={limited_y}, Z={limited_z}'
            )

        target_y = limited_y
        target_z = limited_z

        self.get_logger().info(
            f'Moviendo {direction} con paso {step} m -> '
            f'Nuevo objetivo: X={target_x}, Y={target_y}, Z={target_z}'
        )

        success = self.mover_a_coordenada(target_x, target_y, target_z)

        if success:
            self.current_x = target_x
            self.current_y = target_y
            self.current_z = target_z
            self.get_logger().info(
                f'Posición actual actualizada: '
                f'X={self.current_x}, Y={self.current_y}, Z={self.current_z}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LLMCartesianController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
