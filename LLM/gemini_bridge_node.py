import json
from google import genai

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MODEL = "gemini-2.5-flash"

SYSTEM_PROMPT = """
You are a command interpreter for a ROS2 mobile robot project.

Your job is to convert natural language instructions in English or Spanish
into a strict JSON command for ROS2.

Return ONLY valid JSON.
No markdown.
No explanations.

Allowed commands:
- MOVE
- STOP
- GO_TO_STATION
- START_RECTANGLE
- INVALID

Allowed directions:
- UP
- DOWN
- LEFT
- RIGHT

JSON format:
{
  "command": "MOVE | STOP | GO_TO_STATION | START_RECTANGLE | INVALID",
  "direction": "UP | DOWN | LEFT | RIGHT | null",
  "distance_m": number or null,
  "station_id": number or null,
  "duration_s": number or null,
  "reason": string or null
}
"""

class GeminiBridgeNode(Node):
    def __init__(self):
        super().__init__('gemini_bridge_node')

        self.client = genai.Client()

        self.sub = self.create_subscription(
            String,
            '/user_command',
            self.command_callback,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/robot_action',
            10
        )

        self.get_logger().info('Gemini bridge node started.')

    def command_callback(self, msg):
        user_text = msg.data.strip()
        self.get_logger().info(f"Received user_text: [{user_text}]")

        if not user_text:
            invalid_data = {
                "command": "INVALID",
                "direction": None,
                "distance_m": None,
                "station_id": None,
                "duration_s": None,
                "reason": "Empty input"
            }

            out = String()
            out.data = json.dumps(invalid_data)
            self.pub.publish(out)
            self.get_logger().warn("Received empty input.")
            return

        prompt = f"{SYSTEM_PROMPT}\n\nUser instruction: {user_text}"

        try:
            response = self.client.models.generate_content(
                model=MODEL,
                contents=prompt
            )

            raw = response.text.strip()
            self.get_logger().info(f"Raw Gemini response: {raw}")

            data = json.loads(raw)

            out = String()
            out.data = json.dumps(data)
            self.pub.publish(out)

            self.get_logger().info(f"Published JSON: {out.data}")

        except Exception as e:
            self.get_logger().error(f"Gemini request failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GeminiBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
