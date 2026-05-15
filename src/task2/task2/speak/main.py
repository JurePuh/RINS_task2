import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess


class Speak(Node):
    def __init__(self):
        super().__init__('speak')
        self.get_logger().info('Speak node started, listening on /speak')

        self.subscription = self.create_subscription(
            String,
            '/speak',
            self.speak_callback,
            10
        )

    def speak_callback(self, msg: String):
        text = msg.data
        self.get_logger().info(f'Speaking: "{text}"')

        try:
            subprocess.Popen(['espeak-ng', '-v', 'en-gb-scotland', '-s', '100', text])
        except FileNotFoundError:
            self.get_logger().error('espeak-ng not found! Install with: sudo apt install espeak-ng')


def main():
    rclpy.init(args=None)
    node = Speak()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

