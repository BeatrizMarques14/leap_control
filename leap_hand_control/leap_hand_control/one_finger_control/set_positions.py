import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import re

class SetPositions(Node):
    def __init__(self):
        super().__init__('set_positions')
        self.publisher = self.create_publisher(Int32MultiArray, '/dynamixel_set_finger_positions', 10)

    def publish_positions(self, positions):
        msg = Int32MultiArray(data=positions)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publicando posições: {positions}')


def main(args=None):
    rclpy.init(args=args)
    node = SetPositions()
    
    try:
        while rclpy.ok():
            input_str = input("Introduzir as posições (ex: [2048 2048 2048 2048]) ou enviar comando (ex:close hand): ")
            input_str = re.sub(r'[^0-9\s]', '', input_str)  # Remove colchetes e caracteres não numéricos
            positions = list(map(int, input_str.split()))
            node.get_logger().info(f'Minhas posições: {positions}')
            node.publish_positions(positions)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
