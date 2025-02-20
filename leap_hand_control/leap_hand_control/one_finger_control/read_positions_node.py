import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class ReadPositions(Node):
    def __init__(self):
        super().__init__('read_positions')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/dynamixel_finger_positions',
            self.listener_callback,
            10)
        self.subscription  # Previne que o garbage collector elimine a subscrição
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Posições recebidas: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ReadPositions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
