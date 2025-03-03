import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time
from datetime import datetime
import csv

class ReadVelocities(Node):
    def __init__(self):
        super().__init__('read_velocities')

        # Criar timestamp para o nome do ficheiro
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"/home/beatrix/ros2_ws/src/leap_control/leap_hand_control/leap_hand_control/data/velocities/finger_velocities_{timestamp}.csv"

        # Criar e escrever o cabeçalho do CSV
        with open(self.csv_filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Motor1", "Motor2", "Motor3", "Motor4"])  # Cabeçalho


        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/dynamixel_finger_velocities',
            self.listener_callback,
            2000)
        
        self.start_time = self.get_clock().now()
    
    def listener_callback(self, msg):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        data_row = [elapsed_time] + list(msg.data)  # Converte msg.data para lista
        with open(self.csv_filename, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(data_row)
        #self.get_logger().info(f'Velocidades : {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ReadVelocities()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando nó e salvando dados.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
