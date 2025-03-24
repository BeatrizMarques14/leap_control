import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import re
import numpy as np

class SetPositions(Node):
    def __init__(self):
        super().__init__('set_positions')
        self.publisher = self.create_publisher(Int32MultiArray, '/set_fingers_positions', 10)
        self.min = np.array([-1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, -0.349, -0.47, -1.20, -1.34]) + np.pi #limites minimos de todos os motores para simulação fornecidos peo codigo da LEAP Hand
        self.max = np.array([1.047,    2.23,  1.885,  2.042,  1.047,   2.23,  1.885,  2.042,  1.047,   2.23,  1.885,  2.042,  2.094,  2.443, 1.90,  1.88]) + np.pi #limites maximos de todos os motores para simulação fornecidos peo codigo da LEAP Hand

    def publish_positions(self, positions):
        msg = Int32MultiArray(data=positions)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publicando posições: {positions}')

    def radians_to_dynamixel(self,angle_rad):
        return int((angle_rad * 4095) / (2 * np.pi))

def main(args=None):
    rclpy.init(args=args)
    node = SetPositions()
    finger_names = ["index", "middle", "ring", "thumb"]
    
    try:
        while rclpy.ok():
            input_str = input("Introduzir o dedo e as posições (ex: middle 2048 2048 2048 2048) ou comando (ex: thumb close): ").strip()
            input_parts = input_str.split()

            if not input_parts:
                continue

            finger_name = input_parts[0].lower()

            data_to_send = []

            if len(input_parts) == 2 and input_parts[0].lower() == "hand":
                command = input_parts[1].lower()
                if command == "close":
                    # Enviar posições de fecho para middle e thumb
                    data_to_send.extend([finger_names.index("middle"), 1564, 2048, 3136, 2569])
                    data_to_send.extend([finger_names.index("thumb"), 2550, 630, 2935, 3030])
                elif command == "open":
                    # Enviar posições de abertura para middle e thumb
                    data_to_send.extend([finger_names.index("middle"), 1024, 2048, 2048, 2048])
                    data_to_send.extend([finger_names.index("thumb"), 2048, 1024, 2048, 2048])
                else:
                    node.get_logger().error("Comando inválido para 'hand'. Use 'close' ou 'open'.")
                    continue

            elif finger_name not in finger_names:
                node.get_logger().error("Nome do dedo inválido! Escolha entre: index, middle, ring, thumb.")
                continue

            elif len(input_parts) == 2 and input_parts[1].lower() in ["close", "open"]:
                command = input_parts[1].lower()
                if command == "close":
                    if finger_name == "middle":
                        positions = [1800, 2048, 2890, 2700]
                    elif finger_name == "thumb":
                        positions = [2550, 630, 2935, 3030]
                elif command == "open":
                    if finger_name == "middle":
                        positions = [1024, 2048, 2048, 2048]
                    elif finger_name == "thumb":
                        positions = [2048, 1024, 2048, 2048]
                data_to_send.extend([finger_names.index(finger_name)] + positions)
            else:
                input_values = " ".join(input_parts[1:])  # Ignorar o nome do dedo
                input_values = re.sub(r'[^0-9\s]', '', input_values)  # Remover caracteres não numéricos
                positions = list(map(int, input_values.split()))
                data_to_send.extend([finger_names.index(finger_name)] + positions)
            
            node.get_logger().info(f'Dedo: {finger_name}, Posições: {positions}')
            node.publish_positions(data_to_send)  # Adiciona o índice do dedo à mensagem

    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

