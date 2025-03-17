import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import collections
import time
import csv
from datetime import datetime
import numpy as np

GOAL_CURRENT_VALUE = 300
GRASPING_CURRENT_VALUE = 100


class Middle(Node):
    def __init__(self):
        super().__init__('middle_manager')

        self.pos = []
        self.vels = []
        self.currs = []
        self.last_vel = []
        self.time_last_vel = 0.0
        self.last_vels = np.zeros(4)
        self.relative_vels = np.ones(4,dtype=int)

        #self.current = GOAL_CURRENT_VALUE
        self.currents = np.ones(4)*GOAL_CURRENT_VALUE

        #timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        #self.time_last_vel = self.get_clock().now()

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/middle_data',
            self.listener_callback,
            10) 

        #self.start_time = self.get_clock().now()


    def listener_callback(self, msg):
        self.pos = [msg.data[pos] for pos in range(0,len(msg.data)-1,3)]
        self.vels = [msg.data[vel] for vel in range(1,len(msg.data)-1,3)]
        self.currs = [msg.data[vel] for vel in range(2,len(msg.data)-1,3)]
        time_now = msg.data[len(msg.data)-1]
        
        #time_diff = time_now- self.time_last_vel
        # if any(np.array(currents) > 0.2*GOAL_CURRENT_VALUE):
        #     self.get_logger().info(f'd_vel: {(np.array(velocities)[3] - self.vel[3]) / time_diff}')
        #quando existe redução da velocidade e aumento de corrente, diminui a goal current para não esmagar objetos
        # if (any(abs((np.array(velocities) - self.vel)) / time_diff) < 0.1) and (any(np.array(currents) > 0.8*GOAL_CURRENT_VALUE)):
        #     self.get_logger().info('Grasping!!!!')
        #     self.set_goal_current(np.ones(len(MOTOR_IDS), dtype=int)*GRASPING_CURRENT_VALUE)
            

        # elif (all(self.currents == GRASPING_CURRENT_VALUE) and all(np.array(currents) < 0.8*self.currents)):
        #     #se a corrente for pequena, o dedo não está a apanhar nada e apenas se movimenta
        #     self.set_goal_current(np.ones(len(MOTOR_IDS), dtype=int)*GOAL_CURRENT_VALUE)

        # self.vel = velocities
        # self.time_last_vel = time
        
        
        self.time_last_vel = msg.data[len(msg.data)-1]
        self.get_logger().info(f'Time:{msg.data}')
        


def main(args=None):
    rclpy.init(args=args)
    node = Middle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
