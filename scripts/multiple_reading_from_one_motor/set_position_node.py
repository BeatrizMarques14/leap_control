#!/usr/bin/env python3
import rospy
import argparse
from std_msgs.msg import Int32

def set_position(position):
    rospy.init_node("set_position_node", anonymous=True)
    pub = rospy.Publisher("/set_position", Int32, queue_size=10)

    rospy.sleep(1)  # Aguarda o ROS inicializar completamente

    pub.publish(position)
    rospy.loginfo(f"Enviando posição: {position}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Define a posição do motor.")
    parser.add_argument("position", type=int, help="Posição desejada para o motor")
    args = parser.parse_args()

    try:
        set_position(args.position)
    except rospy.ROSInterruptException:
        pass
