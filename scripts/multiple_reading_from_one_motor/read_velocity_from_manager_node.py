#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def velocity_callback(msg):
    rospy.loginfo(f"Velocidade Atual: {msg.data}")

rospy.init_node('velocity_node')
rospy.Subscriber('/dynamixel/velocity', Int32, velocity_callback)
rospy.spin()
