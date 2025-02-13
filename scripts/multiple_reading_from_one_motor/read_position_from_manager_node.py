#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def position_callback(msg):
    rospy.loginfo(f"Posição Atual: {msg.data}")

rospy.init_node('position_node')
rospy.Subscriber('/dynamixel/position', Int32, position_callback)
rospy.spin()
