#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def pwm_callback(msg):
    rospy.loginfo(f"PWM Atual: {msg.data}")

rospy.init_node('pwm_node')
rospy.Subscriber('/dynamixel/pwm', Int32, pwm_callback)
rospy.spin()
