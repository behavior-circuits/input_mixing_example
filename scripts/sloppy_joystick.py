#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import behavior_gates as bg


def joystick_callback(twist):

    wait_time       = np.abs(np.random.normal(0,0.04))
    ctr_noise       = np.array([np.random.normal(0,0.01),np.random.normal(0,0.2)])
    lin_quant_steps = []
    ang_quant_steps = []
    joystick_input  = np.array([twist.linear.x,twist.angular.z])

    rospy.sleep(wait_time)

    joystick_input = joystick_input + ctr_noise
    if lin_quant_steps != []:
        joystick_input[0]=np.digitize(joystick_input[0],bins=lin_quant_steps)
    if ang_quant_steps != []:
        joystick_input[1]=np.digitize(joystick_input[1],bins=ang_quant_steps)

    output = Twist()
    output.linear.x  = max(-1,min(1,joystick_input[0]))
    output.angular.z = max(-1.3,min(1.3,joystick_input[1]))/1.3

    pub.publish(output)
    








if __name__ == '__main__':
	try:
                rospy.init_node("joystick_model")
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		#pub = rospy.Publisher('/sloppy_joy_cmd', Twist, queue_size=1)
                rospy.Subscriber('/joy_cmd', Twist, joystick_callback)
                rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- FUSION-ERROR! ---------")

