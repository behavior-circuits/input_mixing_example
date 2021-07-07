#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import behavior_gates as bg


def joystick_callback(twist):

    wait_time      = np.abs(np.random.normal(0,0.1))
    ctr_noise      = np.array([np.random.normal(0,0.01),np.random.normal(0,0.1)])
    joystick_input = np.array([twist.linear.x,twist.angular.z])

    rospy.sleep(wait_time)

    joystick_input = joystick_input + ctr_noise

    output = Twist()
    output.linear.x  = joystick_input[0]
    output.angular.z = joystick_input[1]

    pub.publish(output)
    








if __name__ == '__main__':
	try:
                rospy.init_node("joystick_model")
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
                rospy.Subscriber('/joy_cmd_vel', Twist, joystick_callback)
                rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- FUSION-ERROR! ---------")

