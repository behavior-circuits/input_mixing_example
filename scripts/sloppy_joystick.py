#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import behavior_gates as bg



class sloppy_joystick:

    def __init__(self):
        self.lin_quant_steps = []
        self.ang_quant_steps = []
        self.noise_variance  = np.array([0.01,0.2])
        self.joystick_input  = np.zeros(2)

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #pub = rospy.Publisher('/sloppy_joy_cmd', Twist, queue_size=1)
        rospy.Subscriber('/joy_cmd', Twist, self.joystick_callback)
        while not rospy.is_shutdown():
            wait_time       = np.abs(np.random.normal(0,0.04))
            ctr_noise       = np.array([np.random.normal(0,self.noise_variance[0]),np.random.normal(0,self.noise_variance[1])])

            rospy.sleep(wait_time)

            joystick_input = self.joystick_input + ctr_noise
            if self.lin_quant_steps != []:
                joystick_input[0]=np.digitize(joystick_input[0],bins=self.lin_quant_steps)
            if self.ang_quant_steps != []:
                joystick_input[1]=np.digitize(joystick_input[1],bins=sel.fang_quant_steps)

            output = Twist()
            output.linear.x  = max(-1,min(1,joystick_input[0]))
            output.angular.z = max(-1.3,min(1.3,joystick_input[1]))/1.3

            pub.publish(output)
            

    def joystick_callback(self,twist):
        self.joystick_input  = np.array([twist.linear.x,twist.angular.z])










if __name__ == '__main__':
	try:
                rospy.init_node("joystick_model")
                joy =sloppy_joystick()
                rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- FUSION-ERROR! ---------")

