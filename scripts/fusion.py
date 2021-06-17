#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import behavior_gates as bg

class Fusion:
	def __init__(self):
                # array containes cat,blue,orange,cheese,collision
                self.behaviors = np.zeros((5,2))
                for i in range(5):
                    rospy.Subscriber(sys.argv[i+1], Point, self.generate_behavior,i)
		self.pub                     = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		while not rospy.is_shutdown():
			self.fusion()

        def generate_behavior(self,msg,args):
                self.behaviors[args[0]] = [data.linear.x,data.angular.z]

	def fusion(self):
		cmd_vel = Twist()
		cmd_vel.linear.x = 1
                cmd_vel.angular.z = prevail_gate(or_gate(invoke_gate(self.behaviors[2,1],self.behaviors[0,1]),self.behaviors[3,1]),self.behaviors[4,1])
                #cmd_vel.angular.z = self.behaviors[1,1]
                print(self.behaviors)
		self.pub.publish(cmd_vel)



if __name__ == '__main__':
	try:
		rospy.init_node("fusion")
		fus = Fusion()	
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- FUSION-ERROR! ---------")

