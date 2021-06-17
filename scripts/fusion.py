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
		self.pub                     = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
		while not rospy.is_shutdown():
			self.fusion()

        def generate_behavior(self,msg,args):
                self.behaviors[args[0]] = [data.linear.x,data.angular.z]

	def fusion(self):
		cmd_vel = Twist()
                print(self.behaviors)
		self.pub.publish(cmd_vel)

