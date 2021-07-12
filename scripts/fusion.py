#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import circuits as circ

class Fusion:
	def __init__(self):
                # array contains homing_cmd col_cmd joy_cmd nav_cmd in that order
                self.behaviors = np.zeros((4,2))
                for i in range(5):
                    rospy.Subscriber(sys.argv[i+1], Twist, self.generate_behavior,i)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		while not rospy.is_shutdown():
			self.fusion()
                        rospy.sleep(0.01)
        def generate_behavior(self,data,index):
                self.behaviors[index,0] = 0.8*data.linear.x
                self.behaviors[index,1] = data.angular.z

	def fusion(self):
                circuit = circ.circuit_dict[sys.argv[5]]
                cmd_vel = circuit(self.behaviors)
		self.pub.publish(cmd_vel)


def EQUIV(x,y):
	return bg.SNOT(XOR)


if __name__ == '__main__':
	try:
		rospy.init_node("fusion")
		fus = Fusion()	
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- FUSION-ERROR! ---------")

