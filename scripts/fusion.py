#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import behavior_gates as bg

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
                self.behaviors[index,0] = data.linear.x
                self.behaviors[index,1] = data.angular.z

	def fusion(self):
		cmd_vel = Twist()
                # Circuit goes here
                print(self.behaviors)
		cmd_vel.linear.x = bg.INVOKE(self.behaviors[2,0],10*self.behaviors[0,0])#bg.AND(self.behaviors[0,0],self.behaviors[1,0])
                cmd_vel.angular.z = bg.PREVAIL(self.behaviors[1,1],bg.INVOKE(self.behaviors[2,1],10*self.behaviors[0,1]))
		self.pub.publish(cmd_vel)



if __name__ == '__main__':
	try:
		rospy.init_node("fusion")
		fus = Fusion()	
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- FUSION-ERROR! ---------")

