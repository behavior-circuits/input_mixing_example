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
                self.behaviors[index,0] = 0.8*data.linear.x
                self.behaviors[index,1] = data.angular.z

	def fusion(self):
		cmd_vel = Twist()
                # Circuit goes here
                print(self.behaviors)
                '''
                Fusion circuit 1
                ----------------
                
                This Circuit avoids collisions and assist the joystick using a homing behaviour which is invoked by the joystick
                '''
		cmd_vel.linear.x = bg.AND(10*self.behaviors[1,0],bg.INVOKE(self.behaviors[2,0],self.behaviors[0,0]))
                cmd_vel.angular.z = bg.PREVAIL(self.behaviors[1,1], bg.INVOKE(self.behaviors[2,1],self.behaviors[0,1]))



                '''
                Fusion circuit 2
                ----------------
                
                This Circuit lets the system navigate autonomously except if the joystick provides a different command
                '''
		#cmd_vel.linear.x = bg.PREVAIL(self.behaviors[2,0], self.behaviors[0,0])
                #cmd_vel.angular.z = bg.PREVAIL(self.behaviors[2,1], bg.PREVAIL(self.behaviors[1,1],self.behaviors[0,1]))

                
		self.pub.publish(cmd_vel)



def EQUIV(x,y):
	return bg.SNOT(XOR)


if __name__ == '__main__':
	try:
		rospy.init_node("fusion")
		fus = Fusion()	
	except rospy.ROSInterruptException:
		rospy.loginfo("---------- FUSION-ERROR! ---------")

