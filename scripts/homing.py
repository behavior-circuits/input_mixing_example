#! /usr/bin/env python
import numpy as np
import rospy
import behavior_gates as bg

from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class TargetCalculation:

    def __init__(self):
        self.robot_pose = Pose()
        self.target = PoseStamped()
        self.cmd_vel = Twist()
        rospy.Subscriber('/move_base_simple/goal', PoseStamped,self.target_callback)
        rospy.Subscriber('/odom', Odometry, self.pose_callback)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        while not rospy.is_shutdown():

            
            rho, alpha = self.calculate_rho_alpha()
            cmd_vel = Twist()

            normalized_angle = sigmoid(alpha,0.8,0,True)

            normalized_rho   = normaliser(rho,0.1,0.02)

            cmd_vel.linear.x = 0.1 *bg.AMP(normalized_rho,bg.NOT(np.abs(normalized_angle))) #normalized_rho 
            #print(normalized_rho)
            cmd_vel.angular.z = -1* normalized_angle#bg.AMP(normalized_angle,normalized_rho)

	    print(alpha,cmd_vel.angular.z)
            #print(rho,cmd_vel.linear.x)
	    #print(cmd_vel)
            self.cmd_vel = cmd_vel
            pub.publish(self.cmd_vel)
            rospy.sleep(0.01)

    def pose_callback(self, odom):
        #print('odom: ' + str(odom))
        self.robot_pose = odom.pose.pose

    def target_callback(self,data):
        #print("target: " + str(data))
        self.target=data

    def calculate_rho_alpha(self):
        rot = self.robot_pose.orientation
        roll, pitch, yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        dist = np.array([self.robot_pose.position.x - self.target.pose.position.x,
                         self.robot_pose.position.y - self.target.pose.position.y,
                         self.robot_pose.position.z - self.target.pose.position.z])
          
        rho = np.linalg.norm(dist)        
	#print(rho)
        alpha = np.arctan2(dist[0],dist[1]) +np.pi/2 + yaw

        if alpha > np.pi:
            alpha -= 2*np.pi
        '''
        if alpha < -np.pi:
            alpha += np.pi
        '''



        return (rho, alpha)


def normaliser(x,max_val,min_val):
    if x >= max_val:
        return 1
    if x <= min_val:
        return 0
    else:
        return (x-min_val)/(max_val-min_val)

def sigmoid(x,steepness,midpoint,symmetric=False):
    if symmetric== True:
        return 2/(1+np.exp(-steepness*(x-midpoint)))-1
    else:
        return 1/(1+np.exp(-steepness*(x-midpoint)))


if  __name__=="__main__":
    rospy.init_node("homing")
    try:
        node=TargetCalculation()

    except rospy.ROSInterruptException:
        pass





