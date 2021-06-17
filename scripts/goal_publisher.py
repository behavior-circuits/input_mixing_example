#! /usr/bin/env python
import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped


class TargetCalculation:

    def __init__(self):
        self.target      = PoseStamped()
        self.target.header.stamp =rospy.Time.now()
        self.target.header.frame_id = 'odom'
        self.target.pose.position.x=5
        self.target.pose.position.y=5
        '''
        The move_base_simple subscriber subscribes to the position of the movement goal described in rviz using:
        rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz
        '''
        rospy.Subscriber('/move_base_simple/goal', PoseStamped,self.target_callback)
        pub            = rospy.Publisher("goal_pose", PoseStamped, queue_size=10)
        while not rospy.is_shutdown():   
            pub.publish(self.target)



    def target_callback(self,data):
        self.target=data


if  __name__=="__main__":
    rospy.init_node("target_calculation")
    try:
        node=TargetCalculation()

    except rospy.ROSInterruptException:
        pass
