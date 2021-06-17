#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import behavior_gates as bg



'''
Die gesammte Kollisionsvermeidung ist in einer Klasse verpackt. Die aktuellen Geschwindigkeitsdaten werden durch den Callback des
Geschwindigkeits-Subscribers bereitgestellt. Die aktuelle Geschwindigkeit wird da fuer die Berechnung der neuen Sollgeschwindigkeit verwendet. 
Alternativ haetten hier auch globale Variabeln verwendet werden koennen. Diese Methode wird in der Community allerdings als
eleganter angesehen.
'''
class CollisionAvoidance:

    def __init__(self):

        self.current_lin_vel_x = 0.0
        self.current_ang_vel_z = 0.0


        '''
        Die momentane Geschwindigkeit des Roboters wird in der Simulation auf dem Topic odom gepublisht. 
        Hier wollen wir subscriben. Ebenfalls benoetigen wir die Daten des Laserscanners. 
        '''
        rospy.Subscriber("odom", Odometry, self.velocity_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        '''
        Das Ergebnis der Berechnung wird dem Roboter als Soll-Geschwindigkeit zurueckgegeben.
        '''
        self.col_avoid_publisher = rospy.Publisher("/col_cmd", Twist, queue_size=10)

        rospy.spin()

    def velocity_callback(self, current_odometry):
        self.current_lin_vel_x = current_odometry.twist.twist.linear.x
        self.current_ang_vel_z = current_odometry.twist.twist.angular.z


    def laser_callback(self, current_laser_scan):
        sensor_angles = np.arange(current_laser_scan.angle_min, 
                          current_laser_scan.angle_max + current_laser_scan.angle_increment,
                          current_laser_scan.angle_increment)
        sensor_ranges = np.array(current_laser_scan.ranges)
        force = self.calculate_force(sensor_angles,sensor_ranges)
        velocity_adjustment = Twist()
        velocity_adjustment.linear.x  = force[0]
        velocity_adjustment.angular.z = force[1]
        
        self.col_avoid_publisher.publish(velocity_adjustment)


    def calculate_force(self,sonar_angles,sonar_ranges):
        force = np.zeros(2)
        force[0]=0.1
        #force calculation using analogical gates
        force_contrib=np.zeros(len(sonar_angles))
        linear_force_contrib=np.zeros(len(sonar_angles))
        for i in range(len(sonar_angles)):
            if sonar_angles[i] >= np.pi:
                sonar_angles[i]=sonar_angles[i]-2*np.pi

            if sonar_ranges[i] == 0:
		sonar_ranges[i]=10000
            normalised_range=normaliser(sonar_ranges[i],0.4,0.07)
            normalised_angle=sigmoid(sonar_angles[i],3,0,symmetric=True)   
 		

            force_contrib[i]= bg.AND(bg.AMP(bg.SNOT(normalised_range),-1*bg.SNOT(normalised_angle)),-1*bg.SNOT(normalised_angle))
            linear_force_contrib[i]= bg.AND(normalised_range,bg.SNOT(normalised_angle))

        force[1]=sigmoid(np.sum(force_contrib),10,0,True)
	force[0]= sigmoid(np.sum(linear_force_contrib),1,0)*0.1

        #print(force[1],np.sum(force_contrib))
        force[1]=force[1]
        return force



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




if __name__ == '__main__':

    rospy.init_node("CollisionAvoidance")
    print("ros node initiated")
    try:
        node = CollisionAvoidance()
    except rospy.ROSInterruptException:
        pass

