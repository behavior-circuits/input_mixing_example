'''
This script preprocesses rosbags and turns them into panda dataframes

'''


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

import rospy
import rosbag
import math
from pathlib import Path 

# path of the rosbags
rosbag_path = 'rosbags'
# choose participent
participent_numbers = [5]#[2, 3, 4, 5,6]
# choose rosbags for each participent
bag_names = ["main_circuit", "main_circuit_second_target", "normal_joystick", "normal_joystick_second_target", "sloppy_joystick", "sloppy_joystick_second_target","main_circuit_homing","main_circuit"]



'''
Auxilarry Functions
---------------------------------------------
'''

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z # in radians


def create_df_from_twist(bag, twist_topic):
    twist_df = pd.DataFrame(columns=['time', 'v', 'omega'])
    for topic, msg, t in bag.read_messages(topics=[twist_topic]):
        time = rospy.Time.to_sec(t) # convert to float
        v = msg.linear.x
        omega = msg.angular.z
        twist_df = twist_df.append({'time': time, 'v': v, 'omega': omega}, ignore_index=True)
    return twist_df

def create_df_from_laserscan(bag, laserscan_topic, t0=0): # t0 is a time offset that is subtracted
    scan_df = pd.DataFrame(columns=['time', 'ranges', 'angles'])
    for topic, msg, t in bag.read_messages(topics=[laserscan_topic]):
        time = (rospy.Time.to_sec(t) - t0)
        ranges = msg.ranges
        ranges = np.array(ranges)
        angles = np.arange(msg.angle_min, msg.angle_max+msg.angle_increment, msg.angle_increment)
        angles = angles[ranges > msg.range_min]
        ranges = ranges[ranges > msg.range_min]
        scan_df = scan_df.append({'time': time, 'ranges': ranges, 'angles': angles}, ignore_index=True)
    return scan_df

def create_df_from_odometry(bag, odometry_topic, t0=0, delete_xy_offset=False): # t0 is a time offset that is subtracted
    odom_df = pd.DataFrame(columns=['time', 'x', 'y', 'yaw', 'v', 'omega'])

    for topic, msg, t in bag.read_messages(topics=[odometry_topic]):
        time = (rospy.Time.to_sec(t) - t0)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        odom_df = odom_df.append({'time': time, 'x': x, 'y': y, 'yaw': yaw, 'v': v, 'omega': omega}, ignore_index=True)

    if delete_xy_offset:
        odom_df['x'] = odom_df['x'] - odom_df['x'][0]
        odom_df['y'] = odom_df['y'] - odom_df['y'][0]
        #odom_df['yaw'] = odom_df['yaw'] - odom_df['yaw'].iloc[0]
        #odom_df = odom_df - odom_df.iloc[0] # assuming v and omega are zero at the beginning
    return odom_df


'''
------------------------------------
'''



# list twist topic names
twist_topic_list = ['sloppy_joy_cmd', 'nav_cmd', 'joy_cmd', 'homing_cmd', 'cmd_vel_rc100', 'cmd_vel', 'col_cmd']
for participent_nr in participent_numbers:
    for bag_name in bag_names:
    
        folder_path_df = "experimental_data/participent_" + str(participent_nr) + '/' + bag_name  
        print(folder_path_df)
        Path(folder_path_df).mkdir(parents=True, exist_ok=True)
        bag = rosbag.Bag(rosbag_path + '/participent_' + str(participent_nr) + '/' + bag_name + '.bag')
    
        # create a pandas dataframe (df) for every twist topic
        for twist_topic_name in twist_topic_list:
            twist_df = create_df_from_twist(bag, "/" + twist_topic_name)
            file_path = folder_path_df + "/twist_" + twist_topic_name + ".csv"
            twist_df.to_csv(file_path, index = False, header=True)
        
        # create a pandas dataframe (df) for laserscan topic
        laserscan_df = create_df_from_laserscan(bag, "/scan")
        laserscan_df.to_csv(folder_path_df + "/scan.csv", index = False, header=True)
    
        # create a pandas dataframe (df) for odom topic
        odom_df = create_df_from_odometry(bag, "/odom", delete_xy_offset=True)
        odom_df.to_csv(folder_path_df + "/odom.csv", index = False, header=True)


