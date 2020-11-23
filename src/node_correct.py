#!/usr/bin/env python

import sys

import numpy as np

import roslib
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

scan_data = LaserScan()
vel_data = Twist()
chosen_model = 0

def percentageReturn(start, end, value):
    if(np.abs(value) < np.abs(start)):
        print(value)
        print(start)
        return 0
    elif(np.abs(value)>np.abs(end)):
        return 1
    result = value/(np.abs(end)-np.abs(start))
    if (result > 1 or result < -1):
        result = np.sign(result)
    return result

def velocityModel(distance, number):
    #step function
    if(number==1):
        if(distance>0.6):
            return 1
        else:
            return 0
    #proportional velocity
    elif(number==2):
        return percentageReturn(0.5, 2, distance)
    #tanh velocity
    elif(number==3):
        return np.tanh(distance)
    #default: x-squared + linear velocity
    else:
        if(distance < -1 or distance > 1):
            return percentageReturn(np.sign(distance)*1, np.sign(distance)*5, distance)
        else:
            if(distance<0.3):
                return 0
            else:
                return (distance) ** 2

def callback(data, call):
    #laserscan
    if (call):
        global scan_data
        scan_data = data

    #velocity
    else:
        global vel_data
        vel_data.linear = data.linear
        vel_data.angular = data.angular 

def scanWorker(laser_scan):
    angle_min = laser_scan.angle_min
    angle_max = laser_scan.angle_max
    angle_increment = laser_scan.angle_increment
    ranges = laser_scan.ranges
    scan_count = len(ranges)

    #get value for middle-most scan
    middle_value = np.round(scan_count//2)
    return ranges[middle_value]

#subscriber to scan data
scan = rospy.Subscriber('/scan', LaserScan, callback, 1)

velo = rospy.Subscriber('/cmd_vel', Twist, callback, 0)

#publisher mover
pub = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size= 10)

rospy.init_node('correcter')

subrate = rospy.Rate(60)

while not rospy.is_shutdown():
    change_vel = vel_data
    while(scan_data == LaserScan()):
        subrate.sleep()
    subrate.sleep()
    if (change_vel.linear.x > 0):
        middle_value = scanWorker(scan_data)
        change_vel.linear.x=velocityModel(middle_value, chosen_model)
    
    pub.publish(change_vel)
