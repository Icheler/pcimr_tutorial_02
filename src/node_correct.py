#!/usr/bin/env python

import sys

import numpy as np

import roslib
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

scan_data = LaserScan()
vel_data = Twist()
#chosen velocity model: see velocityModel()
chosen_model = 0

#returns percentage based value between start and end points
def percentageReturn(start, end, value):
    if(np.abs(value) < np.abs(start)):
        return 0
    elif(np.abs(value)>np.abs(end)):
        return 1
    result = value/(np.abs(end)-np.abs(start))
    if (result > 1 or result < -1):
        result = np.sign(result)
    return result

#implementation of several velocity models
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

#standard callback for laserScan and velocity Data gathered by Subscribers
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

# returns range values for specific interval specified by direction
def scanWorker(laser_scan, direction):
    angle_min = laser_scan.angle_min
    ranges = laser_scan.ranges
    scan_count = len(ranges)
    start = 0
    end = scan_count-1

    #get position for middle-most scan
    middle_value = int(scan_count//2)

    #get 45 degree approximate lasers
    approx_number = int(0.8/np.abs(angle_min)*(middle_value))

    #directions from left to leftForward to Forward to rightForward to right
    if(direction==0):
        target_value = middle_value - 2*approx_number
        end = target_value + approx_number
    elif(direction==1):
        target_value = middle_value - approx_number
        start = target_value - approx_number
        end = target_value + approx_number
    elif(direction==2):
        start = middle_value - approx_number
        end = middle_value + approx_number
    elif(direction==3):
        target_value = middle_value + approx_number
        start = target_value - approx_number
        end = target_value + approx_number
    else:
        target_value = middle_value + 2*approx_number
        start = target_value - approx_number        

    result = ranges[start:end]
    return result

#returns scanWorker scan_values after assigning directions to them
def getDirectionalScan(velo_data, scan_data):
    if(velo_data.linear.x>0):
        if(velo_data.linear.y == 0):
            return(scanWorker(scan_data, 2))
        elif(velo_data.linear.y < 0):
            return(scanWorker(scan_data, 3))
        else:
            return(scanWorker(scan_data, 1))
    else:
        if(velo_data.linear.y < 0):
            return(scanWorker(scan_data, 0))
        else:
            return(scanWorker(scan_data, 4))

#subscriber to scan data
scan = rospy.Subscriber('/scan', LaserScan, callback, 1)

#subscriber to cmd_vel data
velo = rospy.Subscriber(rospy.get_param("listening_channel"), Twist, callback, 0)

#publisher correcter
pub = rospy.Publisher(rospy.get_param("publishing_channel"), Twist, queue_size= 10)

rospy.init_node('correcter')

subrate = rospy.Rate(60)

#while no scan data is available, we wait
while(scan_data == LaserScan()):
    while not rospy.is_shutdown:
        subrate.sleep()

while not rospy.is_shutdown():
    subrate.sleep()
    change_vel = vel_data

    #see if sensor readings can impact the velocity, i.e moving backwards in any way is not feasable for us to adjust
    if ((change_vel.linear.x > 0 or (change_vel.linear.y != 0)) and not (change_vel.linear.x < 0)):
        ranges = getDirectionalScan(change_vel, scan_data)
        lowest_vel = 100
        #get lowest velocity estimate for all points in ranges cloud
        for i in range(len(ranges)):
            test_vel = velocityModel(ranges[i], chosen_model)
            if(test_vel<lowest_vel):
                lowest_vel=test_vel
        if(vel_data.linear.x != 0):
            change_vel.linear.x= lowest_vel
        if(vel_data.linear.y != 0):
            change_vel.linear.y= lowest_vel*np.sign(change_vel.linear.y)
    
    #quality of life to see adjustments
    print(change_vel)
    pub.publish(change_vel)
