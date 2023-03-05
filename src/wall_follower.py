#!/usr/bin/env python2

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")





    def __init__(self):
        # TODO:
        # Initialize your publishers and
        # subscribers here



        #self.side=self.SIDE 
        #self.desired_distance = self.DESIRED_DISTANCE
        self.side =self.SIDE
        self.desired_distance = self.DESIRED_DISTANCE
        self.velocity = self.VELOCITY
        # Set gains for PD controller
        self.kp = 4
        self.kd = 0.2

        # Initialize error term and previous error term
        self.error = 0.0
        self.prev_error = 0.0





        rospy.init_node('wall_follower')

        # Initialize the publishers and subscribers
        self.cmd_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.mrk_pub = rospy.Publisher('/visualization', Marker, queue_size=1)
        self.min_angle = np.deg2rad(-13)
        self.max_angle = np.deg2rad(13)


    def laser_callback(self, scan_msg):
        # wall follower logic here
        #Convert the ranges data to a numpy array
        ranges = np.array(scan_msg.ranges)
        goodpoints= np.abs(ranges) < 4

        # Slice the ranges data to the desired field of view
        angle_increments = scan_msg.angle_increment * np.arange(len(scan_msg.ranges))
        angles = scan_msg.angle_min + angle_increments
        fov_indices = np.where((angles >= self.min_angle) & (angles <= self.max_angle))
        fov_ranges = ranges[fov_indices]
        x_cartesian = ranges*np.cos(angles)
        y_cartesian = ranges*np.sin(angles) 
        
        #x_cartesian = fov_ranges*np.cos(angles)
        #y_cartesian = fov_ranges*np.sin(angles)


        coords = (x_cartesian,y_cartesian)
        
       # Fit a line to the ranges data using least squares regression
        n = len(x_cartesian)
        sistart = int(0)
        slice_index = int( n * 2/3 )
        slice_index_before = int( n * 1 /3)

        #slice_good_part = np.abs(ranges) < self.DESIRED_DISTANCE + 3

        # slice_x = np.array(x_cartesian)[slice_good_part]
        # slice_y = np.array(y_cartesian)[slice_good_part]


        #slice_pointsx = np.array(x_cartesian)[slice_good_part]
        #slice_pointsy = np.array(y_cartesian)[slice_good_part]

        # if self.SIDE == 1:
        #     x_r_slice= slice_pointsx > -1
        # else:
        #     x_r_slice = slice_pointsx < 1
        
        # slice_pointsx = np.array(slice_pointsx)[x_r_slice]
        # slice_pointsy = np.array(slice_pointsy)[x_r_slice]













       # print(slice_pointsx)
        if self.SIDE == -1:
            slice_good_part = ranges[:slice_index] < self.DESIRED_DISTANCE + 2
            slice_pointsx = np.array(x_cartesian[:slice_index])[slice_good_part]
            slice_pointsy = np.array(y_cartesian[:slice_index])[slice_good_part]

        else:
            slice_good_part = ranges[slice_index_before:] < self.DESIRED_DISTANCE + 2
            slice_pointsx = np.array(x_cartesian[slice_index_before:])[slice_good_part]
            slice_pointsy = np.array(y_cartesian[slice_index_before:])[slice_good_part]

        # slice_good_part = ranges[slice_index_before:n] < self.desired_distance + 2
        # slice_pointsx = np.array(x_cartesian[slice_index_before:n])[slice_good_part]
        # slice_pointsy = np.array(y_cartesian[slice_index_before:n])[slice_good_part]
        # print(slice_pointsx) 

       #coeffs = np.lin(x, y, 1)    \\\




        #print(x)
        A = np.vstack([slice_pointsx, np.ones(len(slice_pointsx))]).T
        m, c = np.linalg.lstsq(A, slice_pointsy)[0] 
        y_pred = slice_pointsx*m + c
        vt=VisualizationTools()
        vt.plot_line(slice_pointsx, y_pred, self.mrk_pub)
        print(c , m)
        # x_mean = np.mean(np.arange(n))
        # y_mean = np.mean(fov_ranges)
        # num = np.sum((np.arange(n) - x_mean) * coords)
        # den = np.sum((np.arange(n) - x_mean)**2)
        # slope = num / den
        # intercept = y_mean - slope * x_mean
         # Calculate the intersection point with the wall
        #wall_distance = -c/ np.cos(np.arctan(m))   
        # 


        wall_distance = abs(-c-m )/(np.sqrt(1+m**2))
        # if self.SIDE == 1:
        #     wall_distance = abs(c)/(np.sqrt(1+m**2))
        # else:
        #     wall_distance = abs(c)/(np.sqrt(1+m**2))
        print("wall distance" , wall_distance)

        error = self.DESIRED_DISTANCE - wall_distance
        print(self.desired_distance)
        # if self.SIDE == 1:
        #     error = self.desired_distance - wall_distance
        # else:
        #     error = -self.desired_distance - wall_distance
        print("error", error)
        # Calculate the derivative of the error
        delta_error = self.error - self.prev_error

        # Calculate the steering angle using PD control
        if self.SIDE ==-1:
            steering_angle = self.kp * error + self.kd * delta_error
        else:
            steering_angle = -self.kp * error - self.kd * delta_error

        # Apply limits to the steering angle
        steering_angle = np.clip(steering_angle, self.min_angle, self.max_angle)
        print("steering angle" ,steering_angle)
    


  




        # Publish the steering command
        cmd_msg = AckermannDriveStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.header.frame_id = "base_link"
        cmd_msg.drive.speed = self.VELOCITY # Set the speed to 1.0 m/s
        cmd_msg.drive.steering_angle = steering_angle # Set the initial steering angle to 0 degrees
        self.cmd_pub.publish(cmd_msg)
        self.prev_error = self.error

    # TODO:
    # Write your callback functions here.

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
