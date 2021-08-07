#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import sys
import math
from sensor_msgs.msg import LaserScan
import numpy as np
import os


class ROSRunner:
    CAR_WIDTH = 0.31
    # the min difference between adjacent LiDAR points for us to call them disparate
    DIFFERENCE_THRESHOLD = 2.
    SPEED = 5. 
    # the extra safety room we plan for along walls (as a percentage of car_width/2)
    SAFETY_PERCENTAGE = 300.

    def __init__(self, agent_name):        
        self.agent_name = agent_name
        self.pub_drive = None

    def lidar_callback(self, data):
        ranges = np.asarray(data.ranges)
        speed, angle = self.process_lidar(ranges)
        # create message & publish
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.pub_drive.publish(msg)

    def run(self):
        print ("agent_name", agent_name)
        rospy.init_node('gym_agent_%s' % self.agent_name, anonymous=True)
        self.pub_drive = rospy.Publisher('/%s/drive' % self.agent_name, AckermannDriveStamped, queue_size=5)
        # start listening
        rospy.Subscriber('/%s/scan' % self.agent_name, LaserScan, self.lidar_callback)
        rospy.sleep(3)
        rospy.spin()

    def preprocess_lidar(self, ranges):
        """ Any preprocessing of the LiDAR data can be done in this function.
            Possible Improvements: smoothing of outliers in the data and placing
            a cap on the maximum distance a point can be.
        """
        # remove quadrant of LiDAR directly behind us
        eighth = int(len(ranges)/8)
        return np.array(ranges[eighth:-eighth])
    
     
    def get_differences(self, ranges):
        """ Gets the absolute difference between adjacent elements in
            in the LiDAR data and returns them in an array.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        differences = [0.] # set first element to 0
        for i in range(1, len(ranges)):
            differences.append(abs(ranges[i]-ranges[i-1]))
        return differences
    
    def get_disparities(self, differences, threshold):
        """ Gets the indexes of the LiDAR points that were greatly
            different to their adjacent point.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        disparities = []
        for index, difference in enumerate(differences):
            if difference > threshold:
                disparities.append(index)
        return disparities

    def get_num_points_to_cover(self, dist, width):
        """ Returns the number of LiDAR points that correspond to a width at
            a given distance.
            We calculate the angle that would span the width at this distance,
            then convert this angle to the number of LiDAR points that
            span this angle.
            Current math for angle:
                sin(angle/2) = (w/2)/d) = w/2d
                angle/2 = sininv(w/2d)
                angle = 2sininv(w/2d)
                where w is the width to cover, and d is the distance to the close
                point.
            Possible Improvements: use a different method to calculate the angle
        """
        angle = 2*np.arcsin(width/(2*dist))
        num_points = int(np.ceil(angle / self.radians_per_point))
        return num_points

    def cover_points(self, num_points, start_idx, cover_right, ranges):
        """ 'covers' a number of LiDAR points with the distance of a closer
            LiDAR point, to avoid us crashing with the corner of the car.
            num_points: the number of points to cover
            start_idx: the LiDAR point we are using as our distance
            cover_right: True/False, decides whether we cover the points to
                         right or to the left of start_idx
            ranges: the LiDAR points

            Possible improvements: reduce this function to fewer lines
        """
        ranges = 0.0 
        return ranges

    def extend_disparities(self, disparities, ranges, car_width, extra_pct):
        """ For each pair of points we have decided have a large difference
            between them, we choose which side to cover (the opposite to
            the closer point), call the cover function, and return the
            resultant covered array.
            Possible Improvements: reduce to fewer lines
        """
        ranges = 0.0
    
        return ranges
            
    def get_steering_angle(self, range_index, range_len):
        """ Calculate the angle that corresponds to a given LiDAR point and
            process it into a steering angle.
            Possible improvements: smoothing of aggressive steering angles
        """
        steering_angle = 0.0
        # lidar_angle =  #TODO
        # steering_angle = #TODO 
        return steering_angle

    def process_lidar(self, ranges):
        """ Run the disparity extender algorithm!
            Possible improvements: varying the speed based on the
            steering angle or the distance to the farthest point.
        """
        self.radians_per_point = (2*np.pi)/len(ranges)
        proc_ranges = self.preprocess_lidar(ranges)
        differences = self.get_differences(proc_ranges)
        disparities = self.get_disparities(differences, self.DIFFERENCE_THRESHOLD)
        proc_ranges = self.extend_disparities(disparities, proc_ranges,
                self.CAR_WIDTH, self.SAFETY_PERCENTAGE)
        steering_angle = self.get_steering_angle(proc_ranges.argmax(),
                len(proc_ranges))
        speed = self.SPEED
        return speed, steering_angle

    
"""
Nothing to change in here
"""
if __name__ == "__main__":    
    agent_name = os.environ.get("F1TENTH_AGENT_NAME")
    runner = ROSRunner(agent_name)
    # launch
    runner.run()
