#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import sys
import math
from sensor_msgs.msg import LaserScan
import numpy as np
import os

class ROSRunner:
    """ 
        initialize your parameters in here.
        "self." prefix represents the instance of the class.
    """    
    def __init__(self, agent_name):        
        self.agent_name = agent_name
        self.pub_drive = None
        #PID CONTROL PARAMS
        # self.kp = 0.2#TODO
        # self.kd = 1e-3#TODO
        # self.ki = 1e-5#TODO
        # Define extra parameters for PID controller (e.g. previous error, previous integral term) #TODO         
        self.prev_error = 0.0 
        self.error = 0.0
        self.integral = 0.0
        self.last_callback_time = None        
        self.initial_lidar_callback = True
        # Define lookahead and desired distance to wall #TODO
        # self.lookahead =  #TODO
        # self.desired_distance =  #TODO
          

    """ 
        Implement your own PID controller 
    """
    def pid_control(self):       
        current_time = rospy.get_time()
        time_diff = current_time - self.last_callback_time        
        angle = 0.0
        # P = #TODO 
        # D = #TODO
        # if abs(self.integral) > 1e-5 or math.isnan(self.integral):
        #     self.integral = 0
        # else:
        #     self.integral = #TODO
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0  #TODO      
        msg.drive.steering_angle = 0.0 #TODO
        # Publish command to simulation 
        self.pub_drive.publish(msg)
        # self.last_callback_time = current_time
        return 0.0 

    """ 
        Laser callback: process lidar data to compute control input.
        Hint: You may want to call pid controller in here using the processed error based on the laser measurement. 
    """
    def lidar_callback(self, data):
        # if self.initial_lidar_callback:
        #     self.last_callback_time = rospy.get_time()
        # else:
        #     self.initial_lidar_callback = False
        #     return
        # angles = np.arange(data.angle_min,data.angle_max,data.angle_increment)                             
        for idx, angle in enumerate(angles):       
            # Get distances and angles
            rospy.loginfo(str(idx))
            rospy.loginfo(str(angle))        
        # Dt =    #TODO                
        # self.error = self.desired_distance - Dt  #TODO
        #send error to pid_control
        self.pid_control()
        

    """
        Do not modify the below "run" function 
    """    
    def run(self):        
        print ("agent_name", agent_name)
        rospy.init_node('gym_agent_%s' % self.agent_name, anonymous=True)
        self.pub_drive = rospy.Publisher('/%s/drive' % self.agent_name, AckermannDriveStamped, queue_size=5)        
        # start listening
        rospy.Subscriber('/%s/scan' % self.agent_name, LaserScan, self.lidar_callback)
        rospy.sleep(3)
        rospy.spin()


"""
Nothing to change in here
"""
if __name__ == "__main__":    
    agent_name = os.environ.get("F1TENTH_AGENT_NAME")
    runner = ROSRunner(agent_name)
    # launch
    runner.run()
