#!/usr/bin/env python
"""
Title: ROS Tutorial: TF and Turtlesim

Objective: Teach students how to use the TF API in ROS and practice with the Turtlesim package.

Description: In this practical session, students will learn about the TF API in ROS and apply this knowledge using the Turtlesim package. The tutorial will provide a skeleton Python code for students to practice with and modify as needed.

Materials:

A computer with ROS installed
Python programming environment
Turtlesim package installed
"""


import rospy
import tf
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

def pose_callback(pose):
    # Get the current position and orientation of the turtle
    x, y, theta = pose.x, pose.y, pose.theta

    # Define the transformation from the "world" frame to the "turtle" frame
    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, theta),
                     rospy.Time.now(),
                     "turtle",
                     "world")

if __name__ == "__main__":
    rospy.init_node("turtle_tf_broadcaster")

    # Subscribe to the turtle's pose topic
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    rospy.spin()
