#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Twist

def leader_follower(leader, follower, target_distance, max_speed):
    listener = tf.TransformListener()
    cmd_vel_publisher = rospy.Publisher(f'/{follower}/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (trans1, rot1) = listener.lookupTransform("/world", f"/{leader}", rospy.Time(0))
            (trans2, rot2) = listener.lookupTransform("/world", f"/{follower}", rospy.Time(0))

            # Calculate the distance between turtles
            distance = math.sqrt((trans1[0] - trans2[0])**2 + (trans1[1] - trans2[1])**2)

            # Calculate the required linear speed to maintain target distance
            linear_speed = max_speed * (distance - target_distance)

            # Calculate the angle difference between turtles
            angle_diff = math.atan2(trans1[1] - trans2[1], trans1[0] - trans2[0]) - tf.transformations.euler_from_quaternion(rot2)[2]

            # Calculate the required angular speed to turn towards the leader
            angular_speed = 4.0 * angle_diff

            # Publish the velocities to the follower turtle
            cmd_vel = Twist()
            cmd_vel.linear.x = linear_speed
            cmd_vel.angular.z = angular_speed
            cmd_vel_publisher.publish(cmd_vel)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("leader_follower")

    # Define the names of the turtles
    turtles = ["turtle1", "turtle2"]

    # Define target distance and max speed
    target_distance = 2.0
    max_speed = 1.5

    leader_follower(turtles[0], turtles[1], target_distance, max_speed)
