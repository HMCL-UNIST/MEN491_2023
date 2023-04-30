#!/usr/bin/env python
import rospy
import tf
import math

def calculate_distance(turtle1, turtle2):
    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans1, rot1) = listener.lookupTransform("/world", f"/{turtle1}", rospy.Time(0))
            (trans2, rot2) = listener.lookupTransform("/world", f"/{turtle2}", rospy.Time(0))

            distance = math.sqrt((trans1[0] - trans2[0])**2 + (trans1[1] - trans2[1])**2)
            rospy.loginfo(f"Distance between {turtle1} and {turtle2}: {distance:.2f}")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rospy.sleep(1.0)

if __name__ == "__main__":
    rospy.init_node("turtle_distance_calculator")

    # Define the names of the turtles
    turtles = ["turtle1", "turtle2"]

    calculate_distance(turtles[0], turtles[1])
