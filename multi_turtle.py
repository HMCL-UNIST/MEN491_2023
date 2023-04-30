#!/usr/bin/env python
import rospy
import tf
from turtlesim.msg import Pose

def pose_callback(pose, turtle_name):
    x, y, theta = pose.x, pose.y, pose.theta

    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, theta),
                     rospy.Time.now(),
                     turtle_name,
                     "world")

def create_turtle_subscriber(turtle_name):
    rospy.Subscriber(f"/{turtle_name}/pose", Pose, pose_callback, callback_args=turtle_name)

if __name__ == "__main__":
    rospy.init_node("multi_turtle_tf_broadcaster")

    # Define the names of the turtles
    turtles = ["turtle1", "turtle2"]

    # Spawn a new turtle
    rospy.wait_for_service("spawn")
    spawn_turtle = rospy.ServiceProxy("spawn", turtlesim.srv.Spawn)
    spawn_turtle(5, 5, 0, "turtle2")

    # Subscribe to the turtles' pose topics
    for turtle_name in turtles:
        create_turtle_subscriber(turtle_name)

    rospy.spin()
