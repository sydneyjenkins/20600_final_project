#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3

class RobotMovement(object):
    def __init__(self):
        rospy.init_node("robot_movement")
        self.initialized = False

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.wait_for_service('/gazebo/reset_world')
        reset = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        rospy.sleep(2)

        print("Initialized")
        self.initialized = True

        self.set_v(0, 0.5)

        rospy.sleep(5)
        reset()
        print("Reset")


    def set_v(self, velocity, angular_velocity):
        """ The current velocity and angular velocity of the robot are set here
        """
        v1 = Vector3(velocity, 0.0, 0.0)
        v2 = Vector3(0.0, 0.0, angular_velocity)
        t = Twist(v1, v2)
        self.cmd_vel_pub.publish(t)


    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = RobotMovement()
    node.run()
