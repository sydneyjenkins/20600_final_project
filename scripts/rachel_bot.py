#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

class RachelBot(object):

    def __init__(self, DEBUG=False):

        self.initialized = False

        self.DEBUG = DEBUG

        if self.DEBUG:
            rospy.init_node("rachel_bot")

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('/rachel_bot/camera/rgb/image_raw',
                Image, self.image_callback)

        # subscribe to the robot's scan topic
        rospy.Subscriber("/rachel_bot/scan", LaserScan, self.process_scan)

        # set up publisher and Twist to publish to /cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/rachel_bot/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        self.initialized = True


    def set_v(self, velocity, angular_velocity):
        """ The current velocity and angular velocity of the robot are set here
        """
        v1 = Vector3(velocity, 0.0, 0.0)
        v2 = Vector3(0.0, 0.0, angular_velocity)
        t = Twist(v1, v2)
        self.cmd_vel_pub.publish(t)


    def process_scan(self, data):
        self.twist.linear.x = 0.1
        self.cmd_vel_pub.publish(self.twist)
        return


    def image_callback(self, data):
        return


    def run(self):
        if self.DEBUG:
            rospy.spin()


# this is for running the node manually for debugging and testing
if __name__ == '__main__':
    node = RachelBot(DEBUG=True)
    node.run()
