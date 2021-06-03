#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
import numpy as np
from bot import Bot

class KirBot(Bot):

    def __init__(self, odom_positions, handle_odom_positions, DEBUG=False):
        super().__init__("kir_bot", odom_positions, handle_odom_positions)

        self.initialized = False

        self.DEBUG = DEBUG

        if self.DEBUG:
            rospy.init_node("kir_bot")

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('/kir_bot/camera/rgb/image_raw',
                Image, self.image_callback, queue_size=10)

        # subscribe to the robot's scan topic
        rospy.Subscriber("/kir_bot/scan", LaserScan, self.process_scan, queue_size=10)

        # set up publisher and Twist to publish to /cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/kir_bot/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        self.max_speed = 2

        self.params = {
            "prey_weight": 1, # points straight towards nearest prey
            "parallel_weight": 0, # points parallel to nearest obstacle
            # "max_parallel_drive_scaled_weight": 0,
            "away_weight": .2, # points straight away from nearest obstacle
            # "explore_weight": 0, # vector pointing to unexplored areas based on odom

            "prey_only_pixel_percent": 0.5,

            "min_turn_only_angle": 0.4,
            "base_speed": 0.15,
            "scaled_speed": 0, # scales with angle err (lower err = higher speed)
            "angle_adjust_rate": 0.5
        }

        self.away_angle = 0
        self.parallel_angle = 0
        self.prey_angle = 0
        self.pixel_percent = 0

        self.initialized = True


    def set_v(self, velocity, angular_velocity):
        """ The current velocity and angular velocity of the robot are set here
        """
        v1 = Vector3(velocity, 0.0, 0.0)
        v2 = Vector3(0.0, 0.0, angular_velocity)
        t = Twist(v1, v2)
        self.cmd_vel_pub.publish(t)


    def process_scan(self, data):
        if not self.initialized:
            return
        # self.twist.angular.z = 0.5
        # self.cmd_vel_pub.publish(self.twist)

        min_dist = None
        min_dist_index = None
        for i in range(len(data.ranges)):
            dist = data.ranges[i]
            if dist == math.inf:
                continue

            if min_dist is None or dist < min_dist:
                min_dist = dist
                min_dist_index = i

        away_angle = 0
        parallel_angle = 0

        if min_dist is not None:
            away_angle = (min_dist_index / 360) * 2 * math.pi + math.pi

            if min_dist_index < 180:
                # left
                diff = min_dist_index - 90
            else:
                # right
                diff = min_dist_index - 270
            parallel_angle = diff / 360 * 2 * math.pi

        away_angle = self.normalize_radian_angle(away_angle)
        parallel_angle = self.normalize_radian_angle(parallel_angle)

        self.away_angle = away_angle
        self.parallel_angle = parallel_angle
        # target_angle = parallel_angle
        self.move_robot()

    def move_robot(self):
        target_angle = self.prey_angle * self.params["prey_weight"]
        use_other_weights = True

        if self.pixel_percent > self.params["prey_only_pixel_percent"]:
            use_other_weights = False

        if use_other_weights:
            target_angle += self.away_angle * self.params["away_weight"] + self.parallel_angle * self.params["parallel_weight"]

        target_angle = self.normalize_radian_angle(target_angle)

        abs_err = abs(target_angle)

        err_handling_speed = self.params["base_speed"] * self.max_speed + self.params["scaled_speed"] / (abs_err + .0001)
        speed = min(self.max_speed, err_handling_speed)
        if abs_err > (self.params["min_turn_only_angle"] * 3):
            speed = 0
    
        angular_speed = target_angle * self.params["angle_adjust_rate"]
        self.set_v(speed, angular_speed)

    def normalize_degree_angle(self, angle):
        new_angle = angle
        while (new_angle <= -180):
            new_angle += 360
        while (new_angle > 180):
            new_angle -= 360
        return new_angle

    def normalize_radian_angle(self, angle):
        return math.radians(self.normalize_degree_angle(math.degrees(angle)))

    def image_callback(self, msg):
        if not self.initialized:
            return
        
        self.prey_angle = math.pi * 0.5

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        h, w, d = image.shape

        # lower_bounds = np.array([0, 121/2, 241/2]) 
        # upper_bounds = np.array([20, 180/2, 300/2])
        # rgb_lower = [np.asarray([lower_bounds[i],20, 20]) for i in range(3)]
        # rgb_upper = [np.asarray([upper_bounds[i],255, 255]) for i in range(3)]

        lower_blue = np.array([235/2, 230, 230])
        upper_blue = np.array([245/2, 255, 255])

        lower_green = np.array([115/2, 230, 230])
        upper_green = np.array([125/2, 255, 255])

        lower_yellow = np.array([55/2, 230, 230])
        upper_yellow = np.array([65/2, 255, 255])
        
        lower_bounds = [lower_blue, lower_green, lower_yellow]
        upper_bounds = [upper_blue, upper_green, upper_yellow]

        best_err_to_approx_angle = None

        self.pixel_percent = 0

        for i in range(len(lower_bounds)):
            lower = lower_bounds[i]
            upper = upper_bounds[i]

            mask = cv2.inRange(hsv, lower, upper)

            M = cv2.moments(mask)
            # if there are any colored pixels found
            if M['m00'] > 0:
                    pixel_percent = (mask>0).mean()
                    self.pixel_percent = min(1, max(pixel_percent, self.pixel_percent))
                    # center of the colored pixels in the image
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    err = (w / 2 - cx)
                    err_to_approx_angle =  err / w * math.pi * 0.5
                    if best_err_to_approx_angle is None or abs(err_to_approx_angle) < abs(best_err_to_approx_angle):
                        best_err_to_approx_angle = err_to_approx_angle
            else:
                continue
                # # spin if the goal color is not visible
                # self.set_v(0, .4)

        if best_err_to_approx_angle is not None:
            self.prey_angle = best_err_to_approx_angle


    def run(self):
        if self.DEBUG:
            rospy.spin()


# this is for running the node manually for debugging and testing
if __name__ == '__main__':
    node = KirBot(DEBUG=True)
    node.run()
