#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy, math
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
from bot import Bot

# How close we will get to wall.
distance = .4

class SydneyBot(Bot):

    def __init__(self, odom_positions, DEBUG=False):
        self.name = "sydney_bot"
        super().__init__(self.name, odom_positions)

        self.initialized = False

        self.DEBUG = DEBUG

        if self.DEBUG:
            rospy.init_node(self.name)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber(f'/{self.name}/camera/rgb/image_raw',
                Image, self.use_color_data)

        # subscribe to the robot's scan topic
        rospy.Subscriber(f"/{self.name}/scan", LaserScan, self.image_callback)
        # set up publisher and Twist to publish to /cmd_vel
        self.cmd_vel_pub = rospy.Publisher(f'/{self.name}/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        self.found_wall = False
        self.concave_turn = False
        self.obstacle = 9999
        self.following = False

        self.initialized = True


    def set_v(self, velocity, angular_velocity):
        """ The current velocity and angular velocity of the robot are set here
        """
        v1 = Vector3(velocity, 0.0, 0.0)
        v2 = Vector3(0.0, 0.0, angular_velocity)
        t = Twist(v1, v2)
        self.cmd_vel_pub.publish(t)


    def process_scan(self, data):
        self.twist.linear.x = -0.1
        self.cmd_vel_pub.publish(self.twist)
        return

    def use_color_data(self, data):
        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_color = numpy.array([ 0, 10, 0])
        upper_color = numpy.array([255, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)

        lower_orange = numpy.array([ 10, 100, 20])
        upper_orange = numpy.array([25, 255, 255])
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

        h, w, d = image.shape

        # using moments() function, the center of the colored pixels is determined
        M = cv2.moments(mask_orange)
        # if there are any colored pixels found
        if M['m00'] > 0:
                # center of the colored pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                err = w/2 - cx
                if err < 2:
                    self.obstacle = err
                    return

        self.obstacle = 9999
        return

    def image_callback(self, data):
                if not self.initialized:
                    return
                #angle_count = 0
                #min_angle = numpy.argmin(data.ranges)
                min_angle = numpy.argmin(np.asarray(data.ranges))#.index(min(distances))
                min_distance = data.ranges[min_angle]
                if not self.found_wall:# and (min_distance != 0 or self.obstacle<9999):
                    #print("not found wall")
                    if self.obstacle < 9999: 
                        #print("1")
                        turn = .2
                        self.set_v(0.2,turn)
                    elif min_distance < distance and min_angle>10 and min_angle<350: 
                        #print("2", min_distance, min_angle)
                        diff = min_angle-180
                        diff_abs = np.abs(np.abs(diff)-180)/180
                        turn = .5*diff_abs if diff<0 else -.5*diff
                        #print(diff_abs, turn)
                        self.set_v(0.0,turn)
                    elif min_distance < distance: 
                        #print("3")
                        self.set_v(0,0)
                        self.found_wall = True 
                    else: 
                        #print("4")
                        self.set_v(0.2,0)
                    '''if min_distance != 0:
                        self.turn_to_wall(min_distance)
                    else:
                        distances[min_distance] = 9999
                        min_distance = distances.index(min(distances))
                        self.turn_to_wall(min_distance)'''
                    return 
                else:
                    if self.following and self.concave_turn:
                        if min_angle>268 and min_angle < 275: 
                            self.concave_turn = False
                            #print("DONE TURNING")
                        elif self.turn_count > 50:
                            self.found_wall = False
                            self.concave_turn = False
                            self.following = False
                        else: 
                            distance_in_range = data.ranges[200:300]

                            # Turn left at
                            turn = -60*3.14159265/180
                            self.set_v(0.1, turn)
                            self.turn_count = self.turn_count+1
                    elif self.obstacle==9999 and self.following and data.ranges[0] <= distance+.1:
                        # Turn left
                        #print("sees inner corner")
                        turn = 30*3.14159265/180
                        self.set_v(0.05, turn)
                        rospy.sleep(1)
                    elif self.following and data.ranges[275] > distance+.4:
                        # prepare for convex turn 
                        self.concave_turn = True
                        self.turn_count = 0
                    else: 
                        # go straight
                        #print("going straight")
                        if min_angle < 267 and min_angle>90: 
                            #print("m1")
                        # If angle between robot and wall too small, adjust
                            #self.twist.linear.x = 0.
                            #current_angle = 0 
                            #if True: #while(current_angle < 5):
                                #t1 = rospy.Time.now().to_sec()
                                #current_angle = 5*3.14159625/180*(t1-t0)
                            turn = -.02*(270 - min_angle) #-3*3.14159265/180
                            self.set_v(0,turn)
                        elif min_angle>267 and min_angle<275: 
                            if data.ranges[270]>.2:
                                turn = -.02*(3) #-3*3.14159265/180
                                self.set_v(0.3,turn)
                                #print("m2")
                            else: 
                                err = min_angle - 270
                                turn = .001*err
                                self.set_v(0.3,0)#turn)
                                self.following = True
                        elif min_angle>=275: 
                        # If angle between robot and wall too large, adjust
                            #self.twist.linear.x = 0.
                            #print("m3")
                            turn = .006*(min_angle - 270) #3*3.14159265/180
                            self.set_v(0, turn)
                        else: 
                            #print("m4")
                            turn = .01*(min_angle + 90) #3*3.14159265/180
                            self.set_v(0, turn)

                            #self.set_v(0.2,0)
                    # If nearest wall was not actually an obstacle or robot
                    # go towards it 
                    '''if not self.found_wall:
                            if data.ranges[0] >= distance:
                                    # Move forward toward wall
                                    self.set_v(0.2,0) 
                            else: 
                                    # At wall, turn left 
                                    ang = -90*3.14159265/180
                                    self.set_v(0, ang)
                                    rospy.sleep(1)
                                    self.found_wall = True
                    else:
                            if self.concave_turn:
                                    # Turn left at
                                    ang = 90*3.14159265/180
                                    self.set_v(0, ang)
                                    rospy.sleep(1)
                                    self.concave_turn = False
                            if data.ranges[0] <= distance:
                                    # Turn left
                                    ang = -90*3.14159265/180
                                    self.set_v(0, ang)
                                    rospy.sleep(1)
                            elif data.ranges[270] > distance+.5:
                                    # prepare for convex turn 
                                    self.set_v(0.2,0)
                                    rospy.sleep(1)
                                    self.concave_turn = True
                            else: 
                                    self.set_v(0.2,0)'''
                return 
                        
    def turn_to_wall(self, min_distance):
        if min_distance < 3: 
            ang = min_distance*90*3.14159265/180
            #print("ANG:",ang)
            self.set_v(0, ang)
            #rospy.sleep(1)
            self.set_v(0,0)
            #rospy.sleep(4)
        else: 
            ang = -90*3.14159265/180
            self.set_v(0, ang)
            #rospy.sleep(1)
            self.set_v(0,0)
            #rospy.sleep(4)
        return 

    def restart_bot(self):
        self.found_wall = False
        self.concave_turn = False
        self.obstacle = 9999
        self.following = False
        return

    def run(self):
        if self.DEBUG:
            rospy.spin()


# this is for running the node manually for debugging and testing
if __name__ == '__main__':
    node = SydneyBot(DEBUG=True)
    node.run()
