#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel, SpawnModel
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import math

class PredatorCatch(object):

    def __init__(self, reset_world, DEBUG=False):
        self.DEBUG = DEBUG

        if self.DEBUG:
            rospy.init_node("predator_catch")

        self.reset_world = reset_world
        self.last_reset_time = rospy.get_time()

        self.last_dist_print = rospy.get_time()

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.get_models, queue_size=1)

        rospy.wait_for_service('/gazebo/delete_model')
        self.delete_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)


    def handle_reset_timer(self):
        now = rospy.get_time()
        time_diff = now - self.last_reset_time
        if time_diff < .5:
            return True
        elif time_diff >= 60:
            print('60s passed with no capture')
            self.reset()
            return True

        return False


    def get_models(self, data):
        if self.handle_reset_timer():
            return

        predator_pose = Pose()
        prey_name = []
        prey_pose = []

        for i in range(len(data.name)):
            name = data.name[i]
            if self.is_predator_name(name):
                predator_pose = data.pose[i]
            elif self.is_prey_name(name):
                prey_name.append(name)
                prey_pose.append(data.pose[i])

        self.handle_capture_test(predator_pose, prey_name, prey_pose)


    def get_odom_poses(self, odom_poses):
        if self.handle_reset_timer():
            return

        predator_pose = Pose()
        prey_name = []
        prey_pose = []

        for name in odom_poses:
            pose = odom_poses[name]
            if self.is_predator_name(name):
                predator_pose = pose
            elif self.is_prey_name(name):
                prey_name.append(name)
                prey_pose.append(pose)

        self.handle_capture_test(predator_pose, prey_name, prey_pose)


    def is_predator_name(self, name):
        return name == "kir_bot"


    def is_prey_name(self, name):
        return name == "rachel_bot" or name == "alec_bot" or name == "sydney_bot"


    def get_yaw_from_pose(self, p):

        yaw = (euler_from_quaternion([
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w])
                [2])
        return yaw


    def actual_pose(self, p, log=False):
        x = p.position.x
        y = p.position.y
        yaw = self.get_yaw_from_pose(p)
        l = .06
        dx = l * math.cos(yaw)
        dy = l * math.sin(yaw)
        if log:
            print(dx, dy)

        return x - dx, y - dy


    def handle_capture_test(self, predator_pose, prey_name, prey_pose):
        prey_dist = []

        pred_x, pred_y = self.actual_pose(predator_pose)

        for pose in prey_pose:
            prey_x, prey_y = self.actual_pose(pose)
            dist = math.sqrt(pow(prey_x - pred_x, 2) + pow(prey_y - pred_y, 2))
            prey_dist.append(dist)

        now = rospy.get_time()
        if now - self.last_dist_print > 1:
            self.last_dist_print = now

        for i in range(len(prey_dist)):
            # if prey_name[i] == 'alec_bot':
            #     self.actual_pose(prey_pose[i], True)

            if prey_dist[i] < 0.32:
                # prey has been captured
                print(f'Captured {prey_name[i]}')
                self.reset()
                # self.delete_proxy(prey_name[i])

                return


    def reset(self):
        self.last_reset_time = rospy.get_time()
        self.reset_world()
