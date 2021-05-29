#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel, SpawnModel

import math

class PredatorCatch(object):

    def __init__(self, reset_world, DEBUG=False):
        self.DEBUG = DEBUG

        if self.DEBUG:
            rospy.init_node("predator_catch")

        self.reset_world = reset_world
        self.last_reset_time = rospy.get_time()

        # rospy.Subscriber("/gazebo/model_states", ModelStates, self.get_models, queue_size=1)

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


    def handle_capture_test(self, predator_pose, prey_name, prey_pose):
        prey_dist = []

        for pose in prey_pose:
            dist = math.sqrt(pow(pose.position.x - predator_pose.position.x, 2) + pow(pose.position.y - predator_pose.position.y, 2))
            prey_dist.append(dist)

        for i in range(len(prey_dist)):
            if prey_dist[i] < 0.22:
                # prey has been captured
                print(f'Captured {prey_name[i]}')
                self.reset()
                # self.delete_proxy(prey_name[i])

                return


    def reset(self):
        self.last_reset_time = rospy.get_time()
        self.reset_world()
