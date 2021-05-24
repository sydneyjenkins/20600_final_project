#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel, SpawnModel

import math

class predatorCatch(object):

    def __init__(self, DEBUG=False):
        rospy.init_node("predatorCatch")

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.modelsGot)

        self.delete_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    def modelsGot(self, data):
        predator_pose = Pose()
        prey_name = []
        prey_pose = []
        prey_dist = []

        for i in range(len(data.name)):
            if data.name[i] == "kir_bot":
                predator_pose = data.pose[i]
            else:
                prey_name.append(data.name)
                prey_pose.append(data.pose[i])

        for pose in prey_pose:
            dist = math.sqrt(pow(pose.x - predator_pose.x, 2) + pow(pose.y - predator_pose.y, 2) + pow(pose.z - predator_pose.z, 2))
            prey_dist.append(dist)

        for i in range(len(prey_dist)):
            if prey_dist[i] < 0.1:
                self.delete_proxy(prey_name[i])