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
        self.prey_captured = False

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.get_models, queue_size=1)

        rospy.wait_for_service('/gazebo/delete_model')
        self.delete_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    def get_models(self, data):
        predator_pose = Pose()
        prey_name = []
        prey_pose = []
        prey_dist = []

        for i in range(len(data.name)):
            name = data.name[i]
            if name == "kir_bot":
                predator_pose = data.pose[i]
            elif name == "rachel_bot" or name == "alec_bot" or name == "sydney_bot":
                prey_name.append(name)
                prey_pose.append(data.pose[i])

        for pose in prey_pose:
            dist = math.sqrt(pow(pose.position.x - predator_pose.position.x, 2) + pow(pose.position.y - predator_pose.position.y, 2))
            prey_dist.append(dist)

        if self.prey_captured == False:
            for i in range(len(prey_dist)):
                if prey_dist[i] < 0.21:
                    # prey has been captured
                    print(f'Captured {prey_name[i]}')
                    self.prey_captured = True
                    self.reset_world()
                    rospy.sleep(0.01)
                    self.prey_captured = False
                    # self.delete_proxy(prey_name[i])

                    return
