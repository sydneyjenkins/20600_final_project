#!/usr/bin/env python3

import rospy, numpy

from geometry_msgs.msg import Twist, Vector3, Pose
from std_srvs.srv import Empty
from gazebo_msgs.msg import ODEPhysics, ModelState
from gazebo_msgs.srv import SetPhysicsProperties, SetModelState
from alec_bot import AlecBot
from sydney_bot import SydneyBot
from rachel_bot import RachelBot
from kir_bot import KirBot
from predator_catch import PredatorCatch
from genetic_algorithm import GeneticAlgorithm

class RobotMovement(object):
    def __init__(self):
        self.initialized = False

        rospy.init_node("robot_movement")

        self.default_physics = False
        self.training = True
        self.max_time = 30

        self.genetic_algorithm = GeneticAlgorithm(self.max_time, load=True)
        
        self.kir_bot = None

        self.spawn_locs = numpy.array([[0, 0, 0],
                            [0, -1, 0],
                            [0, -2, 0],
                            [0, -3, 0],
                            [1, 0, 0],
                            [1, -1, 0],
                            [1, -3, 0],
                            [2, 0, 0],
                            [2, -1, 0],
                            [2, -2, 0]])

        self.init_gazebo()

        rospy.sleep(2)

        self.reset_world()

        self.odom_positions = {}

        # run prey movement
        self.alec_bot = AlecBot(self.odom_positions)
        self.sydney_bot = SydneyBot(self.odom_positions)
        self.rachel_bot = RachelBot(self.odom_positions)

        self.predator_catch = PredatorCatch(self.max_time, self.reset_world)

        self.kir_bot = KirBot(self.odom_positions, self.predator_catch.get_odom_poses)
        if self.training:
            self.kir_bot.params = self.genetic_algorithm.get_params()

        print("Initialized")
        self.initialized = True


    def init_gazebo(self):
        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_gazebo_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        rospy.wait_for_service('/gazebo/set_physics_properties')
        self.set_gazebo_physics_props = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.set_physics_props()


    def set_physics_props(self):
        ode_config = ODEPhysics()
        ode_config.auto_disable_bodies = False
        ode_config.sor_pgs_precon_iters = 0
        ode_config.sor_pgs_iters = 50
        ode_config.sor_pgs_w = 1.3
        ode_config.sor_pgs_rms_error_tol = 0.0
        ode_config.contact_surface_layer = 0.001
        ode_config.contact_max_correcting_vel = 100.0
        ode_config.cfm = 0.0
        ode_config.erp = 0.2
        ode_config.max_contacts = 20
        gravity = Vector3(0.0, 0.0, -9.8)

        time_step = 0.001
        max_update_rate = 0.0
        if self.default_physics:
            time_step = 0.001
            max_update_rate = 1000.0
        self.set_gazebo_physics_props(time_step, max_update_rate, gravity, ode_config)


    def reset_world(self, score_time=None):
        # self.set_physics_props()
        self.reset_gazebo_world()

        if self.training:
            if score_time is not None:
                self.genetic_algorithm.set_score_by_time(score_time)

            if self.kir_bot is not None:
                self.kir_bot.params = self.genetic_algorithm.get_params()
            
            self.genetic_algorithm.print_progress()

        zero_twist = Twist()
        zero_twist.linear.x = 0
        zero_twist.linear.y = 0
        zero_twist.linear.z = 0
        zero_twist.angular.x = 0
        zero_twist.angular.y = 0
        zero_twist.angular.z = 0

        kir_state = ModelState()
        kir_pose = Pose()
        kir_pose.orientation.x = 0
        kir_pose.orientation.y = 0
        kir_pose.orientation.z = 0
        kir_pose.orientation.w = 0
        kir_state.model_name = 'kir_bot'

        rachel_state = ModelState()
        rachel_pose = Pose()
        rachel_pose.orientation.x = 0
        rachel_pose.orientation.y = 0
        rachel_pose.orientation.z = 0
        rachel_pose.orientation.w = 0
        rachel_state.model_name = 'rachel_bot'

        sydney_state = ModelState()
        sydney_pose = Pose()
        sydney_pose.orientation.x = 0
        sydney_pose.orientation.y = 0
        sydney_pose.orientation.z = 0
        sydney_pose.orientation.w = 0
        sydney_state.model_name = 'sydney_bot'

        alec_state = ModelState()
        alec_pose = Pose()
        alec_pose.orientation.x = 0
        alec_pose.orientation.y = 0
        alec_pose.orientation.z = 0
        alec_pose.orientation.w = 0
        alec_state.model_name = 'alec_bot'
        
        spawn_index = numpy.random.choice(self.spawn_locs.shape[0], 4, False)

        kir_pose.position.x = self.spawn_locs[spawn_index[0]][0]
        kir_pose.position.y = self.spawn_locs[spawn_index[0]][1]
        kir_pose.position.z = self.spawn_locs[spawn_index[0]][2]

        rachel_pose.position.x = self.spawn_locs[spawn_index[1]][0]
        rachel_pose.position.y = self.spawn_locs[spawn_index[1]][1]
        rachel_pose.position.z = self.spawn_locs[spawn_index[1]][2]

        sydney_pose.position.x = self.spawn_locs[spawn_index[2]][0]
        sydney_pose.position.y = self.spawn_locs[spawn_index[2]][1]
        sydney_pose.position.z = self.spawn_locs[spawn_index[2]][2]

        alec_pose.position.x = self.spawn_locs[spawn_index[3]][0]
        alec_pose.position.y = self.spawn_locs[spawn_index[3]][1]
        alec_pose.position.z = self.spawn_locs[spawn_index[3]][2]

        kir_state.pose = kir_pose
        rachel_state.pose = rachel_pose
        sydney_state.pose = sydney_pose
        alec_state.pose = alec_pose

        kir_state.twist = zero_twist
        rachel_state.twist = zero_twist
        sydney_state.twist = zero_twist
        alec_state.twist = zero_twist

        self.set_model_state(kir_state)
        self.set_model_state(rachel_state)
        self.set_model_state(sydney_state)
        self.set_model_state(alec_state)

        try:
            self.sydney_bot.restart_bot()
            self.alec_bot.restart_bot()
        except: 
            #self.bots not yet initialized
            pass


    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = RobotMovement()
    node.run()
