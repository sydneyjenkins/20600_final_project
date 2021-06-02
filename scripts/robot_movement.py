#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Empty
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties
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

        self.default_physics = True
        self.training = False
        self.disable_capture = True
        self.max_time = 30

        if self.training:
            self.genetic_algorithm = GeneticAlgorithm(self.max_time, load=True)
        
        self.kir_bot = None

        self.init_gazebo()

        rospy.sleep(2)

        self.reset_world()

        self.odom_positions = {}

        # run prey movement
        self.alec_bot = AlecBot(self.odom_positions)
        self.sydney_bot = SydneyBot(self.odom_positions)
        self.rachel_bot = RachelBot(self.odom_positions)

        if self.disable_capture:
            odom_pose_callback = lambda x: x
        else:
            self.predator_catch = PredatorCatch(self.max_time, self.reset_world)
            odom_pose_callback = self.predator_catch.get_odom_poses

        self.kir_bot = KirBot(self.odom_positions, odom_pose_callback)
        if self.training:
            self.kir_bot.params = self.genetic_algorithm.get_params()

        print("Initialized")
        self.initialized = True


    def init_gazebo(self):
        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_gazebo_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        rospy.wait_for_service('/gazebo/set_physics_properties')
        self.set_gazebo_physics_props = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

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
