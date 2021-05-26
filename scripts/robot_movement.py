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

class RobotMovement(object):
    def __init__(self):

        self.initialized = False

        rospy.init_node("robot_movement")

        self.init_gazebo()

        rospy.sleep(2)

        self.reset_world()

        self.odom_positions = {}

        # run prey movement
        self.alec_bot = AlecBot(self.odom_positions)
        self.sydney_bot = SydneyBot(self.odom_positions)
        self.rachel_bot = RachelBot(self.odom_positions)
        self.kir_bot = KirBot(self.odom_positions, self.handle_odom_positions)

        PredatorCatch()

        print("Initialized")
        self.initialized = True

        rospy.sleep(60)
        self.reset_world()
        print("Reset")


    def init_gazebo(self):
        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        rospy.wait_for_service('/gazebo/set_physics_properties')
        set_physics_props = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

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

        set_physics_props(0.001, 1000.0, gravity, ode_config)


    def handle_odom_positions(self):
        # reset the world if either 60s has passed or the predator is within a certain
        # range of a prey bot

        # print(self.odom_positions)
        return


    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = RobotMovement()
    node.run()
