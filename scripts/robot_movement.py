#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Empty
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties

class RobotMovement(object):
    def __init__(self):
        rospy.init_node("robot_movement")
        self.initialized = False

        self.cmd_vel_pub = rospy.Publisher('/sydney_bot/cmd_vel', Twist, queue_size=10)

        self.init_gazebo()

        rospy.sleep(2)

        print("Initialized")
        self.initialized = True

        self.set_v(0.5, 0.5)

        rospy.sleep(60)
        self.reset()
        print("Reset")


    def init_gazebo(self):
        rospy.wait_for_service('/gazebo/reset_world')
        self.reset = rospy.ServiceProxy('/gazebo/reset_world', Empty)

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

        set_physics_props(0.002, 0.0, gravity, ode_config)


    def set_v(self, velocity, angular_velocity):
        """ The current velocity and angular velocity of the robot are set here
        """
        v1 = Vector3(velocity, 0.0, 0.0)
        v2 = Vector3(0.0, 0.0, angular_velocity)
        t = Twist(v1, v2)
        self.cmd_vel_pub.publish(t)


    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = RobotMovement()
    node.run()
