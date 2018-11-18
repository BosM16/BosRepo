#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped, PointStamped
import numpy as np
import rospy
import tf2_ros


class WorldModel(object):
    '''
    Contains all available information of the world:
        - Drone dynamic model
        - Position of the obstacles
        - Position and pose of the drone
    Executes predictions and corrections on the drone position estimate using
    Kalman algorithm.
    '''

    def __init__(self):
        """
        Initialization of WorldModel object.
        """
        # Parameters.
        self.max_vel = rospy.get_param('motionplanner/vmax', 0.4)  # m/s
        self.max_accel = rospy.get_param('motionplanner/amax', 0.2)  # m/s**2
        self.drone_radius = rospy.get_param('motionplanner/drone_radius', 0.2)

        # Variables.
        self.xhat = PointStamped()
        self.xhat.header.frame_id = "world"
        self.xhat_r = PointStamped()
        self.xhat_r.header.frame_id = "world_rot"
        self.xhat_r_t0 = PointStamped()
        self.xhat_r_t0.header.frame_id = "world_rot"


if __name__ == '__main__':
    world_model = WorldModel()
