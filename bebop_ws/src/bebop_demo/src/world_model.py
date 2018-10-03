#!/usr/bin/env python

from geometry_msgs.msg import Twist
import numpy as np
import rospy


class WorldModel(object):
    '''
    Contains all information of the world: the position of the obstacles, and
    the position and pose of the drone.
    '''
# AANPASSEN VOOR MEERDERE RICHTINGEN!!
    def __init__(self):
        """
        Initialization of WorldModel object.
        """
        # Parameters.
        self.Ts = 0.01 # s
        self.max_vel = 0.4  # m/s
        self.max_accel = 0.2  # m/s²
        self.pose_bebop = Twist()
        self.xhat = 0.0
        self.Phat = 0.0

        # Matrices for position Kalman filter
        self.A = 1.0
        self.B = self.Ts
        self.C = 1.0
        self.D = 0.0
        # self.pose_obst = Pose2D

    def predict_pos_update(self, vel_input):
        """
        Prediction step of the kalman filter. Update the position of the drone
        using the reference velocity commands.
        """
        # Q matrix contains the process noise covariance
        Q = 10**(-2)
        self.xhat = A*xhat + B*vel_input
        self.Phat = A*Phat*A + Q

    def correct_pos_update(self, pos_meas_x, pos_meas_y, pos_meas_z):
        """
        Correction step of the kalman filter. Update the position of the drone
        using the measurements.
        """
        # R matrix contains the measurement noise covariance
        R = 1
        nu = pos_meas - C*self.xhat
        S = C*Phat*C + R
        L = Phat*C*S**(-1)
        self.xhat = L*nu
        self.Phat = (1-L*C)*self.Phat


if __name__ == '__main__':
    world_model = WorldModel()
