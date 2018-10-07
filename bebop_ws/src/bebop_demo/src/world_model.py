#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose
import numpy as np
import rospy


class WorldModel(object):
    '''
    Contains all information of the world: the position of the obstacles, and
    the position and pose of the drone.
    '''

    def __init__(self):
        """
        Initialization of WorldModel object.
        """
        # Parameters.
        self.Ts = 0.01  # s
        self.max_vel = 0.4  # m/s
        self.max_accel = 0.2  # m/s²
        self.pose_bebop = Twist()
        self.xhat = Pose()
        self.Phat = np.zeros(3)
        self.X = np.zeros((3, 1))

        # Matrices for position Kalman filter
        self.A = np.identity(3)
        self.B = self.Ts
        self.C = np.identity(3)
        self.D = np.zeros(3)
        # self.pose_obst = Pose2D

    def predict_pos_update(self, vel_input):
        """
        Prediction step of the kalman filter. Update the position of the drone
        using the reference velocity commands.
        """
        # Q matrix contains the process noise covariance
        Q = np.array([[1e-2], [1e-2], [1e-2]])

        U = np.array([[vel_input.linear.x],
                     [vel_input.linear.y], [vel_input.linear.z]])
        self.X = self.A*self.X + self.B*U

        self.xhat.position.x = self.X[1, 1]
        self.xhat.position.y = self.X[2, 1]
        self.xhat.position.z = self.X[3, 1]

        self.Phat = self.A*self.Phat*self.A + Q

    def correct_pos_update(self, pos_meas):
        """
        Correction step of the kalman filter. Update the position of the drone
        using the measurements.
        """
        # R matrix contains the measurement noise covariance
        R = np.identity(3)

        Y = np.array([[pos_meas.linear.x],
                     [pos_meas.linear.y], [pos_meas.linear.z]])
        nu = Y - self.C*self.X

        S.
        S = self.C*self.Phat*self.C + R
        L = self.Phat*self.C*S**(-1)
        self.X = self.X + L*nu
        self.Phat = (np.identity(3) - L*self.C)*self.Phat

        self.xhat.position.x = self.X[1, 1]
        self.xhat.position.y = self.X[2, 1]
        self.xhat.position.z = self.X[3, 1]


if __name__ == '__main__':
    world_model = WorldModel()
