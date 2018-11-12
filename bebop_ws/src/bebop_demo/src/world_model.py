#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped, Pose, Point, PointStamped
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
        self.vel_cmd_Ts = rospy.get_param('vel_cmd/sample_time', 0.01)  # s
        self.max_vel = rospy.get_param('motionplanner/vmax', 0.4)  # m/s
        self.max_accel = rospy.get_param('motionplanner/amax', 0.2)  # m/s**2
        self.drone_radius = rospy.get_param('motionplanner/drone_radius', 0.2)

        # Kalman tuning parameters.
        self.R = np.identity(3)  # measurement noise covariance
        self.Q = 1e-1*np.identity(8)  # process noise covariance

        # Variables.
        self.xhat = PointStamped()
        self.xhat.header.frame_id = "world"

        self.X_r = np.zeros(shape=(8, 1))
        self.X_r_t0 = np.zeros(shape=(8, 1))
        self.xhat_r = PointStamped()
        self.xhat_r.header.frame_id = "world_rot"
        self.xhat_r_t0 = PointStamped()
        self.xhat_r_t0.header.frame_id = "world_rot"

        self.Phat_t0 = np.zeros(8)
        self.Phat = np.zeros(8)

        # Transformations.
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.initialize_model()

    def initialize_model(self):
        '''Initializes The model to be used in the Kalman filter.
        State space model x'(t) = A*x(t) + B*u(t) in observable canonical
        form, corresponding to transfer function

                  d1*s^2 + d2*s + d3
        G(s) = --------------------------
                s^3 + n1*s^2 + n2*s + n3

        State space model matrices for position Kalman filter are in
        continuous time!! Are then converted to discrete time further on
        depending on varying Ts.
        '''

        a2x = 5.208
        a1x = 2.65
        a0x = -0.05182
        Ax = np.array([[-a2x, 1., 0.],
                       [-a1x, 0., 1.],
                       [-a0x, 0., 0.]])
        a2y = 6.557
        a1y = 3.781
        a0y = -0.08711
        Ay = np.array([[-a2y, 1., 0.],
                       [-a1y, 0., 1.],
                       [-a0y, 0., 0.]])
        a1z = 4.941
        a0z = 0.01532
        Az = np.array([[-a1z, 1.],
                       [-a0z, 0.]])

        self.A = np.zeros([8, 8])  # continuous A matrix
        self.A[0:3, 0:3] = Ax
        self.A[3:6, 3:6] = Ay
        self.A[6:8, 6:8] = Az

        b2x = 0.02308
        b1x = -0.8863
        b0x = 18.04
        Bx = np.array([b2x, b1x, b0x])
        b2y = 0.02634
        b1y = -1.764
        b0y = 21.8
        By = np.array([b2y, b1y, b0y])
        b1z = 0.09622
        b0z = 4.886
        Bz = np.array([b1z, b0z])
        self.B = np.zeros([8, 3])  # continuous B matrix
        self.B[0:3, 0] = Bx
        self.B[3:6, 1] = By
        self.B[6:8, 2] = Bz

        self.C = np.zeros([3, 8])
        self.C[0, 0] = 1
        self.C[1, 3] = 1
        self.C[2, 6] = 1

    def predict_pos_update(self, vel_cmd_stamped, Ts, X):
        """
        Prediction step of the kalman filter. Update the position of the drone
        using the reference velocity commands.
        Arguments:
            - vel_cmd_stamped = TwistStamped
            - Ts = varying step size over which to integrate.
        """
        vel_cmd = vel_cmd_stamped.twist

        u = np.array([[vel_cmd.linear.x],
                      [vel_cmd.linear.y],
                      [vel_cmd.linear.z]])

        X = (np.matmul(Ts*self.A + np.identity(8), X)
             + np.matmul(Ts*self.B, u))

        xhat_r = PointStamped()
        xhat_r.header.frame_id = "world_rot"
        xhat_r.point.x = X[0, 0]
        xhat_r.point.y = X[3, 0]
        xhat_r.point.z = X[6, 0]

        self.Phat = np.matmul(Ts*self.A + np.identity(8), np.matmul(
            self.Phat, np.transpose(Ts*self.A + np.identity(8)))) + self.Q

        return X, xhat_r

    def correct_pos_update(self, pos_meas, X, xhat_r):
        """
        Correction step of the kalman filter. Update the position of the drone
        using the measurements.
        Argument:
            - pos_meas = PoseStamped expressed in "world_rot" frame.
        """
        y = np.array([[pos_meas.pose.position.x],
                      [pos_meas.pose.position.y],
                      [pos_meas.pose.position.z]])

        nu = y - np.matmul(self.C, X)
        S = np.matmul(self.C, np.matmul(
            self.Phat, np.transpose(self.C))) + self.R
        L = np.matmul(self.Phat, np.matmul(
            np.transpose(self.C), np.linalg.inv(S)))
        X = X + np.matmul(L, nu)
        self.Phat = np.matmul(
            (np.identity(8) - np.matmul(L, self.C)), self.Phat)

        xhat_r.point.x = X[0, 0]
        xhat_r.point.y = X[3, 0]
        xhat_r.point.z = X[6, 0]

        return X, xhat_r

    def get_timestamp(self, stamped_var):
        '''Returns the timestamp of 'stamped_var' (any stamped msg, eg.
        PoseStamped, Pointstamped, TransformStamped,...) in seconds.
        '''
        time = float(stamped_var.header.stamp.to_sec())

        return time

    def get_time_diff(self, stamp1, stamp2):
        '''Returns the difference between to timestamped messages (any stamped
        msg, eg. PoseStamped, Pointstamped, TransformStamped,...) in seconds.
        '''
        time_diff = self.get_timestamp(stamp1) - self.get_timestamp(stamp2)

        return time_diff


if __name__ == '__main__':
    world_model = WorldModel()
