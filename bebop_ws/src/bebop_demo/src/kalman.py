#!/usr/bin/env python

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty

import rospy
import tf2_ros
import tf2_geometry_msgs as tf2_geom


class Kalman(object):

    def __init__(self):
        '''
        Asynchronous kalman filter to estimate position.
        '''

        self.case5 = False

        self.vel_cmd_list = []
        self.vel_list_corr = []

        self.X_r = np.zeros(shape=(8, 1))
        self.X_r_t0 = np.zeros(shape=(8, 1))
        self.vel_cmd_Ts = rospy.get_param('vel_cmd/sample_time', 0.01)  # s

        self.Phat_t0 = np.zeros(8)
        self.Phat = np.zeros(8)

        # Kalman tuning parameters.
        self.R = np.identity(3)  # measurement noise covariance
        self.Q = 1e-1*np.identity(8)  # process noise covariance

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.initialize_model()

    def initialize_model(self):
        '''Initializes the model to be used in the Kalman filter.
        State space model x'(t) = A*x(t) + B*u(t) in observable canonical
        form, corresponding to transfer function

                  b2*s^2 + b1*s + b0
        G(s) = --------------------------
                s^3 + a2*s^2 + a1*s + a0

        State space model matrices for position Kalman filter are in
        continuous time!! Are then converted to discrete time further on
        depending on varying Ts.
        '''

        a2x = 5.458
        a1x = 2.467
        a0x = -0.03519
        Ax = np.array([[-a2x, 1., 0.],
                       [-a1x, 0., 1.],
                       [-a0x, 0., 0.]])
        a2y = 5.921
        a1y = 2.469
        a0y = 0.02513
        Ay = np.array([[-a2y, 1., 0.],
                       [-a1y, 0., 1.],
                       [-a0y, 0., 0.]])
        a1z = 1.515
        a0z = 0.02072
        Az = np.array([[-a1z, 1.],
                       [-a0z, 0.]])

        self.A = np.zeros([8, 8])  # continuous A matrix
        self.A[0:3, 0:3] = Ax
        self.A[3:6, 3:6] = Ay
        self.A[6:8, 6:8] = Az

        b2x = -0.01447
        b1x = -1.008
        b0x = 18.32
        Bx = np.array([b2x, b1x, b0x])
        b2y = -0.008856
        b1y = -1.063
        b0y = 19.74
        By = np.array([b2y, b1y, b0y])
        b1z = 0.6266
        b0z = 1.597
        Bz = np.array([b1z, b0z])
        self.B = np.zeros([8, 3])  # continuous B matrix
        self.B[0:3, 0] = Bx
        self.B[3:6, 1] = By
        self.B[6:8, 2] = Bz

        self.C = np.zeros([3, 8])
        self.C[0, 0] = 1
        self.C[1, 3] = 1
        self.C[2, 6] = 1

    def kalman_pos_predict(self, vel_cmd, xhat_r):
        '''
        Based on the velocity commands send out by the velocity controller,
        calculate a prediction of the position in the future.
        Arguments:
            vel_cmd: TwistStamped
        '''
        if not self.init:
            (self.X_r, xhat_r) = self.wm.predict_step_calc(
                vel_cmd, self.vel_cmd_Ts, self.X_r)
        return xhat_r

    def kalman_pos_correct(self, measurement, xhat_r_t0):
        '''
        Whenever a new position measurement is available, sends this
        information to the perception and then triggers the kalman filter
        to apply a correction step.
        Cases:
            - case 1: multiple velocity commands between successive
                      measurements
            - case 2: only one velocity command between successive
                      measurements
            - case 3: no velocity command between successive measurements
            - case 4: last velocity command corresponds to time after new
                      measurement, but due to delay is already in the list.
            - case 5: last velocity command before measurement is not yet in
                      the list and appears only in the next correction step.
        Arguments:
            measurement_world: PoseStamped expressed in "world" frame.
        '''

        if self.init:
            self.X_r_t0[0, 0] = measurement.point.x
            self.X_r_t0[3, 0] = measurement.point.y
            self.X_r_t0[6, 0] = measurement.point.z
            self.X_r = self.X_r_t0

        if not self.init:
            self.vel_list_corr = self.vel_list_corr + self.vel_cmd_list
            self.vel_cmd_list = []

            if (len(self.vel_list_corr) > 1) and self.get_time_diff(
                    measurement, self.vel_list_corr[1]) < 0:
                self.vel_list_corr = self.vel_list_corr[1:]
            self.case5 = False

            # First make prediction from old point t0 to last point t before
            # new measurement.
            # Calculate variable B (time between latest prediction and new t0).
            time_diff_check = self.get_time_diff(
                measurement, self.latest_vel_cmd)
            late_cmd_vel = []

            # Check for case 4.
            if time_diff_check < 0:
                late_cmd_vel = [self.vel_list_corr[-1]]
                self.vel_list_corr = self.vel_list_corr[0:-1]

            vel_len = len(self.vel_list_corr)
            if (vel_len > 1):
                case3 = False
                Ts = self.get_time_diff(
                    self.vel_list_corr[1], self.xhat_r_t0)
            else:
                case3 = True
                Ts = self.get_time_diff(
                    measurement, self.wm.xhat_r_t0)

            (X, xhat_r) = self.predict_step_calc(
                    self.vel_list_corr[0], Ts, self.X_r_t0)

            # If not case 2 or 3 -> need to predict up to
            # last vel cmd before new_t0
            if vel_len > 2:
                # Case 1.
                for i in range(vel_len - 2):
                    Ts = self.get_time_diff(
                        self.vel_list_corr[i+2], self.vel_list_corr[i+1])
                    (X, xhat_r) = self.predict_step_calc(
                        self.vel_list_corr[i+1], Ts, X)

            # Now make prediction up to new t0 if not case 3.
            B = self.get_time_diff(self.pc.pose_vive,
                                   self.vel_list_corr[-1])
            if B > self.vel_cmd_Ts:
                self.case5 = True
            if not case3:
                (X, xhat_r) = self.predict_step_calc(
                    self.latest_vel_cmd, B, X)

            # ---- CORRECTION ----
            # Correct the estimate at new t0 with the measurement.
            (X, xhat_r_t0) = self.correct_step_calc(measurement, X, xhat_r)
            self.X_r_t0 = X
            xhat_r_t0.header.stamp = measurement.header.stamp

            # Now predict until next point t that coincides with next timepoint
            # for the controller.

            (X, xhat_r) = self.predict_step_calc(
                                    self.vel_list_corr[-1],
                                    (1 + self.case5)*self.vel_cmd_Ts - B, X)

            # Save variable globally
            self.X_r = X

            self.vel_list_corr = [self.vel_list_corr[-1]]
            self.vel_list_corr += late_cmd_vel

        return xhat_r, xhat_r_t0

    def predict_step_calc(self, vel_cmd_stamped, Ts, X):
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

    def correct_step_calc(self, pos_meas, X, xhat_r):
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

    def transform_pose(self, pose, _from, _to):
        '''Transforms pose (geometry_msgs/PoseStamped) from frame "_from" to
        frame "_to".
        Arguments:
            - _from, _to = string, name of frame
        '''
        transform = self.get_transform(_from, _to)
        pose_tf = tf2_geom.do_transform_pose(pose, transform)
        pose_tf.header.stamp = pose.header.stamp
        pose_tf.header.frame_id = _to

        return pose_tf

    def get_transform(self, _from, _to):
        '''Returns the TransformStamped msg of the transform from reference
        frame '_from' to reference frame '_to'
        Arguments:
            - _from, _to = string, name of frame
        '''
        tf_f_in_t = self.tfBuffer.lookup_transform(
            _to, _from, rospy.Time(0), rospy.Duration(0.1))

        return tf_f_in_t
