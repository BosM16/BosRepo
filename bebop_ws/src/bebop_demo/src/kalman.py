#!/usr/bin/env python

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from std_msgs.msg import Empty

import rospy
import tf2_ros
import tf2_geometry_msgs as tf2_geom

import numpy as np


class Kalman(object):

    def __init__(self, A, B, C):
        '''
        Asynchronous kalman filter to estimate position.
        '''

        self.case5 = False

        # Assign model matrices
        self.A = A
        self.B = B
        self.C = C

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

    def kalman_pos_predict(self, vel_cmd, xhat_r):
        '''
        Based on the velocity commands send out by the velocity controller,
        calculate a prediction of the position in the future.
        Arguments:
            vel_cmd: TwistStamped
        '''
        if not self.init:
            (self.X_r, xhat_r, self.Phat) = self.predict_step_calc(
                vel_cmd, self.vel_cmd_Ts, self.X_r, self.Phat)
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
            self.X_r_t0[0, 0] = measurement.pose.position.x
            self.X_r_t0[3, 0] = measurement.pose.position.y
            self.X_r_t0[6, 0] = measurement.pose.position.z
            self.X_r = self.X_r_t0
            xhat_r = xhat_r_t0

        if not self.init:
            self.vel_list_corr = self.vel_list_corr + self.vel_cmd_list
            self.vel_cmd_list = []

            if (len(self.vel_list_corr) > 1) and self.get_time_diff(
                    measurement, self.vel_list_corr[1]) > 0:
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
                    self.vel_list_corr[1], xhat_r_t0)
            else:
                case3 = True
                Ts = self.get_time_diff(
                    measurement, xhat_r_t0)

            (X, xhat_r, Phat) = self.predict_step_calc(
                    self.vel_list_corr[0], Ts, self.X_r_t0, self.Phat_t0)

            # If not case 2 or 3 -> need to predict up to
            # last vel cmd before new_t0
            if vel_len > 2:
                # Case 1.
                for i in range(vel_len - 2):
                    Ts = self.get_time_diff(
                        self.vel_list_corr[i+2], self.vel_list_corr[i+1])
                    (X, xhat_r, Phat) = self.predict_step_calc(
                        self.vel_list_corr[i+1], Ts, X, Phat)

            # Now make prediction up to new t0 if not case 3.
            B = self.get_time_diff(measurement,
                                   self.vel_list_corr[-1])
            if B > self.vel_cmd_Ts:
                self.case5 = True
            if not case3:
                (X, xhat_r, Phat) = self.predict_step_calc(
                    self.latest_vel_cmd, B, X, Phat)

            # ---- CORRECTION ----
            # Correct the estimate at new t0 with the measurement.
            (X, xhat_r_t0, Phat) = self.correct_step_calc(measurement, X, xhat_r, Phat)
            self.X_r_t0 = X
            xhat_r_t0.header.stamp = measurement.header.stamp

            # Now predict until next point t that coincides with next timepoint
            # for the controller.

            (X, xhat_r, Phat) = self.predict_step_calc(
                                    self.vel_list_corr[-1],
                                    (1 + self.case5)*self.vel_cmd_Ts - B, X, Phat)

            # Save variable globally
            self.X_r = X
            self.Phat = Phat

            self.vel_list_corr = [self.vel_list_corr[-1]]
            self.vel_list_corr += late_cmd_vel

        return xhat_r, xhat_r_t0

    def predict_step_calc(self, vel_cmd_stamped, Ts, X, Phat):
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

        Phat = np.matmul(Ts*self.A + np.identity(8), np.matmul(
            Phat, np.transpose(Ts*self.A + np.identity(8)))) + self.Q

        return X, xhat_r, Phat

    def correct_step_calc(self, pos_meas, X, xhat_r, Phat):
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
            Phat, np.transpose(self.C))) + self.R
        L = np.matmul(Phat, np.matmul(
            np.transpose(self.C), np.linalg.inv(S)))
        X = X + np.matmul(L, nu)
        Phat = np.matmul(
            (np.identity(8) - np.matmul(L, self.C)), Phat)
        self.Phat_t0 = Phat

        xhat_r.point.x = X[0, 0]
        xhat_r.point.y = X[3, 0]
        xhat_r.point.z = X[6, 0]

        return X, xhat_r, Phat

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
