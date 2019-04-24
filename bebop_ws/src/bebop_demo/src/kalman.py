#!/usr/bin/env python

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from std_msgs.msg import Empty

import rospy
import tf2_ros
import tf2_geometry_msgs as tf2_geom

import numpy as np


class Kalman(object):

    def __init__(self, model):
        '''
        Asynchronous kalman filter to estimate position.
        '''

        self.case5 = False
        self.init = True

        # Assign model matrices
        self.A = model.A
        self.B = model.B
        self.C = model.C
        self.C_vel = model.C_vel
        self.D_vel = model.D_vel

        self.vel_list_corr = []

        self.X_r = np.zeros(shape=(8, 1))
        self.X_r_t0 = np.zeros(shape=(8, 1))
        self.input_cmd_Ts = rospy.get_param('vel_cmd/sample_time', 0.01)  # s

        self.Phat_t0 = np.zeros(8)
        self.Phat = np.zeros(8)

        # Kalman tuning parameters.
        self.R = np.identity(3)  # measurement noise covariance
        self.Q = 1e3*np.identity(8)  # process noise covariance

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def kalman_pos_predict(self, input_cmd, yhat_r):
        '''
        Based on the velocity commands send out by the velocity controller,
        calculate a prediction of the position in the future.
        Arguments:
            input_cmd: TwistStamped
        '''
        # print '---------------self.X_r before', self.X_r
        (self.X_r, yhat_r, vhat_r, self.Phat) = self.predict_step_calc(
            input_cmd, self.input_cmd_Ts, self.X_r, self.Phat)
        # print '---------------self.X_r after', self.X_r
        return yhat_r, vhat_r

    def kalman_pos_correct(self, measurement, yhat_r_t0):
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

        self.vel_list_corr = self.vel_list_corr + self.input_cmd_list
        self.input_cmd_list = []

        if (len(self.vel_list_corr) > 1) and self.get_time_diff(
                yhat_r_t0, self.vel_list_corr[1]) > 0:
            self.vel_list_corr = self.vel_list_corr[1:]
        self.case5 = False

        # First make prediction from old point t0 to last point t before
        # new measurement.
        # Calculate variable B (time between latest prediction and new t0).
        time_diff_check = self.get_time_diff(
            measurement, self.vel_list_corr[-1])
        late_cmd_vel = []

        # Check for case 4.
        if time_diff_check < 0:
            late_cmd_vel = [self.vel_list_corr[-1]]
            self.vel_list_corr = self.vel_list_corr[0:-1]

        vel_len = len(self.vel_list_corr)
        if (vel_len > 1):
            case3 = False
            Ts = self.get_time_diff(self.vel_list_corr[1], yhat_r_t0)
        else:
            case3 = True
            Ts = self.get_time_diff(measurement, yhat_r_t0)
        # print '\n kalman first predict step Ts, X_r_t0 \n', Ts, self.X_r_t0
        (X, yhat_r, vhat_r, Phat) = self.predict_step_calc(
                self.vel_list_corr[0], Ts, self.X_r_t0, self.Phat_t0)

        # If not case 2 or 3 -> need to predict up to
        # last vel cmd before new_t0
        if vel_len > 2:
            # Case 1.
            for i in range(vel_len - 2):
                Ts = self.get_time_diff(
                    self.vel_list_corr[i+2], self.vel_list_corr[i+1])
                # print '\n kalman second predict step Ts and yhat_r \n', Ts, yhat_r.point
                (X, yhat_r, vhat_r, Phat) = self.predict_step_calc(
                    self.vel_list_corr[i+1], Ts, X, Phat)

        B = self.get_time_diff(measurement,
                               self.vel_list_corr[-1])
        if (B > self.input_cmd_Ts):
            self.case5 = True

        # Now make prediction up to new t0 if not case 3.
        if not case3:
            # print '\n kalman third predict step Ts and yhat_r \n', B, yhat_r.point
            (X, yhat_r, vhat_r, Phat) = self.predict_step_calc(
                self.vel_list_corr[-1], B, X, Phat)
        else:
            self.case5 = False
            B = B % self.input_cmd_Ts

        # ---- CORRECTION ----
        # Correct the estimate at new t0 with the measurement.
        # print '\n kalman correct yhat_r \n', yhat_r.point
        (X, yhat_r_t0, Phat) = self.correct_step_calc(
                                                measurement, X, yhat_r, Phat)
        self.X_r_t0 = X
        yhat_r_t0.header.stamp = measurement.header.stamp

        # Now predict until next point t that coincides with next timepoint
        # for the controller.
        # print '\n kalman fourth predict step Ts and yhat_r \n', (1 + self.case5)*self.input_cmd_Ts - B, yhat_r_t0.point
        (X, yhat_r, vhat_r, Phat) = self.predict_step_calc(
                                self.vel_list_corr[-1],
                                (1 + self.case5)*self.input_cmd_Ts - B,
                                X, Phat)

        # Save variable globally
        self.X_r = X
        self.Phat = Phat

        self.vel_list_corr = [self.vel_list_corr[-1]]
        self.vel_list_corr += late_cmd_vel

        return yhat_r, yhat_r_t0

    def predict_step_calc(self, input_cmd_stamped, Ts, X, Phat):
        """
        Prediction step of the kalman filter. Update the position of the drone
        using the reference velocity commands.
        Arguments:
            - input_cmd_stamped = TwistStamped
            - Ts = varying step size over which to integrate.
        """
        input_cmd = input_cmd_stamped.twist

        u = np.array([[input_cmd.linear.x],
                      [input_cmd.linear.y],
                      [input_cmd.linear.z]])
        X = (np.matmul(Ts*self.A + np.identity(8), X)
             + np.matmul(Ts*self.B, u))

        Y = np.matmul(self.C, X)
        Y_vel = np.matmul(self.C_vel, X) + np.matmul(self.D_vel, u)

        yhat_r = PointStamped()
        yhat_r.header.frame_id = "world_rot"
        yhat_r.point.x = Y[0, 0]
        yhat_r.point.y = Y[1, 0]
        yhat_r.point.z = Y[2, 0]

        vhat_r = PointStamped()
        vhat_r.header.frame_id = "world_rot"
        vhat_r.point.x = Y_vel[0, 0]
        vhat_r.point.y = Y_vel[1, 0]
        vhat_r.point.z = Y_vel[2, 0]

        Phat = np.matmul(Ts*self.A + np.identity(8), np.matmul(
            Phat, np.transpose(Ts*self.A + np.identity(8)))) + self.Q

        return X, yhat_r, vhat_r, Phat

    def correct_step_calc(self, pos_meas, X, yhat_r, Phat):
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

        Y = np.matmul(self.C, X)

        yhat_r.point.x = Y[0, 0]
        yhat_r.point.y = Y[1, 0]
        yhat_r.point.z = Y[2, 0]

        return X, yhat_r, Phat

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
