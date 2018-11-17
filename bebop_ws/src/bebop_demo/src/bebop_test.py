#!/usr/bin/env python

from geometry_msgs.msg import Twist, PoseStamped, Point, PointStamped
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker
from bebop_demo.srv import GetPoseEst, GetPoseEstResponse, GetPoseEstRequest

import numpy as np
import math as m
import rospy
import tf2_ros
import tf2_geometry_msgs as tf2_geom

from perception import *
from world_model import *


class Demo(object):
    '''
    Acts as hub for the Perception and WorldModel.
    '''

    def __init__(self):
        '''
        Initialization of Demo object.
        '''
        self.init = True
        self.case5 = False

        rospy.init_node('bebop_demo')

        self.vel_cmd_list = []
        self.vel_list_corr = []

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.pose_pub = rospy.Publisher(
            "/world_model/xhat", PointStamped, queue_size=1)
        self.pose_r_pub = rospy.Publisher(
            "/world_model/xhat_r", PointStamped, queue_size=1)

        rospy.Subscriber(
            'vive_localization/ready', Empty, self.vive_ready)
        rospy.Subscriber(
            'vive_localization/pose', PoseStamped, self.kalman_pos_correct)

        self._get_pose_service = None

    def start(self):
        '''
        Starts running of bebop_demo node.
        '''
        print '-------------------- \n Demo started \n --------------------'
        rospy.spin()

    def vive_ready(self, *_):
        '''
        Provides get_pose service when vive is calibrated.
        '''
        self._get_pose_service = rospy.Service(
            "/world_model/get_pose", GetPoseEst, self.get_kalman_pos_est)

    def get_kalman_pos_est(self, req_vel):
        '''
        Receives a time-stamped twist and returns an estimate of the position
        Argument:
            - req_vel = TwistStamped
        '''
        self.init = False

        self.vel_cmd_list.append(req_vel.vel_cmd)
        self.latest_vel_cmd = req_vel.vel_cmd
        self.kalman_pos_predict(self.latest_vel_cmd)

        # Publish latest estimate to read out Kalman result.
        # Transform xhat to world frame.
        self.wm.xhat = self.transform_point(
            self.wm.xhat_r, "world_rot", "world")
        # print "xhat\n", self.wm.xhat
        # print "xhat_r\n", self.wm.xhat_r
        self.pose_r_pub.publish(self.wm.xhat_r)
        self.pose_pub.publish(self.wm.xhat)

        return GetPoseEstResponse(self.wm.xhat)

    def kalman_pos_predict(self, vel_cmd):
        '''
        Based on the velocity commands send out by the velocity controller,
        calculate a prediction of the position in the future.
        Arguments:
            vel_cmd: TwistStamped
        '''
        if not self.init:
            (self.wm.X_r, self.wm.xhat_r) = self.wm.predict_pos_update(
                vel_cmd, self.wm.vel_cmd_Ts, self.wm.X_r)

    def kalman_pos_correct(self, measurement_world):
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
        measurement = self.transform_pose(
                                measurement_world, "world", "world_rot")
        self.pc.pose_vive = measurement_world
        if self.init:
            self.wm.xhat_r_t0.header = measurement.header
            self.wm.xhat_r_t0.point = measurement.pose.position
            self.wm.X_r_t0[0, 0] = self.wm.xhat_r_t0.point.x
            self.wm.X_r_t0[3, 0] = self.wm.xhat_r_t0.point.y
            self.wm.X_r_t0[6, 0] = self.wm.xhat_r_t0.point.z
            self.wm.xhat_r = self.wm.xhat_r_t0
            self.wm.X_r = self.wm.X_r_t0

        if not self.init:
            self.vel_list_corr = self.vel_list_corr + self.vel_cmd_list
            self.vel_cmd_list = []

            if (len(self.vel_list_corr) > 1) and self.wm.get_time_diff(
                    self.pc.pose_vive, self.vel_list_corr[1]) < 0:
                self.vel_list_corr = self.vel_list_corr[1:]
            self.case5 = False

            # First make prediction from old point t0 to last point t before
            # new measurement.
            # Calculate variable B (time between latest prediction and new t0).
            time_diff_check = self.wm.get_time_diff(
                self.pc.pose_vive, self.latest_vel_cmd)
            late_cmd_vel = []

            # Check for case 4.
            if time_diff_check < 0:
                late_cmd_vel = [self.vel_list_corr[-1]]
                self.vel_list_corr = self.vel_list_corr[0:-1]

            vel_len = len(self.vel_list_corr)
            if (vel_len > 1):
                case3 = False
                Ts = self.wm.get_time_diff(
                    self.vel_list_corr[1], self.wm.xhat_r_t0)
            else:
                case3 = True
                Ts = self.wm.get_time_diff(
                    self.pc.pose_vive, self.wm.xhat_r_t0)

            (X, xhat_r) = self.wm.predict_pos_update(
                    self.vel_list_corr[0], Ts, self.wm.X_r_t0)

            # If not case 2 or 3 -> need to predict up to
            # last vel cmd before new_t0
            if vel_len > 2:
                # Case 1.
                for i in range(vel_len - 2):
                    Ts = self.wm.get_time_diff(
                        self.vel_list_corr[i+2], self.vel_list_corr[i+1])
                    (X, xhat_r) = self.wm.predict_pos_update(
                        self.vel_list_corr[i+1], Ts, X)

            # Now make prediction up to new t0 if not case 3.
            B = self.wm.get_time_diff(self.pc.pose_vive,
                                      self.vel_list_corr[-1])
            if B > self.wm.vel_cmd_Ts:
                self.case5 = True
            if not case3:
                (X, xhat_r) = self.wm.predict_pos_update(
                    self.latest_vel_cmd, B, X)

            # ---- CORRECTION ----
            # Correct the estimate at new t0 with the measurement.
            (X, xhat_r) = self.wm.correct_pos_update(measurement, X, xhat_r)
            self.wm.X_r_t0 = X
            self.wm.xhat_r_t0 = xhat_r
            self.wm.xhat_r_t0.header.stamp = measurement.header.stamp

            # Now predict until next point t that coincides with next timepoint
            # for the controller.

            (X, xhat_r) = self.wm.predict_pos_update(
                                    self.vel_list_corr[-1],
                                    (1 + self.case5)*self.wm.vel_cmd_Ts - B, X)

            # Save variable globally
            self.wm.xhat_r = xhat_r
            self.wm.X_r = X

            self.vel_list_corr = [self.vel_list_corr[-1]]
            self.vel_list_corr += late_cmd_vel

    def transform_point(self, point, _from, _to):
        '''Transforms point (geometry_msgs/PointStamped) from frame "_from" to
        frame "_to".
        Arguments:
            - _from, _to = string, name of frame
        '''
        transform = self.get_transform(_from, _to)
        point_transformed = tf2_geom.do_transform_point(point, transform)

        return point_transformed

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


if __name__ == '__main__':
    demo = Demo()
    demo.pc = Perception()
    demo.wm = WorldModel()
    demo.start()
