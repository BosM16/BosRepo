#!/usr/bin/env python

from geometry_msgs.msg import Twist, PoseStamped, Point, PointStamped
from std_msgs.msg import Empty
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
        print 'PREDICT STEP', '\n'
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
            # print 'CORRECT STEP,\n'
            # print 'times of vel_cmds at start:\n'
            # for vel in self.vel_list_corr:
            #     t_vel = self.wm.get_timestamp(vel)
            #     print '-  ', t_vel - m.floor(t_vel)

            if (len(self.vel_list_corr) > 1) and self.wm.get_time_diff(
                    self.pc.pose_vive, self.vel_list_corr[1]) < 0:
                self.vel_list_corr = self.vel_list_corr[1:]
            # print 'vel cmd _list length after mod', len(self.vel_list_corr)
            self.case5 = False
            # tpose = self.wm.get_timestamp(self.pc.pose_vive)
            # txhatr = self.wm.get_timestamp(self.wm.xhat_r_t0)
            # print 'tpose', tpose - m.floor(tpose)
            # print 'txhatr', txhatr - m.floor(txhatr)
            # print "Applying Kalman correction step"
            #
            # print "measurement_world\n", measurement_world
            # print "measurement\n", measurement

            # First make prediction from old point t0 to last point t before
            # new measurement.
            # Calculate variable B (time between latest prediction and new t0).

            time_diff_check = self.wm.get_time_diff(
                self.pc.pose_vive, self.latest_vel_cmd)
            late_cmd_vel = []
            # Check for case 4.
            # print 'len velocity list before shrinking', len(self.vel_list_corr)
            if time_diff_check < 0:
                # print 'time diff check NOT ok, adjust t_last_update'
                # t_last_update = self.wm.get_timestamp(self.vel_list_corr[-2])
                late_cmd_vel = [self.vel_list_corr[-1]]
                self.vel_list_corr = self.vel_list_corr[0:-1]

            vel_len = len(self.vel_list_corr)
            # print 'len velocity list before check for case 3', vel_len
            if (vel_len > 1):
                case3 = False
                Ts = self.wm.get_time_diff(
                    self.vel_list_corr[1], self.wm.xhat_r_t0)
                tstamp = self.wm.get_timestamp(self.vel_list_corr[1])  # For debugging only!
                # print 'case 1 or 2'
            else:
                case3 = True
                Ts = self.wm.get_time_diff(
                    self.pc.pose_vive, self.wm.xhat_r_t0)
                tstamp = self.wm.get_timestamp(self.pc.pose_vive)  # For debugging only!
            # print 'Ts 151\n', Ts

            # t_1st_velcmd = self.wm.get_timestamp(self.vel_list_corr[0])  # For debugging only!
            # t_last_update = self.wm.get_timestamp(self.vel_list_corr[-1])

            # old_t0 = self.wm.get_timestamp(self.wm.xhat_r_t0)
            # new_t0 = self.wm.get_timestamp(self.pc.pose_vive)
            # if old_t0 == new_t0:
            #     print '--------------\n equal\n---------------\n'

            # Print all times of vel cmds:
            # print 'times of vel_cmds:\n'
            # for vel in self.vel_list_corr:
            #     t_vel = self.wm.get_timestamp(vel)
            #     print '-  ', t_vel - m.floor(t_vel)
            # print 'tstamp of first vel cmd', t_1st_velcmd - m.floor(t_1st_velcmd, -2)
            # print 'tstamp of second vel cmd', tstamp - m.floor(tstamp, -2)
            # print 'old t0', old_t0 - m.floor(old_t0)
            # print 'new t0', new_t0 - m.floor(new_t0)
            # print 't_last_update', t_last_update - m.floor(t_last_update, -2)

            (X, xhat_r) = self.wm.predict_pos_update(
                    self.vel_list_corr[0], Ts, self.wm.X_r_t0)  # Ts should be made variable depending on time between vel cmd's.
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
            B = self.wm.get_time_diff(self.pc.pose_vive, self.vel_list_corr[-1])
            if B > self.wm.vel_cmd_Ts:
                # print 'case 5'
                self.case5 = True
            # print 'Ts 164\n', B
            if not case3:
                # print 'not case 3'
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
            # print 'Ts 183\n', (1 + case5)*self.wm.vel_cmd_Ts - B, '\n'

            (X, xhat_r) = self.wm.predict_pos_update(
                                    self.vel_list_corr[-1],
                                    (1 + self.case5)*self.wm.vel_cmd_Ts - B, X)

            # Save variable globally
            self.wm.xhat_r = xhat_r
            self.wm.X_r = X

            # self.pose_r_pub.publish(self.wm.xhat_r)
            # self.pose_pub.publish(self.wm.xhat)

            self.vel_list_corr = [self.vel_list_corr[-1]]
            self.vel_list_corr += late_cmd_vel
            # tfinished = rospy.Time.now().to_sec()
            # print 'tfinished', tfinished - m.floor(tfinished), '\n ---- \n'
            # print 'times of vel_cmds at end:\n'
            # for vel in self.vel_list_corr:
            #     t_vel = self.wm.get_timestamp(vel)
            #     print '-  ', t_vel - m.floor(t_vel)
            # print '\n ---- \n'



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
