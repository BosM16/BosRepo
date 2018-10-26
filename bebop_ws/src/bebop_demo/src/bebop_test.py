#!/usr/bin/env python

from geometry_msgs.msg import Twist, PoseStamped, Point, PointStamped
from std_msgs.msg import Empty
from bebop_demo.srv import GetPoseEst, GetPoseEstResponse, GetPoseEstRequest
import numpy as np
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

        rospy.init_node('bebop_demo')

        rospy.Subscriber(
            'vive_localization/pose', PoseStamped, self.kalman_pos_correct)

        self.vel_cmd_list = []
        self.point_pub = PointStamped()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.pose_pub = rospy.Publisher(
            "/world_model/xhat", PointStamped, queue_size=1)
        self.pose_r_pub = rospy.Publisher(
            "/world_model/xhat_r", PointStamped, queue_size=1)

        rospy.Subscriber(
            'vive_localization/ready', Empty, self.vive_ready)

        self._get_pose_service = None

    def start(self):
        '''
        Starts running of bebop_demo node.
        '''
        print '-------------------- \n Kalman started \n --------------------'
        rospy.spin()

    def vive_ready(self, *_):
        '''
        Set variable to True when vive is calibrated
        '''
        self._get_pose_service = rospy.Service(
            "/world_model/get_pose", GetPoseEst, self.get_kalman_pos_est)

    def get_kalman_pos_est(self, req_vel):
        '''
        Receives a time-stamped twist and returns an estimate of the position
        Argument:
            - vel_cmd = TwistStamped
        '''
        self.init = False

        self.vel_cmd_list.append(req_vel.vel_cmd)
        self.latest_vel_cmd = req_vel.vel_cmd
        self.kalman_pos_predict(self.latest_vel_cmd)

        # Publish latest estimate to read out Kalman result.
        self.transform_point("world_rot", "world")
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
            self.wm.predict_pos_update(vel_cmd, self.wm.B)
        # Results in an updated xhat_r.

    def kalman_pos_correct(self, measurement_world):
        '''
        Whenever a new position measurement is available, sends this
        information to the perception and then triggers the kalman filter
        to apply a correction step.

        Arguments:
            measurement: PoseStamped
        '''
        if self.init:
            self.wm.xhat_r_t0.point = measurement_world.pose.position
        else:
            measurement = self.transform_pose(
                                    measurement_world, "world", "world_rot")
            self.pc.pose_vive = measurement_world

            # First make prediction from old point t0 to last point t before
            # new measurement.
            # Calculate variable B (time between latest prediction and new t0).
            t_last_update = self.wm.get_timestamp(self.latest_vel_cmd)
            new_t0 = self.wm.get_timestamp(self.pc.pose_vive)
            time_diff_check = new_t0 - t_last_update
            late_cmd_vel = []
            # Check for case 4.
            if time_diff_check < 0:
                print '---\ncase 4'
                t_last_update = self.wm.get_timestamp(self.vel_cmd_list[-2])
                late_cmd_vel = [self.vel_cmd_list[-1]]
                self.vel_cmd_list = self.vel_cmd_list[0:-1]

            vel_len = len(self.vel_cmd_list)

            # set xhat equal to xhat at t0 to start prediction from here
            self.wm.xhat_r = self.wm.xhat_r_t0

            if (vel_len > 1):
                tstamp = self.wm.get_timestamp(self.vel_cmd_list[1])
            else:
                tstamp = new_t0

            self.wm.predict_pos_update(
                    self.vel_cmd_list[0], tstamp - self.wm.t0)
            # If not case 2 or 3 -> need to predict up to
            # last vel cmd before new_t0
            if vel_len > 2:
                print '---\ncase 1'
                for i in range(vel_len - 2):
                    self.wm.predict_pos_update(self.vel_cmd_list[i], self.wm.B)

            # Now make prediction up to new t0 if not case 3.
            B = (new_t0 - t_last_update)*np.identity(3)
            if not tstamp == new_t0:
                print '---\ncase 2'
                self.wm.predict_pos_update(self.latest_vel_cmd, B)
            if vel_len == 1:
                print '---\ncase 3'
            # Correct the estimate at new t0 with the measurement.
            self.wm.correct_pos_update(self.pc.pose_vive)
            # Now predict until next point t that coincides with next timepoint
            # for the controller.
            self.wm.predict_pos_update(self.vel_cmd_list[-1], self.wm.B - B)

            self.vel_cmd_list = [self.vel_cmd_list[-1]]
            self.vel_cmd_list + late_cmd_vel

    def transform_point(self, _from, _to):
        '''Transforms point (geometry_msgs/PointStamped) from frame "_from" to
        frame "_to".
        '''
        transform = self.get_transform(_from, _to)
        self.wm.xhat = tf2_geom.do_transform_point(self.wm.xhat_r, transform)

    def transform_pose(self, pose, _from, _to):
        '''Transforms pose (geometry_msgs/PoseStamped) from frame "_from" to
        frame "_to".
        '''
        transform = self.get_transform(_from, _to)
        # Only works with transformstamped.
        pose_tf = tf2_geom.do_transform_pose(pose, transform)

        return pose_tf

    def get_transform(self, _from, _to):
        '''
        '''
        tf_f_in_t = self.tfBuffer.lookup_transform(
            _to, _from, rospy.Time(0), rospy.Duration(0.1))

        return tf_f_in_t


if __name__ == '__main__':
    demo = Demo()
    demo.pc = Perception()
    demo.wm = WorldModel()
    demo.start()
