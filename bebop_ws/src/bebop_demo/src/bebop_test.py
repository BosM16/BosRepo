#!/usr/bin/env python

from geometry_msgs.msg import Twist, PoseStamped, Point, PointStamped
from bebop_demo.srv import GetPoseEst, GetPoseEstResponse, GetPoseEstRequest

import numpy as np
import math as m
import rospy
import tf2_ros       # kijken wat nog weg mag door verplaatsen naar kalman file
import tf2_geometry_msgs as tf2_geom

from perception import *
from world_model import *
from kalman import *


class Demo(object):
    '''
    Acts as hub for the Perception and WorldModel.
    '''

    def __init__(self):
        '''
        Initialization of Demo object.
        '''

        rospy.init_node('bebop_demo')

        self.pose_pub = rospy.Publisher(
            "/world_model/xhat", PointStamped, queue_size=1)
        self.pose_r_pub = rospy.Publisher(
            "/world_model/xhat_r", PointStamped, queue_size=1)

        rospy.Subscriber(
            'vive_localization/ready', Empty, self.vive_ready)
        rospy.Subscriber(
            'vive_localization/pose', PoseStamped, self.new_measurement)
        self._get_pose_service = None

    def start(self):
        '''
        Starts running of bebop_demo node.
        '''
        print '-------------------- \n Demo started \n --------------------'
        self.kalman.init = True
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
        self.kalman.init = False

        self.kalman.vel_cmd_list.append(req_vel.vel_cmd)
        self.kalman.latest_vel_cmd = req_vel.vel_cmd
        self.wm.xhat_r = self.kalman.kalman_pos_predict(
                                        self.kalman.latest_vel_cmd, self.wm.xhat_r)

        # Publish latest estimate to read out Kalman result.
        # Transform xhat to world frame.
        self.wm.xhat = self.transform_point(
            self.wm.xhat_r, "world_rot", "world")
        # print "xhat\n", self.wm.xhat
        # print "xhat_r\n", self.wm.xhat_r
        self.pose_r_pub.publish(self.wm.xhat_r)
        self.pose_pub.publish(self.wm.xhat)

        return GetPoseEstResponse(self.wm.xhat)

    def new_measurement(self, measurement_world):

        self.pc.pose_vive = measurement_world
        measurement = self.kalman.transform_pose(
                                measurement_world, "world", "world_rot")
        if self.kalman.init:
            self.wm.xhat_r_t0.header = measurement.header
            self.wm.xhat_r_t0.point = measurement.pose.position
            self.wm.xhat_r = self.wm.xhat_r_t0

        self.wm.xhat_r, self.wm.xhat_r_t0 = self.kalman.kalman_pos_correct(
                                                measurement, self.wm.xhat_r_t0)
        self.wm.xhat = self.transform_point(
            self.wm.xhat_r, "world_rot", "world")

    def transform_point(self, point, _from, _to):
        '''Transforms point (geometry_msgs/PointStamped) from frame "_from" to
        frame "_to".
        Arguments:
            - _from, _to = string, name of frame
        '''
        transform = self.kalman.get_transform(_from, _to)
        point_transformed = tf2_geom.do_transform_point(point, transform)

        return point_transformed


if __name__ == '__main__':
    demo = Demo()
    demo.pc = Perception()
    demo.wm = WorldModel()
    demo.kalman = Kalman(demo.wm.A, demo.wm.B, demo.wm.C)
    demo.start()
