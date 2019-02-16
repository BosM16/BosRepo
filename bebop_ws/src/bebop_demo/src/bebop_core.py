#!/usr/bin/env python

from geometry_msgs.msg import (
    Twist, PoseStamped, Point, PointStamped, TwistStamped)
from vive_localization.msg import PoseMeas
from std_msgs.msg import String, Empty
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
            "/world_model/yhat", PointStamped, queue_size=1)
        self.pose_r_pub = rospy.Publisher(
            "/world_model/yhat_r", PointStamped, queue_size=1)
        self.fsm_state = rospy.Publisher(
            "/fsm/state", String, queue_size=1)

        rospy.Subscriber(
            'vive_localization/ready', Empty, self.vive_ready)
        rospy.Subscriber(
            'vive_localization/pose', PoseMeas, self.new_measurement)
        rospy.Subscriber(
            'fsm/task', String, self.switch_task)
        rospy.Subscriber('ctrl_keypress/rtake_off', Bool, self.take_off)
        rospy.Subscriber('ctrl_keypress/rland', Bool, self.land)

        self.state = "initialization"
        self.fsm_state.publish("initialization")  # Finished when pushing controller buttons
        self.state_sequence = ["standby"]
        self._get_pose_service = None
        self.task_list = {"standby": [],
                          "take-off": ["take_off"],
                          "land": ["land"],
                          "point to point": ["omg_fly"],
                          "draw follow traj": ["draw path", "calculate path",
                                               "follow path"]}

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
        # Don't do prediction and transformation calculations if the
        # measurement is invalid.
        if self.measurement_valid:

            self.kalman.vel_cmd_list.append(req_vel.vel_cmd)
            self.kalman.latest_vel_cmd = req_vel.vel_cmd

            # print '---------------------kalman predict step velocity used', req_vel.vel_cmd.twist.linear
            self.wm.yhat_r, self.wm.vhat_r = self.kalman.kalman_pos_predict(
                                            self.kalman.latest_vel_cmd, self.wm.yhat_r)

            # Transform the rotated yhat and vhat to world frame.
            self.wm.yhat = self.transform_point(
                self.wm.yhat_r, "world_rot", "world")
            self.wm.vhat = self.transform_point(
                self.wm.vhat_r, "world_rot", "world")
            self.wm.yaw = self.pc.yaw

            self.pose_r_pub.publish(self.wm.yhat_r)
            self.pose_pub.publish(self.wm.yhat)

        return GetPoseEstResponse(
            self.wm.yhat, self.wm.vhat, self.wm.yaw, self.measurement_valid)

    def new_measurement(self, data):
        '''Processes incoming measurement from Vive localization.
        data:
            meas_world: PoseStamped
            yaw: float32
        '''
        self.pc.pose_vive = data.meas_world
        self.pc.yaw = data.yaw
        self.measurement_valid = self.pc.measurement_check()

        measurement = self.kalman.transform_pose(
                                self.pc.pose_vive, "world", "world_rot")
        if self.measurement_valid:
            if self.kalman.init:
                self.wm.yhat_r_t0.header = measurement.header
                zero_vel_cmd = TwistStamped()
                zero_vel_cmd.header = measurement.header
                self.kalman.vel_cmd_list = [zero_vel_cmd]
                self.kalman.init = False

            # Apply correction step.
            self.wm.yhat_r, self.wm.yhat_r_t0 = self.kalman.kalman_pos_correct(
                                                measurement, self.wm.yhat_r_t0)
            self.wm.yhat = self.transform_point(
                self.wm.yhat_r, "world_rot", "world")
            self.pose_pub.publish(self.wm.yhat)

    def switch_task(self, task):
        '''Reads out the task topic and switches to the desired task.
        '''
        if task not in self.task_list:
            print "Not a valid task, drone will remain in standby state."

        self.state_sequence = self.task_list.get(task, ["standby"])

    def send_states(self):
        '''Runs along the state sequence, sends out the current state and
        returns to the standby state when task is completed.
        '''
        if self.state_sequence != ["standby"]:
            for state in self.state_sequence:
                self.fsm_state.publish(state)

                # Check if previous state is finished to switch to next state.
                state_finish = False
                while not state_finish:
                    state_finish = check_state_finish()
                    rospy.sleep(0.1)

            self.fsm_state.publish("standby")

        rospy.sleep(0.1)


####################
# Task functions #
####################

    def take_off(self, empty):
        '''Check if take-off button is pressed and switch to take-off sequence.
        '''
        self.state_sequence = ["take-off"]

    def land(self, empty):
        '''Check if land button is pressed and switch to land sequence.
        '''
        #  ALS GELAND OOK NAAR STANDBY MODE????
        # OMG TOOLS MOET AF!
        self.state_sequence = ["land"]


####################
# Helper functions #
####################

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
    demo.kalman = Kalman(demo.wm.model)
    demo.start()
