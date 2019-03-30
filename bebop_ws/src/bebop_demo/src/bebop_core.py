#!/usr/bin/env python

from geometry_msgs.msg import (
    Twist, PoseStamped, Point, PointStamped, TwistStamped)
from vive_localization.msg import PoseMeas
from std_msgs.msg import String, Empty, Bool
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

        self.transfoldstamp = 0.0

        self.state = "initialization"
        self.state_sequence = []
        self.change_state = False
        self.new_task = False
        self.state_finish = False
        self.omg_standby = False
        self.airborne = False
        self.task_dict = {"standby": [],
                          "take-off": ["take-off"],
                          "land": ["land"],
                          "point to point": ["omg standby", "omg fly"],
                          "draw follow traj": ["land", "draw path", "take-off",
                                               "fly to start", "follow path"],
                          "place window obstacles": ["place window obstacles",
                                                     "configure motionplanner",
                                                     "take-off"]}

        self.meas_rot = rospy.Publisher(
            'world_mode/meas_rot', PoseStamped, queue_size=1)
        self.pose_pub = rospy.Publisher(
            'world_model/yhat', PointStamped, queue_size=1)
        self.pose_r_pub = rospy.Publisher(
            'world_model/yhat_r', PointStamped, queue_size=1)
        self.fsm_state = rospy.Publisher(
            'fsm/state', String, queue_size=1)
        # Finished when pushing controller buttons
        self.fsm_state.publish("initialization")

        rospy.Subscriber(
            'vive_localization/ready', Empty, self.vive_ready)
        rospy.Subscriber(
            'vive_localization/pose', PoseMeas, self.new_measurement)
        rospy.Subscriber(
            'fsm/task', String, self.switch_task)
        rospy.Subscriber(
            'ctrl_keypress/rmenu_button', Empty, self.take_off_land)
        rospy.Subscriber(
            'controller/state_finish', Empty, self.ctrl_state_finish)
        rospy.Subscriber(
            'ctrl_keypress/rtrackpad', Empty, self.switch_state)
        rospy.Subscriber(
            'ctrl_keypress/rtrigger', Bool, self.r_trigger)

        self._get_pose_service = None

    def start(self):
        '''
        Starts running of bebop_demo node. Runs along the state sequence, sends
        out the current state and returns to the standby state when task is
        completed.
        '''
        print '-------------------- \n Demo started \n --------------------'

        while not rospy.is_shutdown():
            if self.new_task:
                self.new_task = False
                self.change_state = False
                self.state_finish = False

                # Run over sequence of states corresponding to current task.
                for state in self.state_sequence:
                    self.state = state
                    print "bebop_core state changed to:", self.state
                    self.fsm_state.publish(state)

                    # Omg tools should return to its own standby status unless
                    # the controller trackpad has been pressed.
                    if self.state == "omg standby":
                        self.omg_standby = True

                    task_final_state = (self.state == self.state_sequence[-1])
                    # Check if previous state is finished and if allowed to
                    # switch state based on controller input.
                    while not ((self.state_finish and (
                                self.change_state or task_final_state)) or
                               self.new_task):
                        # Remaining in state. Allow state action to continue.
                        rospy.sleep(0.1)

                    self.change_state = False
                    self.state_finish = False

                    leave_omg = (
                        self.state == "omg standby" and not self.omg_standby)
                    # User forces leaving omg with trackpad or other new task
                    # received --> leave the for loop for the current task.
                    if (leave_omg or self.new_task):
                        print 'BROKE FOR LOOP'
                        break

                # Make sure that omg-tools task is repeated until force quit.
                if self.omg_standby:
                    self.new_task = True
                # Only publish standby state when task is finished.
                # Except for repetitive tasks (back to first state in task).
                if not self.new_task:
                    self.fsm_state.publish("standby")
                    print "bebop_core state changed to:", "standby"

            rospy.sleep(0.1)

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
                                    self.kalman.latest_vel_cmd)

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
        self.meas_rot.publish(measurement)

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

####################
# Task functions #
####################

    def switch_task(self, task):
        '''Reads out the task topic and switches to the desired task.
        '''
        if task.data not in self.task_dict:
            print "Not a valid task, drone will remain in standby state."

        self.state_sequence = self.task_dict.get(task.data, [])
        self.new_task = True
        print "bebop_core received a new task:", task.data

    def take_off_land(self, empty):
        '''Check if menu button is pressed and switch to take-off or land
        sequence depending on last task that was executed.
        '''
        if not ((self.state == "take-off") and (self.state == "land")):
            if self.airborne:
                self.state_sequence = self.task_dict.get("land", [])
            else:
                self.state_sequence = self.task_dict.get("take-off", [])
            self.airborne = not self.airborne
            self.new_task = True
            print "bebop_core received a new task:", self.state_sequence[0]

####################
# Helper functions #
####################

    def switch_state(self, empty):
        '''When controller trackpad is pressed changes change_state variable
        to true to allow fsm to switch states in state sequence.
        '''
        if self.state == "omg standby":
            self.omg_standby = False
            self.new_task = False
        self.change_state = True

    def ctrl_state_finish(self, empty):
        '''Checks whether controller has finished the current state.
        '''
        self.state_finish = True

    def r_trigger(self, pressed):
        if pressed.data and (self.state == "omg standby"):
            self.change_state = True

    def transform_point(self, point, _from, _to):
        '''Transforms point (geometry_msgs/PointStamped) from frame "_from" to
        frame "_to".
        Arguments:
            - _from, _to = string, name of frame
        '''
        transform = self.kalman.get_transform(_from, _to)
        self.transfoldstamp = transform.header.stamp

        point_transformed = tf2_geom.do_transform_point(point, transform)

        return point_transformed


if __name__ == '__main__':
    demo = Demo()
    demo.pc = Perception()
    demo.wm = WorldModel()
    demo.kalman = Kalman(demo.wm.model)
    demo.start()
