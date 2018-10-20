#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from bebop_demo.srv import GetPoseEst, GetPoseEstResponse, GetPoseEstRequest
import rospy

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
        rospy.init_node('bebop_demo')

        rospy.Subscriber(
            'vive_localization/pose', PoseStamped, self.kalman_pos_correct)

        rospy.Service("get_pose", GetPoseEst, self.get_kalman_pos_est)

    def start(self):
        '''
        Starts running of bebop_demo node.
        '''
        rospy.spin()

    def get_kalman_pos_est(self, vel_cmd):
        '''
        '''
        self.latest_vel_cmd = vel_cmd
        pose_est = self.kalman_pos_predict(self.latest_vel_cmd)

        return GetPoseEstResponse(pose_est)

    def kalman_pos_predict(self, vel_cmd):
        '''
        Based on the velocity commands send out by the velocity controller,
        calculate a prediction of the position in the future.
        '''
        self.wm.predict_pos_update(vel_cmd, self.wm.B)

        return self.wm.xhat

    def kalman_pos_correct(self, measurement):
        # timing data to know length of preceding prediction step necessary
        '''
        Whenever a new position measurement is available, sends this
        information to the perception and then triggers the kalman filter
        to apply a correction step.
        '''
        # data moet nog verwerkt worden naar gewenste formaat
        self.pc.pose_vive = measurement

        # Calculate variable B (time between latest prediction and new t0).
        new_t0 = wm.get_timestamp(measurement)
        t_last_update = wm.get_timestamp(self.latest_vel_cmd)
        # not sure how to make time/duration instance --> float
        B = float(new_t0 - t_last_update)

        # Make prediction up to new t0.
        self.wm.predict_pos_update(self.latest_vel_cmd, B)
        # Correct the estimate at new t0 with the measurement.
        self.wm.correct_pos_update(self.pc.pose_vive)


if __name__ == '__main__':
    demo = Demo()
    demo.pc = Perception()
    demo.wm = WorldModel()
    demo.start()
