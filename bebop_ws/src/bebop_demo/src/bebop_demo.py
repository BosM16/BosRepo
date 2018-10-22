#!/usr/bin/env python

from geometry_msgs.msg import Twist, PoseStamped, Point, PointStamped
from bebop_demo.srv import GetPoseEst, GetPoseEstResponse, GetPoseEstRequest
import rospy
import tf2_ros

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

        self.pose_pub = rospy.Publisher(
            "/world_model/xhat", Point, queue_size=1)
        self.pose_pub_stamped = rospy.Publisher(
            "/world_model/xhat_stamped", PointStamped, queue_size=1)

        rospy.Service(
            "/world_model/get_pose", GetPoseEst, self.get_kalman_pos_est)

    def start(self):
        '''
        Starts running of bebop_demo node.
        '''
        rospy.spin()

    def get_kalman_pos_est(self, vel_cmd):
        '''
        Receives a time-stamped twist and returns an estimate of the position
        '''
        self.init = False

        self.vel_cmd_list.append(vel_cmd)
        self.latest_vel_cmd = vel_cmd
        self.kalman_pos_predict(self.latest_vel_cmd)

        # Publish latest estimate to read out Kalman result.
        self.pose_pub.publish(self.wm.xhat)
        self.xhat_rviz.header.stamp = rospy.Time.now()
        # Broadcast stamped position for visualization in rviz.
        self.xhat_stamped.point = self.xhat
        self.pose_pub_stamped.publish(self.xhat_stamped)

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
        # Results in an updated xhat.

    def kalman_pos_correct(self, measurement):
        '''
        Whenever a new position measurement is available, sends this
        information to the perception and then triggers the kalman filter
        to apply a correction step.

        Arguments:
            measurement: PoseStamped
        '''
        # TODO: transform measurement to "world_rot" frame.

        if self.init:
            self.wm.xhat_t0 = measurement.pose.position
        else:
            # data moet nog verwerkt worden naar gewenste formaat
            self.pc.pose_vive = measurement

            # First make prediction from old point t0 to last point t before
            # new measurement.
            index = 1
            vel_cmd_tstamp = self.wm.get_timestamp(self.vel_cmd_list[index])
            self.wm.xhat = self.wm.xhat_t0
            self.wm.predict_pos_update(
                    self.vel_cmd_list[index], vel_cmd_tstamp - self.wm.t0)

            while vel_cmd_tstamp < new_t0:
                self.wm.predict_pos_update(self.vel_cmd_list[index], self.B)
                index += 1
                vel_cmd_tstamp = self.wm.get_timestamp(
                                                    self.vel_cmd_list[index])

            # Calculate variable B (time between latest prediction and new t0).
            new_t0 = self.wm.get_timestamp(self.pc_pose_vive)
            t_last_update = self.wm.get_timestamp(self.latest_vel_cmd)
            B = new_t0 - t_last_update
            # Now make prediction up to new t0.
            self.wm.predict_pos_update(self.latest_vel_cmd, B)
            # Correct the estimate at new t0 with the measurement.
            self.wm.correct_pos_update(self.pc.pose_vive)
            # Now predict until next point t that coincides with next timepoint
            # for the controller.
            self.wm.predict_pos_update(self.latest_vel_cmd, self.B - B)

            self.vel_cmd_list = [self.latest_vel_cmd]


if __name__ == '__main__':
    demo = Demo()
    demo.pc = Perception()
    demo.wm = WorldModel()
    demo.start()
