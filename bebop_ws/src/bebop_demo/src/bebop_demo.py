#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose2D, Pose, PoseStamped, Point
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
        rospy.init_node('bebop_demo')

        rospy.Subscriber(
            'vive_localization/pose', PoseStamped, self.kalman_pos_correct)

        self.vel_cmd_list = []

        # Would make more sense if it were a PoseStamped, but let's wait what
        # comes out of the Vive.
        rospy.Subscriber('Pose_pub', PoseStamped, self.kalman_pos_correct)
        self.publish_pos_est = rospy.Publisher(
            'xhat', Pose, queue_size=1)

        rospy.Service("get_pose", GetPoseEst, self.get_kalman_pos_est)

        self.pose_pub = rospy.Publisher("/wm/position_estimate", Point)

        self.broadc = tf2_ros.TransformBroadcaster()
        self.tf_kp_in_w = TransformStamped()
        self.tf_kp_in_w.header.frame_id = "world"
        self.tf_kp_in_w.child_frame_id = "kalman_pos"

    def start(self):
        '''
        Starts running of bebop_demo node.
        '''
        rospy.spin()

    def get_kalman_pos_est(self, vel_cmd):
        '''
        Receives a time-stamped twist and returns an estimate of the position
        '''
        self.vel_cmd_list.append(vel_cmd)
        self.latest_vel_cmd = vel_cmd
        pose_est = self.kalman_pos_predict(self.latest_vel_cmd)
        self.publish_pos_est.publish(pose_est)

        return GetPoseEstResponse(pose_est.position)

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
        new_t0 = self.wm.get_timestamp(measurement)
        t_last_update = self.wm.get_timestamp(self.latest_vel_cmd)
        B = new_t0 - t_last_update

        # First make prediction from old point t0 to point t before new
        # measurement.
        index = 1
        vel_cmd_tstamp = self.wm.get_timestamp(self.vel_cmd_list[index])
        self.wm.xhat = self.wm.xhat_t0
        self.wm.predict_pos_update(
                self.vel_cmd_list[index], vel_cmd_tstamp - self.wm.t0)

        while vel_cmd_tstamp < new_t0:
            self.wm.predict_pos_update(self.vel_cmd_list[index], self.B)
            index += 1
            vel_cmd_tstamp = self.wm.get_timestamp(self.vel_cmd_list[index])

        # Now make prediction up to new t0.
        self.wm.predict_pos_update(self.latest_vel_cmd, B)
        # Correct the estimate at new t0 with the measurement.
        self.wm.correct_pos_update(self.pc.pose_vive)
        # Now predict until next point t that coincides with next timepoint for
        # the controller.
        self.wm.predict_pos_update(self.latest_vel_cmd, self.B - B)

        self.vel_cmd_list = [self.latest_vel_cmd]

        # Publish latest estimate to read out Kalman result.
        self.pose_pub.publish(self.wm.xhat)
        self.tf_kp_in_w.transform.translation.x = self.wm.xhat.x
        self.tf_kp_in_w.transform.translation.y = self.wm.xhat.y
        self.tf_kp_in_w.transform.translation.z = self.wm.xhat.z
        self.stbroadc.sendTransform(self.tf_kp_in_w)


if __name__ == '__main__':
    demo = Demo()
    demo.pc = Perception()
    demo.wm = WorldModel()
    demo.start()
