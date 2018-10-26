#!/usr/bin/env python

from geometry_msgs.msg import Twist, PoseStamped, Point, PointStamped
from bebop_demo.srv import GetPoseEst, GetPoseEstResponse, GetPoseEstRequest
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

        rospy.Service(
            "/world_model/get_pos", GetPoseEst, self.get_kalman_pos_est)

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
        self.wm.xhat = transform_point(self.wm.xhat_r, "world_rot", "world")
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
        # TODO: transform measurement to "world_rot" frame.

        if self.init:
            self.wm.xhat_r_t0.point = measurement_world.pose.position
        else:
            measurement = self.transform_pose(
                                    measurement_world, "world", "world_rot")
            self.pc.pose_vive = measurement

            # First make prediction from old point t0 to last point t before
            # new measurement.
            index = 1
            vel_cmd_tstamp = self.wm.get_timestamp(self.vel_cmd_list[index])
            self.wm.xhat_r = self.wm.xhat_r_t0
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

    def transform_point(self, point, _from, _to):
        '''Transforms point (geometry_msgs/PointStamped) from frame "_from" to
        frame "_to".
        '''
        transform = self.get_transform(_from, _to)
        point_tf = tf2_geom.do_transform_point(point, transform)

        return point_tf

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
