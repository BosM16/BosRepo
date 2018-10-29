#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Empty

import numpy as np
import rospy
import sys
import tf
import tf2_ros
import triad_openvr


class LocalizationTest(object):
    '''
    '''

    def __init__(self):
        '''
        Initialization of Demo object.
        '''
        rospy.init_node('bebop_demo')

        self.tracked_object = 'tracker'

        self.pose_world = PoseStamped()
        self.pose_world.header.frame_id = "world"

        self.pos_update = rospy.Publisher(
            'vive_localization/pose', PoseStamped, queue_size=1)
        self.ready = rospy.Publisher(
            'vive_localization/ready', Empty, queue_size=1)

        rospy.Subscriber('vive_localization/calibrate', Empty, self.calibrate)
        rospy.Subscriber(
            'vive_localization/publish_poses', Empty, self.publish_pose_est)

    def init_transforms(self):

            self.broadc = tf2_ros.TransformBroadcaster()
            self.stbroadc = tf2_ros.StaticTransformBroadcaster()

            self.tfBuffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tfBuffer)

            self.tf_w_in_v = TransformStamped()
            self.tf_w_in_v.header.frame_id = "vive"
            self.tf_w_in_v.child_frame_id = "world"

            self.tf_r_in_w = TransformStamped()
            self.tf_r_in_w.header.frame_id = "world"
            self.tf_r_in_w.child_frame_id = "world_rot"

            self.tf_t_in_v = TransformStamped()
            self.tf_w_in_v.header.frame_id = "vive"
            self.tf_w_in_v.child_frame_id = "tracker"
            pose_vive = self.get_pose_vive()
            self.tf_t_in_v = self.pose_to_tf(pose_vive, "tracker")
            self.broadc.sendTransform(self.tf_t_in_v)

            self.tf_d_in_t = TransformStamped()
            self.tf_d_in_t.header.stamp = rospy.Time.now()
            self.tf_d_in_t.header.frame_id = "tracker"
            self.tf_d_in_t.child_frame_id = "init_drone"
            roll_d_in_t = np.pi/2
            pitch_d_in_t = -np.pi/2
            yaw_d_in_t = 0.
            quat = tf.transformations.quaternion_from_euler(roll_d_in_t,
                                                            pitch_d_in_t,
                                                            yaw_d_in_t)
            self.tf_d_in_t.transform.translation.x = 0.
            self.tf_d_in_t.transform.translation.y = 0.025
            self.tf_d_in_t.transform.translation.z = 0.1
            self.tf_d_in_t.transform.rotation.x = quat[0]
            self.tf_d_in_t.transform.rotation.y = quat[1]
            self.tf_d_in_t.transform.rotation.z = quat[2]
            self.tf_d_in_t.transform.rotation.w = quat[3]

            self.stbroadc.sendTransform(self.tf_d_in_t)

    def start(self):
        '''
        Starts running of bebop_demo node.
        '''
        print 'Localization test is ON'
        self.rate = rospy.Rate(50.)
        self.v = triad_openvr.triad_openvr()
        self.v.print_discovered_objects()

        self.init_transforms()

        rospy.spin()

    def calibrate(self, *_):

        print '--------------------------- \n'
        print 'CALIBRATION STARTED \n'

        pose_vive = self.get_pose_vive()
        self.tf_t_in_v = self.pose_to_tf(pose_vive, "tracker")
        self.broadc.sendTransform(self.tf_t_in_v)
        # self.stbroadc.sendTransform(self.tf_d_in_t)
        rospy.sleep(2.)

        # Calibrate: fix current drone pose as pose of world frame.
        self.tf_w_in_v = self.get_transform("init_drone", "vive")
        self.tf_w_in_v.child_frame_id = "world"
        # To make sure drone-frame is not fixed to world, change "init drone"
        # to "drone".
        self.tf_d_in_t.child_frame_id = "drone"

        self.stbroadc.sendTransform(self.tf_w_in_v)
        print 'tf_w_in_v'
        print self.tf_w_in_v

        print '--------------------------- \n'
        print 'CALIBRATED \n'

    def publish_pose_est(self, *_):
        '''Publishes message that calibration is completed. Starts publishing
        pose measurements.
        '''
        self.ready.publish(Empty())
        print '** Vive localization READY **'

        while not rospy.is_shutdown():
            pose_vive = self.get_pose_vive()
            self.tf_t_in_v = self.pose_to_tf(pose_vive, "tracker")

            self.broadc.sendTransform(self.tf_t_in_v)
            self.stbroadc.sendTransform(self.tf_d_in_t)

            # Calculate and publish pose of drone in world frame.
            tf_d_in_w = TransformStamped()
            tf_d_in_w = self.get_transform("drone", "world")
            pose_world = self.tf_to_pose(tf_d_in_w)
            self.pos_update.publish(pose_world)

            # Calculate and broadcast the rotating world frame.
            # - Tf drone in world to euler angles.
            euler = self.get_euler_angles(tf_d_in_w)
            # - Get yaw.
            yaw = euler[2]
            # - Yaw only (roll and pitch 0.0) to quaternions.
            quat = tf.transformations.quaternion_from_euler(0., 0., yaw)
            self.tf_r_in_w.transform.rotation.x = quat[0]
            self.tf_r_in_w.transform.rotation.y = quat[1]
            self.tf_r_in_w.transform.rotation.z = quat[2]
            self.tf_r_in_w.transform.rotation.w = quat[3]
            self.broadc.sendTransform(self.tf_r_in_w)

            self.rate.sleep()

    def get_euler_angles(self, transf):
        '''
        '''
        quat = (transf.transform.rotation.x,
                transf.transform.rotation.y,
                transf.transform.rotation.z,
                transf.transform.rotation.w)
        euler = tf.transformations.euler_from_quaternion(quat)

        return euler

    def get_transform(self, _from, _to):
        '''
        '''
        tf_f_in_t = self.tfBuffer.lookup_transform(
            _to, _from, rospy.Time(0), rospy.Duration(0.1))

        return tf_f_in_t

    def pose_to_tf(self, pose, child_frame_id):
        '''
        '''
        transf = TransformStamped()
        transf.header.stamp = rospy.Time.now()
        transf.header.frame_id = pose.header.frame_id
        transf.child_frame_id = child_frame_id
        transf.transform.translation = pose.pose.position
        transf.transform.rotation = pose.pose.orientation

        return transf

    def tf_to_pose(self, transf):
        '''
        '''
        pose = PoseStamped()
        pose.header.stamp = transf.header.stamp
        pose.header.frame_id = transf.header.frame_id
        pose.pose.position = transf.transform.translation
        pose.pose.orientation = transf.transform.rotation

        return pose

    def get_pose_vive(self):
        '''
        '''
        if self.tracked_object is 'controller':
            pose = np.array(self.v.devices["controller_1"].get_pose_euler())
        elif self.tracked_object is 'tracker':
            pose = np.array(self.v.devices["tracker_1"].get_pose_euler())

        else:
            print 'No device found.'
            return

        pose[3:6] = pose[3:6]*np.pi/180.
        quat = tf.transformations.quaternion_from_euler(
            pose[5], pose[4], pose[3])

        pose_vive = PoseStamped()
        pose_vive.header.frame_id = "vive"
        pose_vive.header.stamp = rospy.Time.now()
        pose_vive.pose.position.x = pose[0]
        pose_vive.pose.position.y = pose[1]
        pose_vive.pose.position.z = pose[2]
        pose_vive.pose.orientation.x = quat[0]
        pose_vive.pose.orientation.y = quat[1]
        pose_vive.pose.orientation.z = quat[2]
        pose_vive.pose.orientation.w = quat[3]

        return pose_vive


if __name__ == '__main__':
    test = LocalizationTest()
    test.start()
