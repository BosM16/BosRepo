#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Empty
from vive_localization.msg import PoseMeas

import numpy as np
import rospy
import sys
import tf
import tf2_ros
import tf2_geometry_msgs as tf2_geom

from fabulous.color import highlight_red, blue, green

import triad_openvr


class ViveLocalization(object):
    '''
    '''

    def __init__(self):
        '''
        Initialization of Demo object.
        '''
        rospy.init_node('bebop_demo')

        self.tracked_objects = ["tracker_1", "controller_1", "controller_2"]

        self.calib = rospy.get_param('vive_localization/calibrate', True)

        self.pose_t_in_w = PoseStamped()
        self.pose_t_in_w.header.frame_id = "world"

        self.pose_c1_in_w = PoseStamped()
        self.pose_c1_in_w.header.frame_id = "world"
        self.pose_c2_in_w = PoseStamped()
        self.pose_c2_in_w.header.frame_id = "world"

        self.pos_update = rospy.Publisher(
            'vive_localization/pose', PoseMeas, queue_size=1)
        # Note that the following could made more general for any number of
        # tracked objects.
        self.c1_pos_update = rospy.Publisher(
            'vive_localization/c1_pose', PoseStamped, queue_size=1)
        self.c2_pos_update = rospy.Publisher(
            'vive_localization/c2_pose', PoseStamped, queue_size=1)
        self.c_publishers = [self.c1_pos_update, self.c2_pos_update]

        self.ready = rospy.Publisher(
            'vive_localization/ready', Empty, queue_size=1)

        rospy.Subscriber('vive_localization/calibrate', Empty, self.calibrate)
        rospy.Subscriber(
            'vive_localization/publish_poses', Empty, self.publish_pose_est)

    def init_transforms(self):
        '''
        '''
        self.broadc = tf2_ros.TransformBroadcaster()
        self.stbroadc = tf2_ros.StaticTransformBroadcaster()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Take calibrated values for Roblab setting as default.
        # Can be overwridden by 'self.calibrate()' function.
        # (Set calibration in localization.launch file to True)
        self.tf_w_in_v = TransformStamped()
        self.tf_w_in_v.header.frame_id = "vive"
        self.tf_w_in_v.child_frame_id = "world"

        # # Newest calibrated values (origin in center) (Mathias)
        # self.tf_w_in_v.transform.translation.x = -0.0653594065575
        # self.tf_w_in_v.transform.translation.y = -2.93788157322
        # self.tf_w_in_v.transform.translation.z = -3.58790665423
        # self.tf_w_in_v.transform.rotation.x = 0.633938483633
        # self.tf_w_in_v.transform.rotation.y = -0.312469733542
        # self.tf_w_in_v.transform.rotation.z = -0.312720898172
        # self.tf_w_in_v.transform.rotation.w = -0.634578840205
        # Newest calibrated values (origin in center) (Rian)
        self.tf_w_in_v.transform.translation.x = 0.0127584116207
        self.tf_w_in_v.transform.translation.y = -2.9642058687
        self.tf_w_in_v.transform.translation.z = -3.24945095473
        self.tf_w_in_v.transform.rotation.x = -0.339866444679
        self.tf_w_in_v.transform.rotation.y = -0.633483680153
        self.tf_w_in_v.transform.rotation.z = -0.617152604346
        self.tf_w_in_v.transform.rotation.w = 0.319862298045
        # # Old calibrated values (origin in corner)
        # self.tf_w_in_v.transform.translation.x = 0.129081706552
        # self.tf_w_in_v.transform.translation.y = -2.89506984729
        # self.tf_w_in_v.transform.translation.z = -6.92454187852
        # self.tf_w_in_v.transform.rotation.x = -0.343447879743
        # self.tf_w_in_v.transform.rotation.y = -0.639533223494
        # self.tf_w_in_v.transform.rotation.z = -0.61245458901
        # self.tf_w_in_v.transform.rotation.w = 0.312953968416

        self.stbroadc.sendTransform(self.tf_w_in_v)
        rospy.sleep(2.)

        self.tf_r_in_w = TransformStamped()
        self.tf_r_in_w.header.frame_id = "world"
        self.tf_r_in_w.child_frame_id = "world_rot"

        self.tf_t_in_v = TransformStamped()
        self.tf_w_in_v.header.frame_id = "vive"
        self.tf_w_in_v.child_frame_id = "tracker"
        pose_t_in_v = self.get_pose_vive(self.tracked_objects[0])
        self.tf_t_in_v = self.pose_to_tf(pose_t_in_v, "tracker")
        self.broadc.sendTransform(self.tf_t_in_v)

        self.tf_d_in_t = TransformStamped()
        self.tf_d_in_t.header.stamp = rospy.Time.now()
        self.tf_d_in_t.header.frame_id = "tracker"
        self.tf_d_in_t.child_frame_id = "init_drone"
        roll_d_in_t = np.pi  # np.pi/2
        pitch_d_in_t = 0  # -np.pi/2 #- 0.0559  # Tracker tilt
        yaw_d_in_t = -np.pi/2  # 0
        quat = tf.transformations.quaternion_from_euler(roll_d_in_t,
                                                        pitch_d_in_t,
                                                        yaw_d_in_t)
        self.tf_d_in_t.transform.translation.x = 0.
        self.tf_d_in_t.transform.translation.y = 0.015  # 0.025
        self.tf_d_in_t.transform.translation.z = 0.1
        self.tf_d_in_t.transform.rotation.x = quat[0]
        self.tf_d_in_t.transform.rotation.y = quat[1]
        self.tf_d_in_t.transform.rotation.z = quat[2]
        self.tf_d_in_t.transform.rotation.w = quat[3]

        self.stbroadc.sendTransform(self.tf_d_in_t)

        # To make sure drone-frame is not fixed to world, change "init drone"
        # to "drone".
        self.tf_d_in_t.child_frame_id = "drone"

    def start(self):
        '''
        Starts running of localization node.
        '''
        sample_time = rospy.get_param('vive_localization/sample_time', 0.02)
        self.rate = rospy.Rate(1./sample_time)

        self.v = triad_openvr.triad_openvr()
        # self.v.print_discovered_objects()
        if not self.v.devices:  # Check that this works!! Need to check whether
            # empty or not.
            print highlight_red('! Vive Error: No devices found !')
            return

        self.init_transforms()

        if not self.calib:
            # Start periodic publishing of measurements.
            self.publish_pose_est(Empty)

        rospy.spin()

    def calibrate(self, *_):

        print blue('---- Calibration started ---- \n')

        pose_t_in_v = self.get_pose_vive(self.tracked_objects[0])
        self.tf_t_in_v = self.pose_to_tf(pose_t_in_v, "tracker")
        self.broadc.sendTransform(self.tf_t_in_v)
        # self.stbroadc.sendTransform(self.tf_d_in_t)
        # Dirty fix to make sure every transform is broadcasted & received.
        rospy.sleep(2.)

        # Calibrate: fix current drone pose as pose of world frame.
        self.tf_w_in_v = self.get_transform("init_drone", "vive")
        self.tf_w_in_v.child_frame_id = "world"

        self.stbroadc.sendTransform(self.tf_w_in_v)
        print blue('tf_w_in_v \n'), self.tf_w_in_v

        print blue('---- Calibrated ---- \n')

    def publish_pose_est(self, *_):
        '''Publishes message that calibration is completed. Starts publishing
        pose measurements.
        '''
        self.ready.publish(Empty())
        print green('---- Vive Localization running ----')
        # rospy.sleep(10.)
        while not rospy.is_shutdown():
            # =========
            #  TRACKER
            # =========
            pose_t_in_v = self.get_pose_vive(self.tracked_objects[0])
            self.tf_t_in_v = self.pose_to_tf(pose_t_in_v, "tracker")

            self.broadc.sendTransform(self.tf_t_in_v)
            self.stbroadc.sendTransform(self.tf_d_in_t)

            # Calculate and publish pose of drone in world frame and also
            # include yaw.
            tf_d_in_w = TransformStamped()
            tf_d_in_w = self.get_transform("drone", "world")
            pose_t_in_w = self.tf_to_pose(tf_d_in_w)

            # Calculate and broadcast the rotating world frame.
            # - Tf drone in world to euler angles.
            euler = self.get_euler_angles(tf_d_in_w)
            # - Get yaw.
            yaw = euler[2]

            data = PoseMeas(meas_world=pose_t_in_w, yaw=yaw)
            self.pos_update.publish(data)

            # - Yaw only (roll and pitch 0.0) to quaternions.
            quat = tf.transformations.quaternion_from_euler(0., 0., yaw)
            self.tf_r_in_w.transform.rotation.x = quat[0]
            self.tf_r_in_w.transform.rotation.y = quat[1]
            self.tf_r_in_w.transform.rotation.z = quat[2]
            self.tf_r_in_w.transform.rotation.w = quat[3]
            self.broadc.sendTransform(self.tf_r_in_w)

            # =============
            #  CONTROLLERS
            # =============
            for i in range(1, len(self.tracked_objects)):
                # controller_1 (& 2)
                pose_c_in_v = self.get_pose_vive(self.tracked_objects[i])
                pose_c_in_w = self.transform_pose(pose_c_in_v, "vive", "world")

                self.c_publishers[i-1].publish(pose_c_in_w)

            self.rate.sleep()

    def get_pose_vive(self, object):
        '''Returns PoseStamped of the i'th object in self.tracked_objects.
        Pose is expressed in vive reference frame.
        '''
        pose = np.array(
            self.v.devices[object].get_pose_euler())

        pose[3:6] = pose[3:6]*np.pi/180.
        # # Rotate controller 90 to align rviz arrow with wand.
        # if object[0:10] == "controller":
        #     pose[3] += np.pi/2.

        quat = tf.transformations.quaternion_from_euler(
            pose[5], pose[4], pose[3])

        pose_t_in_v = PoseStamped()
        pose_t_in_v.header.frame_id = "vive"
        pose_t_in_v.header.stamp = rospy.Time.now()
        pose_t_in_v.pose.position.x = pose[0]
        pose_t_in_v.pose.position.y = pose[1]
        pose_t_in_v.pose.position.z = pose[2]
        pose_t_in_v.pose.orientation.x = quat[0]
        pose_t_in_v.pose.orientation.y = quat[1]
        pose_t_in_v.pose.orientation.z = quat[2]
        pose_t_in_v.pose.orientation.w = quat[3]

        return pose_t_in_v

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


if __name__ == '__main__':
    localization = ViveLocalization()
    localization.start()
