#!/usr/bin/env python

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import numpy as np
import rospy
import sys
import triad_openvr
import time


class LocalizationTest(object):
    '''
    Switches to the desired state depending on task at hand.
    '''

    def __init__(self):
        '''
        Initialization of Demo object.
        '''
        self.tracked_object = 'tracker'

        self.pose_est_world = Twist()
        self.T_w_to_h = np.identity(4)
        self.T_h_to_w = np.identity(4)

        self.pos_update = rospy.Publisher('pose_est', Twist, queue_size=1)

        rospy.Subscriber('publish_poses', Empty, self.publish_pose_est)
        rospy.Subscriber('calibrate', Empty, self.calibrate)

    def start(self):
        '''
        Starts running of bebop_demo node.
        '''
        rospy.init_node('bebop_demo')
        print 'Lcalization test is ON'
        self.v = triad_openvr.triad_openvr()
        self.v.print_discovered_objects()
        rospy.spin()

    def publish_pose_est(self, *_):
        # print 'in publish pose est'
        (pose_est_vive, pose_vive) = self.get_pose_est()

        pos_vive = np.append(pose_vive[0:3], 1.)

        print '--------------------------- \n'
        print 'new position estimate \n'
        print 'pos_vive', pos_vive
        pos_world = np.matmul(self.T_h_to_w, pos_vive)
        print 'pos world', pos_world
        self.pose_est_world.linear.x = pos_world[0]
        self.pose_est_world.linear.y = pos_world[1]
        self.pose_est_world.linear.z = pos_world[2]
        # print 'publish'
        #  print self.pose_est_world
        self.pos_update.publish(self.pose_est_world)

    def calibrate(self, *_):
        pose_est, pose = self.get_pose_est()

        transl_w_in_h = pose[0:3]
        print 'translation \n', transl_w_in_h
        roll = pose_est.angular.x
        pitch = pose_est.angular.y
        yaw = pose_est.angular.z

        R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                          [np.sin(yaw),  np.cos(yaw), 0.],
                          [0.,            0.,           1.]])

        R_pitch = np.array([[np.cos(pitch),  0., np.sin(pitch)],
                            [0.,             1.,            0.],
                            [-np.sin(pitch), 0., np.cos(pitch)]])

        R_roll = np.array([[1.,           0.,            0.],
                           [0., np.cos(roll), -np.sin(roll)],
                           [0., np.sin(roll),  np.cos(roll)]])

        print 'Rotation matrices \n'
        print 'roll \n', R_roll
        print 'pitch \n', R_pitch
        print 'yaw \n', R_yaw

        R_tracker_in_vive = np.matmul(R_yaw, np.matmul(R_pitch, R_roll))
        # Tracker z-axis points down. Add extra rotation 180 deg roll.
        R_world_in_tracker = np.array([[1.,  0.,  0.],
                                       [0., -1.,  0.],
                                       [0.,  0., -1.]])
        R_tracker_in_vive = np.matmul(R_tracker_in_vive, R_world_in_tracker)

        print 'rotation matrix \n', R_tracker_in_vive

        self.T_w_to_h[0:3, 3] = transl_w_in_h
        self.T_w_to_h[0:3, 0:3] = R_tracker_in_vive

        R_vive_in_tracker = np.transpose(R_tracker_in_vive)
        print 'rotation matrix inverse \n', R_vive_in_tracker
        self.T_h_to_w[0:3, 3] = - np.matmul(R_vive_in_tracker, transl_w_in_h)
        self.T_h_to_w[0:3, 0:3] = R_vive_in_tracker

        print 'T world to htc \n', self.T_w_to_h
        print 'T htc to world \n', self.T_h_to_w
        print 'T world to htc inverted \n', np.linalg.inv(self.T_w_to_h)

    def get_pose_est(self):
        # controller_1 must become tracker_1 when working with the tracker.
        if self.tracked_object is 'controller':
            pose = np.array(self.v.devices["controller_1"].get_pose_euler())
        elif self.tracked_object is 'tracker':
            pose = np.array(self.v.devices["tracker_1"].get_pose_euler())

        else:
            print 'No device found.'
            return
        # print 'pose deg \n', pose
        pose[3:6] = pose[3:6]*np.pi/180.

        pose_est = Twist()
        pose_est.linear.x = pose[0]
        pose_est.linear.y = pose[1]
        pose_est.linear.z = pose[2]

        pose_est.angular.x = pose[5]
        pose_est.angular.y = pose[4]
        pose_est.angular.z = pose[3]

        # print 'pose rad \n', pose
        return (pose_est, pose)


if __name__ == '__main__':
    test = LocalizationTest()
    test.start()
