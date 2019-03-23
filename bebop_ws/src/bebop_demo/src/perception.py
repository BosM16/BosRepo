#!/usr/bin/env python

from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry

import numpy as np
import tf2_ros
import rospy

from fabulous.color import highlight_red


class Perception(object):
    '''
    Read out position of the drone, pose of the drone and position of the
    obstacles
    '''

    def __init__(self):
        """
        Initialization of Perception object.
        """
        self.pose_vive = TwistStamped()
        self.pose_bebop = Twist()
        self.twist_bebop = Twist()
        self.tf_t_in_w_prev = TransformStamped()
        self.init = True

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Name of topic can change depending on name used in the code for
        # reading out the vive.
        rospy.Subscriber('/bebop/odom', Odometry, self.get_bebop_data)

    def get_bebop_data(self, data):
        """
        Updates pose and twist data by using measurements on the bebop itself.
        """
        self.pose_bebop = data.pose.pose
        self.twist_bebop = data.twist.twist

    def measurement_check(self):
        '''Monitor function: checks measurement.
        If the measurement equals the vive frame origin, this means that
        vibrations cause a false measurement.

        Should be moved to monitor later.
        '''
        tf_v_in_w = self.get_transform("vive", "world")
        tf_t_in_w = self.get_transform("tracker", "world")

        meas_distance = np.linalg.norm(
            np.array([tf_t_in_w.transform.translation.x,
                      tf_t_in_w.transform.translation.y,
                      tf_t_in_w.transform.translation.z])
            - np.array([self.tf_t_in_w_prev.transform.translation.x,
                        self.tf_t_in_w_prev.transform.translation.y,
                        self.tf_t_in_w_prev.transform.translation.z]))
        self.tf_t_in_w_prev = tf_t_in_w

        measurement_valid = not (
            (tf_v_in_w.transform == tf_t_in_w.transform) or
            (meas_distance > 0.25))
        if not measurement_valid:
            if not self.init:
                print highlight_red(' Warning: invalid measurement!')
            else:
                self.init = False

        return measurement_valid

    def get_transform(self, _from, _to):
        '''
        '''
        tf_f_in_t = self.tfBuffer.lookup_transform(
            _to, _from, rospy.Time(0), rospy.Duration(0.1))

        return tf_f_in_t


if __name__ == '__main__':
    perception = Perception()
