#!/usr/bin/env python

from std_msgs.msg import Empty, UInt8
from geometry_msgs.msg import Twist, Pose, PoseStamped
from vive_localization.msg import PoseMeas
import rospy
import numpy as np
import scipy.io as io


class Ident(object):

    def __init__(self):
        """
        """
        self.ident_length = 50*10  # 50 pts/second * x s
        self.index = 0
        self.input = np.zeros(self.ident_length)
        self.pos_x = np.zeros(self.ident_length)
        self.pos_y = np.zeros(self.ident_length)
        self.pos_z = np.zeros(self.ident_length)
        self.time = np.zeros(self.ident_length)
        self.yaw = np.zeros(self.ident_length)
        self.measuring = False

        rospy.Subscriber('identification', Empty, self.record)
        rospy.Subscriber(
            'vive_localization/pose', PoseMeas, self.update_pose)

    def start(self):
        rospy.init_node('identification')
        print 'Ourbot identification is ready to go.'
        rospy.spin()

    def record(self, empty):

        print 'Measurements started'
        self.measuring = True

        while self.index < self.ident_length and not rospy.is_shutdown():
            rospy.sleep(0.01)
        self.measuring = False
        print 'Measurements finished'
        self.index = 0

        meas = {}
        meas['description'] = "all commands 02"
        meas['input'] = self.input
        meas['pos_x'] = self.pos_x
        meas['pos_y'] = self.pos_y
        meas['pos_z'] = self.pos_z
        meas['yaw'] = self.yaw
        meas['time'] = self.time
        io.savemat('/home/mathias/recordings/all02.mat', meas)
        print 'Log saved'

    def update_pose(self, meas):
        if self.measuring:
            self.pos_x[self.index] = meas.meas_world.pose.position.x
            self.pos_y[self.index] = meas.meas_world.pose.position.y
            self.pos_z[self.index] = meas.meas_world.pose.position.z
            self.yaw[self.index] = meas.yaw
            self.time[self.index] = meas.meas_world.header.stamp.to_sec()
            self.index += 1


if __name__ == '__main__':
    Billy = Ident()
    Billy.start()
