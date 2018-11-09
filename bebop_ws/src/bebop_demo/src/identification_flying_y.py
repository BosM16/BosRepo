#!/usr/bin/env python

from std_msgs.msg import Empty, UInt8
from geometry_msgs.msg import Twist, Pose, PoseStamped
import rospy
import numpy as np
import scipy.io as io


class Ident(object):

    def __init__(self):
        """
        """
        self.ident_length = 10
        self.index = 0
        self.rate = 12
        self.wait1 = 0.1
        self.wait2 = 0.12
        self.wait3 = 0.10
        span = int(self.ident_length*(
                    self.wait1*4 + self.wait2+self.wait3)*self.rate*50 + 10)
        self.input = np.zeros(span)
        self.output_x = np.zeros(span)
        self.output_y = np.zeros(span)
        self.output_z = np.zeros(span)
        self.vel = Twist()
        self.measuring = False

        self.cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self.take_off = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)
        self.land = rospy.Publisher('bebop/land', Empty, queue_size=1)
        rospy.Subscriber('demo', Empty, self.flying)
        rospy.Subscriber(
            'vive_localization/pose', PoseStamped, self.update_pose)

    def start(self):
        rospy.init_node('identification')
        print 'started'
        rospy.spin()

    def flying(self, empty):

        self.take_off.publish(Empty())
        print 'Billie is flying'

        rospy.sleep(4)
        velocity_max = 0.6

        # move back and forth with a pause in between
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.measuring = True

        for k in range(0, self.ident_length):
            self.vel.linear.y = velocity_max/3.

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait1)

            self.vel.linear.y = -velocity_max/3.

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait2)

            self.vel.linear.y = velocity_max/3.*2.

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait1)

            self.vel.linear.y = -velocity_max/3.*2.

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait3)

            self.vel.linear.y = velocity_max

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait1)

            self.vel.linear.y = -velocity_max

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait1)

        self.measuring = False

        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.cmd_vel.publish(self.vel)

        rospy.sleep(1)

        self.land.publish(Empty())
        print 'Billie has landed'

        meas = {}
        meas['input'] = self.input
        meas['output_x'] = self.output_x
        meas['output_y'] = self.output_y
        meas['output_z'] = self.output_z
        io.savemat('../angle_identification_y.mat', meas)

    def update_pose(self, pose):
        if self.measuring:
            self.input[self.index] = self.vel.linear.y
            self.output_x[self.index] = pose.pose.position.x
            self.output_y[self.index] = pose.pose.position.y
            self.output_z[self.index] = pose.pose.position.z
            self.index += 1


if __name__ == '__main__':
    Billy = Ident()
    Billy.start()
