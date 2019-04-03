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
        self.rate = 10
        self.wait = 0.1
        span = int(self.ident_length*(self.wait*10)*self.rate*50 + 15)
        self.input = np.zeros(span)
        self.output_yaw = np.zeros(span)
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
        velocity_max = 0.8

        # Rotate back and forth with a pause in between.
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0
        self.measuring = True

        for k in range(0, self.ident_length):
            self.vel.angular.z = velocity_max/3.
            # Le rise
            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait)

            self.vel.angular.z = velocity_max/3.*2

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait)

            self.vel.angular.z = velocity_max

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait)

            self.vel.angular.z = velocity_max/3.*2

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait)

            self.vel.angular.z = velocity_max/3.

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait)

            # Le fall
            self.vel.angular.z = -velocity_max/3.

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait)

            self.vel.angular.z = -velocity_max/3.*2

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait)

            self.vel.angular.z = -velocity_max

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait)

            self.vel.angular.z = -velocity_max/3.*2

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait)

            self.vel.angular.z = -velocity_max/3.

            for x in range(0, self.rate):
                self.cmd_vel.publish(self.vel)
                rospy.sleep(self.wait)

        self.measuring = False

        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0
        self.cmd_vel.publish(self.vel)

        rospy.sleep(1)

        self.land.publish(Empty())
        print 'Billie has landed'

        meas = {}
        meas['input'] = self.input
        meas['output_roll'] = self.output_roll
        meas['output_pitch'] = self.output_pitch
        meas['output_yaw'] = self.output_yaw
        io.savemat('../vel_identification_yaw.mat', meas)

    def update_pose(self, pose):
        if self.measuring:
            self.input[self.index] = self.vel.angular.z
            euler = tf.transformations.euler_from_quaternion(pose.pose.orientation)
            self.output_roll[self.index] = euler[0]
            self.output_pitch[self.index] = euler[1]
            self.output_yaw[self.index] = euler[2]
            self.index += 1


if __name__ == '__main__':
    Billy = Ident()
    Billy.start()
