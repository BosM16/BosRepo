#!/usr/bin/env python

from std_msgs.msg import Empty, UInt8
from geometry_msgs.msg import Twist, Pose
import rospy
import numpy as np
import scipy.io as io


class Try_out(object):

    def __init__(self):
        """
        """
        self.input = np.array([])
        self.output_x = np.array([])
        self.output_y = np.array([])
        self.output_z = np.array([])
        self.vel = Twist()

        # use this for bebop_vel_ctrl + bebop_autonomy
        self.cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self.take_off = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)
        self.land = rospy.Publisher('bebop/land', Empty, queue_size=1)
        rospy.Subscriber('demo', Empty, self.flying)
        rospy.Subscriber('vive/pose', Pose, self.update_pose)

    def start(self):
        rospy.init_node('try_out')
        print 'started'
        rospy.spin()

    def flying(self, empty):

        self.take_off.publish(Empty())
        print 'eagle is flying'

        rospy.sleep(4)

        rate = 14

        # move back and forth with a pause in between
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.measuring = True

        for k in range(0, 5):
            self.vel.linear.y = -0.4

            for x in range(0, rate):
                self.cmd_vel.publish(cmd_vel)
                rospy.sleep(0.1)

            # brake
            self.vel.linear.y = 0.0
            self.cmd_vel.publish(cmd_vel)
            rospy.sleep(1.0)

            self.vel.linear.y = 0.4

            for x in range(0, rate):
                self.cmd_vel.publish(cmd_vel)
                rospy.sleep(0.1)

            # brake
            self.vel.linear.y = 0.0
            self.cmd_vel.publish(cmd_vel)
            rospy.sleep(1.0)

        self.measuring = False
        print 'stop', cmd_vel
        self.cmd_vel.publish(self.vel)

        rospy.sleep(1)

        self.land.publish(Empty())
        print 'eagle has landed'

        meas = {}
        meas['inputs'] = self.input
        meas['output_x'] = self.output_x
        meas['output_y'] = self.output_y
        meas['output_z'] = self.output_z
        io.savemat('identification', meas)

    def update_pose(self, pose):
        if self.measuring:
            self.input = np.append(self.input, self.vel.linear.y)
            self.output_x = np.append(self.output_x, pose.position.x)
            self.output_y = np.append(self.output_y, pose.position.y)
            self.output_z = np.append(self.output_z, pose.position.z)


if __name__ == '__main__':
    Billy = Try_out()
    Billy.start()
