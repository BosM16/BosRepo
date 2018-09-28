#!/usr/bin/env python

from std_msgs.msg import Empty, UInt8
from geometry_msgs.msg import Twist
import rospy


class Try_out(object):

    def __init__(self):
        """
        """
        self.cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self.take_off = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)
        self.land = rospy.Publisher('bebop/land', Empty, queue_size=1)
        self.flip = rospy.Publisher('bebop/flip', UInt8, queue_size=1)
        rospy.Subscriber('demo', Empty, self.flying)

    def start(self):
        rospy.init_node('try_out')
        print 'started'
        rospy.spin()

    def flying(self, empty):
        print 'flying'
        self.take_off.publish(Empty())
        rospy.sleep(4)

        cmd_vel = Twist()
        rate = 10

        k = 0.0

        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = -0.4
        cmd_vel.linear.z = 0.0

        # cmd_vel.angular.x = 0.0
        # cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0


        for x in range(0, rate):
            self.cmd_vel.publish(cmd_vel)
            rospy.sleep(0.1)

        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        self.cmd_vel.publish(cmd_vel)
        rospy.sleep(0.6)

        cmd_vel.linear.x = 0.4
        cmd_vel.linear.y = 0.0

        for x in range(0, rate):
            self.cmd_vel.publish(cmd_vel)
            rospy.sleep(0.1)

        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        self.cmd_vel.publish(cmd_vel)
        rospy.sleep(0.6)

        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.4

        for x in range(0, rate):
            self.cmd_vel.publish(cmd_vel)
            rospy.sleep(0.1)

        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        self.cmd_vel.publish(cmd_vel)
        rospy.sleep(0.6)

        cmd_vel.linear.x = -0.4
        cmd_vel.linear.y = 0.0

        for x in range(0, rate):
            self.cmd_vel.publish(cmd_vel)
            rospy.sleep(0.1)

        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0

        # cmd_vel.angular.x = 0.0
        # cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel.publish(cmd_vel)
        rospy.sleep(1.5)

        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = -0.4

        for x in range(0, rate):
            self.cmd_vel.publish(cmd_vel)
            rospy.sleep(0.1)

        cmd_vel.linear.x = 0.4
        cmd_vel.linear.y = 0.0

        for x in range(0, rate):
            self.cmd_vel.publish(cmd_vel)
            rospy.sleep(0.1)

        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.4

        for x in range(0, rate):
            self.cmd_vel.publish(cmd_vel)
            rospy.sleep(0.1)

        cmd_vel.linear.x = -0.4
        cmd_vel.linear.y = 0.0

        for x in range(0, rate):
            self.cmd_vel.publish(cmd_vel)
            rospy.sleep(0.1)

        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0

        # cmd_vel.angular.x = 0.0
        # cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0

        print 'stop', cmd_vel
        self.cmd_vel.publish(cmd_vel)

        # cmd_vel.linear.y = 0.0
        # for x in range(0, rate):
        #     self.cmd_vel.publish(cmd_vel)
        #     rospy.sleep(1/rate)
        #
        # rospy.sleep(1)
        #
        # cmd_vel.linear.y = 0.4
        # for x in range(0, rate):
        #     self.cmd_vel.publish(cmd_vel)
        #     rospy.sleep(1/rate)
        #
        # rospy.sleep(1)
        #
        # cmd_vel.linear.y = 0.0
        # for x in range(0, rate):
        #     self.cmd_vel.publish(cmd_vel)
        #     rospy.sleep(1/rate)
        #
        # rospy.sleep(1)
        #
        # cmd_vel.linear.y = -0.4
        # for x in range(0, rate):
        #     self.cmd_vel.publish(cmd_vel)
        #     rospy.sleep(1/rate)
        #
        # rospy.sleep(1)
        #
        # cmd_vel.linear.y = 0.0
        # for x in range(0, rate):
        #     self.cmd_vel.publish(cmd_vel)
        #     rospy.sleep(1/rate)
        #
        # rospy.sleep(1)
        #
        # cmd_vel.linear.y = 0.4
        # for x in range(0, rate):
        #     self.cmd_vel.publish(cmd_vel)
        #     rospy.sleep(1/rate)
        #
        # rospy.sleep(1)
        #
        # cmd_vel.linear.y = 0.0
        # for x in range(0, rate):
        #     self.cmd_vel.publish(cmd_vel)
        #     rospy.sleep(1/rate)

        rospy.sleep(1)

        self.land.publish(Empty())
        print 'eagle has landed'


if __name__ == '__main__':
    Billy = Try_out()
    Billy.start()
