#!/usr/bin/env python

from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
import rospy


class Bebop(object):

    def __init__(self):
        """
        """
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.take_off = rospy.Publisher('takeoff', Empty, queue_size=1)
        self.land = rospy.Publisher('land', Empty, queue_size=1)
        self.flip = rospy.Publisher('flip', Empty, queue_size=1)
        rospy.Subscriber('demo', Empty, self.flying)

    def start(self):
        rospy.init_node('try_out')
        print 'started'
        rospy.spin()

    def flying(self):
        print 'flying'
        self.take_off.publish(Empty())
        sleep(5)
        cmd_vel = Twist()
        cmd_vel.linear.y = 0.4
        self.cmd_vel.publish(cmd_vel)
        sleep(0.5)
        cmd_vel.linear.y = -0.4
        self.cmd_vel.publish(cmd_vel)
        sleep(0.5)
        self.land.publish(Empty())
        print 'eagle has landed'


if __name__ == '__main__':
    Billy = Bebop()
    Billy.start()
