#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose2D
import rospy

from perception import *
from world_model import *
from vel_cmd_contr import *


class LocalizationTest(object):
    '''
    Switches to the desired state depending on task at hand.
    '''

    def __init__(self):
        '''
        Initialization of Demo object.
        '''
        rospy.init_node('bebop_demo')

        self.pos_update = rospy.Publisher(
            'pose_est', PoseStamped, queue_size=1)

    def start(self):
        '''
        Starts running of bebop_demo node.
        '''
        rospy.spin()


if __name__ == '__main__':
    test = LocalizationTest()
    test.pc = Perception()
    test.wm = WorldModel()
    test.start()
