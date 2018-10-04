#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose, Pose2D
from std_msgs.msg import Bool, Empty, UInt8
import rospy


class Monitor(object):
    '''
    Monitors whether a heartbeat is received, monitors state of the fsm and
    triggers events to determine state of controller, perception etc.
    '''
    def __init__(self):
        """Initialization of Controller object.

        Args:
            sample_time : Period between computed velocity samples.
            update_time : Period of problem solving.
        """


if __name__ == '__main__':
