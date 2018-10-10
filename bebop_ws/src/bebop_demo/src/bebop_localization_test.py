#!/usr/bin/env python

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import rospy
import triad_openvr
import time
import sys


class LocalizationTest(object):
    '''
    Switches to the desired state depending on task at hand.
    '''

    def __init__(self):
        '''
        Initialization of Demo object.
        '''

        self.pub_rate = 60.0  # Hz

        self.pos_update = rospy.Publisher(
            'pose_est', Twist, queue_size=1)

        rospy.Subscriber('publish_poses', Empty, self.publish_pose_est)

    def start(self):
        '''
        Starts running of bebop_demo node.
        '''
        rospy.init_node('bebop_demo')
        print 'loc test is ON'
        rospy.spin()

    def publish_pose_est(self, ignore):
        print 'in publish pose est'
        v = triad_openvr.triad_openvr()
        v.print_discovered_objects()
        print 'printed discovered objects'

        # if len(sys.argv) == 1:
        interval = 1/self.pub_rate
        # elif len(sys.argv) == 2:
        #     interval = 1/float(sys.argv[0])
        # else:
        #     print("Invalid number of arguments")
        #     interval = False

        print 'interval is set'
        print interval
        if interval:
            print 'in if'
            while(True):
                print 'in while'
                start = time.time()
                # txt = ""
                # for each in v.devices["tracker_1"].get_pose_euler():
                #     txt += "%.4f" % each
                #     txt += " "
                # print("\r" + txt, end="")

                # Bos: make a Pose out of the read list and publish it.
                pose = v.devices["tracker_1"].get_pose_euler()
                pose_est = Twist()
                pose_est.linear.x = pose[0]
                pose_est.linear.y = pose[1]
                pose_est.linear.z = pose[2]

                pose_est.angular.x = pose[5]
                pose_est.angular.y = pose[4]
                pose_est.angular.z = pose[3]

                print 'publish'
                self.pos_update.publish(pose_est)

                sleep_time = interval-(time.time()-start)
                if sleep_time > 0:
                    time.sleep(sleep_time)


if __name__ == '__main__':
    test = LocalizationTest()
    test.start()
