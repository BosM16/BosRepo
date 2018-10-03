#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose2D
import rospy

from perception import *
from world_model import *
from vel_cmd_contr import *


class Demo(object):
    '''
    Switches to the desired state depending on task at hand.
    '''

    def __init__(self):
        '''
        Initialization of Demo object.
        '''
        rospy.init_node('bebop_demo')

        # BOS: self.pos_update zou world model moeten publishen + wm ook
        # Subscriber maken?
        self.pos_update = rospy.Publisher('pose_est', Pose2D, queue_size=1)
        rospy.Subscriber('bebop/cmd_vel', Twist, self.kalman_pos_predict)
        rospy.Subscriber('twist1_pub_', Twist, self.kalman_pos_correct)

    def start(self):
        '''
        Starts running of bebop_demo node.
        '''
        rospy.spin()

    def kalman_predict(self, data):
        '''
        Based on the velocity commands send out by the velocity controller,
        calculate a prediction of the position in the future.
        '''
        self.wm.predict_update(data)

        pose_est = Pose2D()
        pose_est.x = self.wm.xhat.x
        pose_est.x = self.wm.xhat.x
        self.pos_update.publish(pose_est)

    def kalman_pos_correct(self, data):
        # timing data to know length of preceding prediction step necessary
        '''
        Checks whether perception has received new position info from the drone
        and then triggers the kalman filter to apply a correction step.
        '''
        while not self.pc.new_val:
            rospy.sleep(0.00001)  # moet beter kunnen

        pos_output = Pose2D
        pos_output.x = self.pc.pose_vive.linear.x
        pos_output.y = self.pc.pose_vive.linear.y
        self.wm.correct_pos_update(pos_output)


if __name__ == '__main__':
    demo = Demo()
    demo.pc = Perception()
    demo.wm = WorldModel()
    demo.start()
