#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose2D
import rospy

from perception import *
from world_model import *
from vel_cmd_contr import *


class Fsm(object):
    '''
    Switches to the desired state depending on task at hand.
    '''

    def __init__(self):
        """
        Initialization of Fsm object.
        """
        rospy.init_node('fsm')

        self.pos_update = rospy.Publisher('pose_est', Pose2D, queue_size=1)
        rospy.Subscriber('bebop/cmd_vel', Twist, self.kalman_predict)
        rospy.Subscriber('twist1_pub_', Twist, self.kalman_correct)

    def start(self):
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

    def kalman_correct(self, data):
        # timing data to know length of preceding prediction step necessary
        '''
        Checks whether perception has received new position info from the drone
        and then triggers the kalman filter to apply a correction step.
        '''
        while not self.percep.new_val:
            rospy.sleep(0.00001)  # moet beter kunnen

        pos_output = Pose2D
        pos_output.x = self.percep.pose_vive.linear.x
        pos_output.y = self.percep.pose_vive.linear.y
        self.wm.correct_update(pos_output)

if __name__ == '__main__':
    fsm = Fsm()
    fsm.percep = Perception()
    fsm.wm = WorldModel()
    fsm.start()
