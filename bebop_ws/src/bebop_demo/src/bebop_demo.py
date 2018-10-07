#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose2D
from ???.srv import GetPoseEst, GetPoseEstResponse, GetPoseEstRequest
import rospy

from perception import *
from world_model import *
from vel_cmd_contr import *


class Demo(object):
    '''
    Switches to the desired state depending on task at hand.
    '''
    # make buffer containing all velocity commands up to timestamp of last
    # kalman correction step

    def __init__(self):
        '''
        Initialization of Demo object.
        '''
        rospy.init_node('bebop_demo')

        self.pos_update = rospy.Publisher('pose_est', Pose2D, queue_size=1)
        rospy.Subscriber('bebop/cmd_vel', Twist, self.kalman_pos_predict)
        rospy.Subscriber('twist1_pub_', Twist, self.kalman_pos_correct)
        rospy.Service("get_pose", GetPoseEst,
                      self.DO_KALMAN_SHIZZLE)

    def start(self):
        '''
        Starts running of bebop_demo node.
        '''
        rospy.spin()

    def DO_KALMAN_SHIZZLE(self, vel_cmd):
        '''
        '''
        vel_cmd.linear.x
        vel_cmd.linear.y
        pose_est = Pose2D()
        return GetPoseEstResponse(pose_est)

    def kalman_pos_predict(self, data):
        '''
        Based on the velocity commands send out by the velocity controller,
        calculate a prediction of the position in the future.
        '''
        self.wm.predict_pos_update(data)

        pose_est = Pose2D()
        pose_est.x = self.wm.xhat.x
        pose_est.y = self.wm.xhat.y
        self.pos_update.publish(pose_est)

    def kalman_pos_correct(self, data):
        # timing data to know length of preceding prediction step necessary
        '''
        Whenever a new position measurement is available, sends this
        information to the perception and then triggers the kalman filter
        to apply a correction step.
        '''
        self.pc.pose_vive = data  # data moet nog verwerkt worden naar gewenste formaat
        self.wm.correct_pos_update(self.pc.pose_vive)
        self.pc.new_val = False


if __name__ == '__main__':
    demo = Demo()
    demo.pc = Perception()
    demo.wm = WorldModel()
    demo.start()
