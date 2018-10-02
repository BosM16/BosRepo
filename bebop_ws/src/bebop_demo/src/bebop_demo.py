#!/usr/bin/env python

from geometry_msgs.msg import Vector3, Point
import rospy

from perception import *
from world_model import *
from vel_cmd_contr import *


class Demo(object):
    '''
    Switches to the desired state depending on task at hand
    '''

    def __init__(self):
        """
        Initialization of Demo object.
        """
        rospy.init_node('fsm')
        vel_input = Vector3()
        pos_output = Point()

    def update(self):
        """
        TIMING!!!!!!!!!!!!!
        """
        self.vel_cmd._robot_est_pose.x = self.wm.xhat.x
        self.vel_cmd._robot_est_pose.y = self.wm.xhat.y
        # na update van positie, bereken nieuwe snelheden die uitgezonden
        # worden en leg deze op aan de drone, gebruik deze snelheden dan om te
        # berekenen waar de drone in de volgende cyclus gaat zijn
        vel_input.linear.x = self._cmd_twist.linear.x
        vel_input.linear.y = self._cmd_twist.linear.y
        self.wm.predict_update(vel_input)

        # If an update from prediction regarding the position of the drone is
        # received, a correction step is carried out
        pos_output.x = self.percep.pose_vive.linear.x
        pos_output.y = self.percep.pose_vive.linear.y
        self.wm.correct_update(pos_output)


if __name__ == '__main__':
    demo = Demo()
    demo.vel_cmd = VelCommander()
    demo.percep = Perception()
    demo.wm = WorldModel()
    demo.vel_cmd.start()
