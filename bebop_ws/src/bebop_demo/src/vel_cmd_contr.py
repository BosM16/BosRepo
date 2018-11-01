#!/usr/bin/env python

from geometry_msgs.msg import Twist, TwistStamped, Pose, Pose2D, Point
from std_msgs.msg import Bool, Empty, UInt8
from bebop_demo.srv import GetPoseEst
from visualization_msgs.msg import MarkerArray

import rospy


class VelCommander(object):

    def __init__(self):
        """Initialization of Controller object.

        Args:
            sample_time : Period between computed velocity samples.
            update_time : Period of problem solving.
        """
        # rospy.init_node("vel_commander_node")

        # self.cmd_vel_received = False

        # self.cmd_twist_st = TwistStamped()

        # self._sample_time = rospy.get_param('vel_cmd/sample_time', 0.01)
        # self.rate = rospy.Rate(1./self._sample_time)

        # TOPIC where reference velocity in m/s is published!
        # rospy.Subscriber('bebop/cmd_vel', Twist, self.store_cmd_vel)

    # def start(self):
    #     """Starts the controller's periodical loop.
    #     """
    #     rate = self.rate
    #     print '------------------- \ncontroller started!\n-------------------'
    #
    #     while not rospy.is_shutdown():
    #         if self.cmd_vel_received:
    #             self.update()
    #         rate.sleep()

    def update(self):
        """
        - Sends out new velocity command.
        - Retrieves new pose estimate.
        """
        # Send velocity sample to WorldModel, receive position estimate.
        # IMPORTANT NOTE: only in this order because cmd_vel is published by
        # joy_teleop, and not calculated in this node.

        # Retrieve new pose estimate from World Model.
        self.get_pose_est()  # opgelet: dit is een predictie voor het volgende
        # tijdstip.

    def store_cmd_vel(self, cmd_vel):
        '''Combines the feedforward and feedback commands to generate a
        velocity command and publishes this command.
        '''
        self.cmd_twist_st.header.stamp = rospy.Time.now()
        self.cmd_twist_st.header.frame_id = "world_rot"
        self.cmd_twist_st.twist = cmd_vel

        self.cmd_vel_received = True

    def get_pose_est(self):
        '''Retrieves a new pose estimate from world model.
        '''
        # print '-- VelCmd wait for service'
        rospy.wait_for_service("/world_model/get_pose")
        # print '-- Velcmd service found'
        try:
            pos_est = rospy.ServiceProxy(
                "/world_model/get_pose", GetPoseEst)
            # print '-- VelCmd Service call --'
            self.xhat = pos_est(self.cmd_twist_st)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


if __name__ == '__main__':
    vel_command = VelCommander()
    vel_command.start()
