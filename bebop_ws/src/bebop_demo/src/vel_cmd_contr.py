#!/usr/bin/env python

from geometry_msgs.msg import Twist, TwistStamped, Pose, Pose2D, Point
from std_msgs.msg import Bool, Empty, UInt8
from bebop_demo.srv import GetPoseEst
from visualization_msgs.msg import MarkerArray

import rospy


class VelCommander(object):
    _cmd_twist = Twist()
    _trigger = Trigger()

    def __init__(self):
        """Initialization of Controller object.

        Args:
            sample_time : Period between computed velocity samples.
            update_time : Period of problem solving.
        """
        rospy.init_node("vel_commander_node")

        self._sample_time = rospy.get_param('vel_cmd/sample_time', 0.01)
        self.rate = rospy.Rate(1./self._sample_time)

        # TOPIC where reference velocity in m/s is published!
        rospy.Subscriber('bebop/cmd_vel', Twist, self.store_cmd_vel)

    def update(self):
        """
        - Updates the controller with newly calculated trajectories and
        velocity commands.
        - Sends out new velocity command.
        - Retrieves new pose estimate.
        """
        # Send velocity sample to WorldModel, receive position estimate.
        # IMPORTANT NOTE: only in this order because cmd_vel is published by
        # joy_teleop, and not calculated in this node.

        # Read cmd_vel
        self.get_vel_cmd()

        # Retrieve new pose estimate from World Model.
        self.get_pose_est()  # opgelet: dit is een predictie voor het volgende
        # tijdstip.

    def store_vel_cmd(self, cmd_vel):
        '''Combines the feedforward and feedback commands to generate a
        velocity command and publishes this command.
        '''
        self._cmd_twist = cmd_vel

    def get_pose_est(self):
        '''Retrieves a new pose estimate from world model.
        '''

        cmd_twist = TwistStamped()
        cmd_twist.header.stamp = rospy.Time.now()
        cmd_twist.twist = self._cmd_twist

        rospy.wait_for_service("get_pose")
        try:
            pose_est = rospy.ServiceProxy(
                "get_pose", GetPoseEst)
            self._robot_est_pose = pose_est(cmd_twist)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def start(self):
        """Starts the controller's periodical loop.
        """
        rate = self.rate
        print 'controller started!'

        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


if __name__ == '__main__':
    vel_command = VelCommander()
    vel_command.start()
