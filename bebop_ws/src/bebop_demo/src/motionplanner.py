#!/usr/bin/env python

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import numpy as np
import rospy

from omg_ros_nav_bridge.msg import (
    Trigger, Trajectory, Settings)

import omgtools as omg


class MotionPlanner(object):
    _result = RobotTrajectory

    def __init__(self):
        """Initializes motionplanner. Subscribe to topics published by
        controller. Sets self as publisher of topics to which controller
        subscribes.
        """
        self._sample_time = rospy.get_param('vel_cmd/sample_time', 0.01)
        self._update_time = rospy.get_param('vel_cmd/update_time', 0.5)
        self._mp_result_topic = rospy.Publisher(
            'mp_result', Trajectory, queue_size=1)
        self._mp_feedback_topic = rospy.Publisher(
            'mp_feedback', Bool, queue_size=1)
        rospy.Subscriber('mp_configure', Settings, self.configure)
        rospy.Subscriber('mp_trigger', Trigger, self.update)

    def configure(self, st):
        """Configures the motionplanner. Creates omgtools Point2point problem
        with room, static and dynamic obstacles.

        Args:
            st : contains data sent over Settings topic.
        """
        print 'configure motionplanner'
        self.safety_margin = rospy.get_param(
            'motionplanner/safety_margin', 0.3)

        self._vehicle = omg.Holonomic(
            shapes=omg.Circle(0.5),
            bounds={'vmax': 0.2, 'vmin': -0.2, 'amax': 0.3, 'amin': -0.3})
        self._vehicle.define_knots(knot_intervals=self.knots)
        self._vehicle.set_options({'safety_distance': self.safety_margin})
        self._robot = omg.Fleet(self._vehicle)
        self._robot.set_initial_conditions([[0., 0.]])
        self._robot.set_terminal_conditions([[0., 0.]])
        # Environment.
        room = {'shape': omg.Rectangle(
            st.room.shape[0], st.room.shape[1]),
                'position': [st.room.position[0], st.room.position[1]]}
        self._obstacles = []
        for k, obst in enumerate(st.obstacles):
            if len(obst.shape) == 1:
                shape = omg.Circle(obst.shape[0])
            elif len(obst.shape) == 2:
                shape = omg.Rectangle(
                    width=obst.shape[0], height=obst.shape[1], orientation=(
                        obst.pose[2]))
            self._obstacles.append(omg.Obstacle(
                {'position': [obst.pose[0], obst.pose[1]]}, shape=shape))

        environment = omg.Environment(room=room)
        environment.add_obstacle(self._obstacles)
        self._robobst = st.robobst

        # Create problem.
        print 'creating problem'
        problem = omg.Point2point(self._robot, environment, freeT=True)
        problem.set_options({'solver_options': {'ipopt': {
            'ipopt.linear_solver': 'ma57',
            'ipopt.print_level': 0}}})
        problem.set_options({
            'hard_term_con': False, 'horizon_time': 10., 'verbose': 1.})
        problem.init()
        self._deployer = omg.Deployer(
            problem, self._sample_time, self._update_time)
        self._deployer.reset()
        self._mp_feedback_topic.publish(True)

    def start(self):
        """Starts the motionplanner by initializing the motionplanner ROS-node.
        """
        rospy.init_node('motionplanner')
        self._goal = Pose2D(x=np.inf, y=np.inf, theta=np.inf)
        print '** Motionplanner Listening **'
        rospy.spin()

    def update(self, cmd):
        """Updates the motionplanner and the deployer solving the Point2point
        problem. Publishes trajectories resulting from calculations.

        Args:
            cmd : contains data sent over Trigger topic.
        """
        # In case goal has changed: set new goal.
        if cmd.goal != self._goal:
            self._goal = cmd.goal
            self._vehicle.set_initial_conditions(
                [cmd.state.x, cmd.state.y])
            self._vehicle.set_terminal_conditions(
                [self._goal.x, self._goal.y])
            self._deployer.reset()
            print 'resetted deployer!'

        state0 = [cmd.state.x, cmd.state.y, cmd.state.theta]
        for l, k in enumerate(self._robobst):
            pos = cmd.obstacles[l].pose[:2]
            vel = cmd.obstacles[l].velocity[:2]
            pos = np.round(pos, 1)
            vel = np.round(vel, 1)
            self._deployer.problem.environment.obstacles[0].set_state(
                    {'position': pos, 'velocity': vel})
        trajectories = self._deployer.update(cmd.current_time, state0)
        self._result = RobotTrajectory(
            v_traj=trajectories['input'][0, :],
            w_traj=trajectories['input'][1, :],
            x_traj=trajectories['state'][0, :],
            y_traj=trajectories['state'][1, :])
        self._mp_result_topic.publish(self._result)


if __name__ == '__main__':
    motionplanner = MotionPlanner()
    motionplanner.start()
