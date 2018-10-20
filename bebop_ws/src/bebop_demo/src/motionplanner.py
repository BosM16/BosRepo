#!/usr/bin/env python

# This file is part of OMG-tools.
#
# OMG-tools -- Optimal Motion Generation-tools
# Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
# All rights reserved.
#
# OMG-tools is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

from std_msgs.msg import Bool
import numpy as np
import rospy

from omg_ros_nav_bridge.msg import (
    Trigger, RobotTrajectory, Settings)

import omgtools as omg


class MotionPlanner(object):
    _result = RobotTrajectory

    def __init__(self):
        """Initializes motionplanner. Subscribe to topics published by
        controller. Sets self as publisher of topics to which controller
        subscribes.
        """
        self._mp_result_topic = rospy.Publisher(
            'mp_result', RobotTrajectory, queue_size=1)
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
        self._sample_time = st.sample_time
        self._update_time = st.update_time
        self.safety_margin = rospy.get_param('vel_cmd/safety_margin', 0.3)
        vmax = rospy.get_param('vel_cmd/max_vel', 0.3)
        amax = rospy.get_param('vel_cmd/max_accel', 0.3)
        self._vehicle = omg.Holonomic(
            shapes=omg.Circle(0.5),
            bounds={'vmax': vmax, 'vmin': -vmax, 'amax': amax, 'amin': -amax})
        self._vehicle.define_knots(knot_intervals=10)
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
        self._goal = [np.inf, np.inf]
        print 'listening'
        rospy.spin()

    def update(self, cmd):
        """Updates the motionplanner and the deployer solving the Point2point
        problem. Publishes trajectories resulting from calculations.

        Args:
            cmd : contains data sent over Trigger topic.
        """
        reset = False
        if cmd.goal.pose != self._goal:
            self._goal = cmd.goal.pose[:]
            self._vehicle.set_initial_conditions(cmd.state.pose[:])
            self._vehicle.set_terminal_conditions(cmd.goal.pose[:])
            reset = True
        if reset:
            self._deployer.reset()
            print 'resetted deployer!'
        state0 = [cmd.state.pose[:]]
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
