#!/usr/bin/env python

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

from bebop_demo.msg import Trigger, Trajectories, Obstacle

from bebop_demo.srv import (ConfigMotionplanner, ConfigMotionplannerResponse,
                            ConfigMotionplannerRequest)

import numpy as np
import omgtools as omg
import rospy


class MotionPlanner(object):

    def __init__(self):
        """Initializes motionplanner. Subscribe to topics published by
        controller. Sets self as publisher of topics to which controller
        subscribes.
        """
        self._sample_time = rospy.get_param('vel_cmd/sample_time', 0.01)
        self._update_time = rospy.get_param('vel_cmd/update_time', 0.5)
        self.knots = rospy.get_param('motionplanner/knot_intervals', 10)
        self.horizon_time = rospy.get_param('motionplanner/horizon_time', 10.)
        self.vmax = rospy.get_param('motionplanner/vmax', 0.2)
        self.amax = rospy.get_param('motionplanner/amax', 0.3)
        self.drone_radius = rospy.get_param('motionplanner/drone_radius', 0.20)
        self.safety_margin = rospy.get_param(
                    'motionplanner/safety_margin', 0.1)

        self._result = Trajectories()
        self._obstacles = []

        self._mp_result_topic = rospy.Publisher(
            'motionplanner/result', Trajectories, queue_size=1)

        rospy.Subscriber('motionplanner/trigger', Trigger, self.update)

        self.configure = rospy.Service(
            "/motionplanner/config_motionplanner", ConfigMotionplanner,
            self.configure)

    def configure(self, obstacles):
        """Configures the motionplanner. Creates omgtools Point2point problem
        with room, static and dynamic obstacles.

        Args:
            obstacles : contains the obstacles sent over the configure service.
        """
        print '----------------------------'
        print 'Motionplanner configuring...'
        print '----------------------------'
        mp_configured = False

        self._vehicle = omg.Holonomic(
            shapes=omg.Circle(self.drone_radius),
            bounds={'vmax': self.vmax, 'vmin': -self.vmax,
                    'amax': self.amax, 'amin': -self.amax})
        self._vehicle.define_knots(knot_intervals=self.knots)
        self._vehicle.set_options({'safety_distance': self.safety_margin})
        self._vehicle.set_initial_conditions([0., 0.])
        self._vehicle.set_terminal_conditions([0., 0.])

        # Environment.
        room_origin_x = rospy.get_param('motionplanner/room_origin_x', 0.)
        room_origin_y = rospy.get_param('motionplanner/room_origin_y', 0.)
        room_origin_theta = rospy.get_param(
            'motionplanner/room_origin_theta', 0.)
        room_width = rospy.get_param('motionplanner/room_width', 1.)
        room_height = rospy.get_param('motionplanner/room_height', 1.)

        room = {'shape': omg.Rectangle(room_width, room_height),
                'position': [room_origin_x, room_origin_y]}

        for k, obst in enumerate(obstacles.obst_list):
            if len(obst.shape) == 1:
                shape = omg.Circle(obst.shape[0])
            elif len(obst.shape) == 2:
                shape = omg.Rectangle(
                    width=obst.shape[0],
                    height=obst.shape[1],
                    orientation=(obst.pose[2]))
            self._obstacles.append(omg.Obstacle(
                {'position': [obst.pose[0], obst.pose[1]]}, shape=shape))

        environment = omg.Environment(room=room)
        environment.add_obstacle(self._obstacles)

        # Create problem.
        print '----------------------------------'
        print 'Motionplanner Creating Problem ...'
        print '----------------------------------'

        problem = omg.Point2point(self._vehicle, environment, freeT=True)
        problem.set_options({'solver_options': {'ipopt': {
            'ipopt.linear_solver': 'ma57',
            'ipopt.print_level': 0}}})
        problem.set_options({
            'hard_term_con': False, 'horizon_time': self.horizon_time,
            'verbose': 1.})

        problem.init()

        self._deployer = omg.Deployer(
            problem, self._sample_time, self._update_time)
        self._deployer.reset()

        mp_configured = True

        return ConfigMotionplannerResponse(mp_configured)

    def start(self):
        """Starts the motionplanner by initializing the motionplanner ROS-node.
        """
        rospy.init_node('motionplanner')
        self._goal = Pose2D(x=np.inf, y=np.inf, theta=np.inf)

        print '---------------------------'
        print '- Motionplanner Listening -'
        print '---------------------------'

        rospy.spin()

    def update(self, cmd):
        """Updates the motionplanner and the deployer solving the Point2point
        problem. Publishes trajectories resulting from calculations.

        Args:
            cmd : contains data sent over Trigger topic.
        """
        # In case goal has changed: set new goal.
        if cmd.goal != self._goal:
            self._goal = cmd.goal_pos
            self._vehicle.set_initial_conditions(
                [cmd.pos_state.x, cmd.pos_state.y],
                [cmd.vel_state.x, cmd.vel_state.y])
            self._vehicle.set_terminal_conditions(
                [self._goal.x, self._goal.y],
                [cmd.goal_vel.x, cmd.goal_vel.y])
            self._deployer.reset()
            print '-------------------------------------------'
            print 'New Goal - Motionplanner Resetted Deployer!'
            print '-------------------------------------------'

        state0 = [cmd.pos_state.x, cmd.pos_state.y]
        input0 = [cmd.vel_state.x, cmd.vel_state.y]

        trajectories = self._deployer.update(cmd.current_time, state0, input0)
        self._result = Trajectories(
            v_traj=trajectories['input'][0, :],
            w_traj=trajectories['input'][1, :],
            x_traj=trajectories['state'][0, :],
            y_traj=trajectories['state'][1, :])
        self._mp_result_topic.publish(self._result)


if __name__ == '__main__':
    motionplanner = MotionPlanner()
    motionplanner.start()
