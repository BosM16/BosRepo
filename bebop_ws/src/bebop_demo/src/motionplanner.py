#!/usr/bin/env python

from geometry_msgs.msg import Pose
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
                    'motionplanner/safety_margin', 0.2)

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

        self._vehicle = omg.Holonomic3D(
            shapes=omg.Sphere(self.drone_radius),
            bounds={'vmax': self.vmax, 'vmin': -self.vmax,
                    'amax': self.amax, 'amin': -self.amax})
        self._vehicle.define_knots(knot_intervals=self.knots)
        self._vehicle.set_options({'safety_distance': self.safety_margin})
        self._vehicle.set_initial_conditions([0., 0., 0.])
        self._vehicle.set_terminal_conditions([0., 0., 0.])

        # Environment.
        room_origin_x = rospy.get_param('motionplanner/room_origin_x', 0.)
        room_origin_y = rospy.get_param('motionplanner/room_origin_y', 0.)
        room_origin_z = rospy.get_param('motionplanner/room_origin_z', 0.)
        room_width = rospy.get_param('motionplanner/room_width', 1.)
        room_depth = rospy.get_param('motionplanner/room_depth', 1.)
        room_height = rospy.get_param('motionplanner/room_height', 1.)

        room = {'shape': omg.Cuboid(room_width, room_depth, room_height),
                'position': [room_origin_x, room_origin_y, room_origin_z]}

        for k, obst in enumerate(obstacles.obst_list):
            if len(obst.shape) == 2:
                shape = omg.RegularPrisma(obst.shape[0], obst.shape[1], 6)
            elif len(obst.shape) == 3:
                shape = omg.Cuboid(
                    width=obst.shape[0],
                    depth=obst.shape[1],
                    height=obst.shape[2])
                    # orientation=(obst.pose[2]))
            self._obstacles.append(omg.Obstacle({'position': [
                    obst.pose[0], obst.pose[1], obst.pose[2]]}, shape=shape))

        environment = omg.Environment(room=room)
        environment.add_obstacle(self._obstacles)

        # Create problem.
        print '----------------------------------'
        print 'Motionplanner Creating Problem ...'
        print '----------------------------------'

        problem = omg.Point2point(self._vehicle, environment, freeT=True)
        problem.set_options({'solver_options': {'ipopt': {
            # 'ipopt.linear_solver': 'ma57',
            'ipopt.print_level': 0}}})
        problem.set_options({
            'hard_term_con': False, 'horizon_time': self.horizon_time,
            'verbose': 1.})

        problem.init()
        problem.fullstop = True

        self._deployer = omg.Deployer(
            problem, self._sample_time, self._update_time)
        self._deployer.reset()

        mp_configured = True

        return ConfigMotionplannerResponse(mp_configured)

    def start(self):
        """Starts the motionplanner by initializing the motionplanner ROS-node.
        """
        rospy.init_node('motionplanner')
        self._goal = Pose()
        self._goal.position.x = np.inf
        self._goal.position.y = np.inf
        self._goal.position.z = np.inf

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
        if cmd.goal_pos != self._goal:
            self._goal = cmd.goal_pos
            self._vehicle.set_initial_conditions(
                [cmd.pos_state.position.x,
                 cmd.pos_state.position.y,
                 cmd.pos_state.position.z],
                [cmd.vel_state.x, cmd.vel_state.y, cmd.vel_state.z])
            self._vehicle.set_terminal_conditions(
                [self._goal.position.x,
                 self._goal.position.y,
                 self._goal.position.z])
            self._deployer.reset()
            print '-------------------------------------------'
            print 'New Goal - Motionplanner Resetted Deployer!'
            print '-------------------------------------------'

        state0 = [cmd.pos_state.position.x,
                  cmd.pos_state.position.y,
                  cmd.pos_state.position.z]
        input0 = [cmd.vel_state.x, cmd.vel_state.y, cmd.vel_state.z]

        trajectories = self._deployer.update(cmd.current_time, state0, input0)

        self._result = Trajectories(
            u_traj=trajectories['input'][0, :],
            v_traj=trajectories['input'][1, :],
            w_traj=trajectories['input'][2, :],
            x_traj=trajectories['state'][0, :],
            y_traj=trajectories['state'][1, :],
            z_traj=trajectories['state'][2, :],)
        self._mp_result_topic.publish(self._result)
        # if not (self._deployer.problem.local_problem.problem.stats()
        #         ['return_status'] == 'Solve_Succeeded'):
        #     self._result = Trajectories(
        #         u_traj=trajectories['input'][0, :],
        #         v_traj=trajectories['input'][1, :],
        #         w_traj=trajectories['input'][2, :],
        #         x_traj=trajectories['state'][0, :],
        #         y_traj=trajectories['state'][1, :],
        #         z_traj=trajectories['state'][2, :])
        # else:
        #     print ('send trajectory of zero input commands since problem'
        #            'is infeasible')
        #     self._result = Trajectories(
        #         u_traj=np.zeros((1, 100)),
        #         v_traj=np.zeros((1, 100)),
        #         w_traj=np.zeros((1, 100)),
        #         x_traj=cmd.pos_state.position.x*np.ones((1, 100)),
        #         y_traj=cmd.pos_state.position.y*np.ones((1, 100)),
        #         z_traj=cmd.pos_state.position.z*np.ones((1, 100)))
        # self._mp_result_topic.publish(self._result)


if __name__ == '__main__':
    motionplanner = MotionPlanner()
    motionplanner.start()
