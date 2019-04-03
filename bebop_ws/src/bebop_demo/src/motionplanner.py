#!/usr/bin/env python

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Empty, String

from bebop_demo.msg import Trigger, Trajectories, Obstacle

from bebop_demo.srv import (ConfigMotionplanner, ConfigMotionplannerResponse,
                            ConfigMotionplannerRequest)

import numpy as np
import omgtools as omg
import rospy

from fabulous.color import highlight_red, highlight_yellow, magenta, green


class MotionPlanner(object):

    def __init__(self):
        """Initializes motionplanner. Subscribe to topics published by
        controller. Sets self as publisher of topics to which controller
        subscribes.
        """
        self._sample_time = rospy.get_param(
            'controller/sample_time', 0.01)
        self.knots = rospy.get_param(
            'motionplanner/knot_intervals', 10)
        self.horizon_time = rospy.get_param(
            'motionplanner/horizon_time', 10.)

        self._result = Trajectories()
        self._obstacles = []

        self._mp_result_topic = rospy.Publisher(
            'motionplanner/result', Trajectories, queue_size=1)

        rospy.Subscriber('motionplanner/trigger', Trigger, self.update)
        rospy.Subscriber('motionplanner/interrupt', Empty, self.interrupt)

        self.configure = rospy.Service(
            "motionplanner/config_motionplanner", ConfigMotionplanner,
            self.configure)

    def configure(self, data):
        """Configures the motionplanner. Creates omgtools Point2point problem
        with room, static and dynamic obstacles.

        Args:
            data :
                obst_list
                difficult_obst
        """
        mp_configured = False

        if data.difficult_obst:
            self.omg_update_time = rospy.get_param(
                'controller/omg_update_time_slow', 0.5)
            self.safety_margin = rospy.get_param(
                'motionplanner/safety_margin_small', 0.1)
            self.drone_radius = rospy.get_param(
                'motionplanner/drone_radius_small', 0.20)
            self.vmax = rospy.get_param(
                'motionplanner/vmax_low', 0.2)
            self.amax = rospy.get_param(
                'motionplanner/amax_low', 0.3)
        else:
            self.omg_update_time = rospy.get_param(
                'controller/omg_update_time', 0.5)
            self.safety_margin = rospy.get_param(
                'motionplanner/safety_margin', 0.2)
            self.drone_radius = rospy.get_param(
                'motionplanner/drone_radius', 0.225)
            self.vmax = rospy.get_param(
                'motionplanner/vmax', 0.2)
            self.amax = rospy.get_param(
                'motionplanner/amax', 0.3)

        self._vehicle = omg.Holonomic3D(
            shapes=omg.Sphere(self.drone_radius),
            bounds={'vmax': self.vmax, 'vmin': -self.vmax,
                    'amax': self.amax, 'amin': -self.amax})
        self._vehicle.define_knots(knot_intervals=self.knots)
        self._vehicle.set_options({'safety_distance': self.safety_margin,
                                   'syslimit': 'norm_2'})
        self._vehicle.set_initial_conditions([0., 0., 0.])
        self._vehicle.set_terminal_conditions([0., 0., 0.])

        # Environment.
        room_width = rospy.get_param('motionplanner/room_width', 5.)
        room_depth = rospy.get_param('motionplanner/room_depth', 5.)
        room_height = rospy.get_param('motionplanner/room_height', 3.)
        room_origin_x = 0.
        room_origin_y = 0.
        room_origin_z = room_height/2

        room = {'shape': omg.Cuboid(room_width, room_depth, room_height),
                'position': [room_origin_x, room_origin_y, room_origin_z]}

        for k, obst in enumerate(data.obst_list):
            if obst.obst_type.data == "inf_cylinder":
                # 2D shape is extended infinitely along z.
                shape = omg.Circle(obst.shape[0])
                position = [obst.pose[0], obst.pose[1]]

            elif obst.obst_type.data == "slalom plate":
                shape = omg.Beam(obst.shape[1]-0.1, 0.1, np.pi/2)
                position = [obst.pose[0], obst.pose[1]]

            elif obst.obst_type.data == "hexagon":
                shape = omg.RegularPrisma(obst.shape[0], obst.shape[1], 6)
                position = [obst.pose[0], obst.pose[1], obst.pose[2]]

            elif obst.obst_type.data == "window plate":
                if (k % 4) <= 1:  # Side plates 1 and 2.
                    shape = omg.Beam(obst.shape[1]-0.1, 0.1, np.pi/2)
                    position = [obst.pose[0], obst.pose[1]]
                else:  # Upper and lower plates 3 and 4.
                    shape = omg.Plate(shape2d=omg.Rectangle(
                                                obst.shape[0], obst.shape[1]),
                                      height=obst.shape[2],
                                      orientation=[0., np.pi/2, 0.])
                    position = [obst.pose[0], obst.pose[1], obst.pose[2]]

            elif obst.obst_type.data == "plate":
                shape = omg.Plate(shape2d=omg.Rectangle(
                                            obst.shape[0], obst.shape[1]),
                                  height=obst.shape[2],
                                  orientation=[0., np.pi/2,
                                               obst.direction])
                position = [obst.pose[0], obst.pose[1], obst.pose[2]]

            else:
                print highlight_yellow(' Warning: invalid obstacle type ')

            self._obstacles.append(omg.Obstacle(
                                        {'position': position}, shape=shape))

        environment = omg.Environment(room=room)
        environment.add_obstacle(self._obstacles)

        # Create problem.
        problem = omg.Point2point(self._vehicle, environment, freeT=False)
        problem.set_options({'solver_options': {'ipopt': {
            'ipopt.linear_solver': 'ma57',
            'ipopt.print_level': 0,
            'ipopt.tol': 1e-4,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.warm_start_bound_push': 1e-6,
            'ipopt.warm_start_mult_bound_push': 1e-6,
            'ipopt.mu_init': 1e-5,
            'ipopt.hessian_approximation': 'limited-memory'
            }}})

        problem.set_options({
            'hard_term_con': True, 'horizon_time': self.horizon_time,
            'verbose': 1.})

        problem.init()
        # problem.fullstop = True

        self._deployer = omg.Deployer(
            problem, self._sample_time, self.omg_update_time)
        self._deployer.reset()

        mp_configured = True
        print green('----   Motionplanner running   ----')

        return ConfigMotionplannerResponse(mp_configured)

    def start(self):
        """Starts the motionplanner by initializing the motionplanner ROS-node.
        """
        rospy.init_node('motionplanner')
        self._goal = Pose()
        self._goal.position.x = np.inf
        self._goal.position.y = np.inf
        self._goal.position.z = np.inf

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
                # [cmd.vel_state.x, cmd.vel_state.y, cmd.vel_state.z]
                )
            self._vehicle.set_terminal_conditions(
                [self._goal.position.x,
                 self._goal.position.y,
                 self._goal.position.z])
            self._deployer.reset()
            print magenta('---- Motionplanner received a new goal -'
                          ' deployer resetted ----')

        state0 = [cmd.pos_state.position.x,
                  cmd.pos_state.position.y,
                  cmd.pos_state.position.z]
        input0 = [cmd.vel_state.x, cmd.vel_state.y, cmd.vel_state.z]

        trajectories = self._deployer.update(cmd.current_time, state0)  # input0)

        calc_succeeded = True
        return_status = self._deployer.problem.problem.stats()['return_status']
        if (return_status != 'Solve_Succeeded'):
            print highlight_red(return_status, ' -- brake! ')
            calc_succeeded = False

        self._result = Trajectories(
            u_traj=trajectories['input'][0, :],
            v_traj=trajectories['input'][1, :],
            w_traj=trajectories['input'][2, :],
            x_traj=trajectories['state'][0, :],
            y_traj=trajectories['state'][1, :],
            z_traj=trajectories['state'][2, :],
            success=calc_succeeded)

        self._mp_result_topic.publish(self._result)

    def interrupt(self, empty):
        '''Stop calculations when goal is reached.
        '''
        self._deployer.reset()


if __name__ == '__main__':
    motionplanner = MotionPlanner()
    motionplanner.start()
