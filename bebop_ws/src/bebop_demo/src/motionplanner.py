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

        self.configure = rospy.Service(
            "motionplanner/config_motionplanner", ConfigMotionplanner,
            self.configure)

    def configure(self, data):
        """Configures the motionplanner. Creates omgtools Point2point problem
        with room, static and dynamic obstacles.

        Args:
            data :
                static_obstacles
                dyn_obstacles
                difficult_obst
        """
        mp_configured = False
        self.n_stat_obst = len(data.static_obstacles)
        self.n_dyn_obst = len(data.dyn_obstacles)

        if data.difficult_obst:
            self.omg_update_time = rospy.get_param(
                'controller/omg_update_time_slow', 0.5)
            safety_margin = rospy.get_param(
                'motionplanner/safety_margin_small', 0.1)
            safety_weight = rospy.get_param(
                 'motionplanner/safety_weight_slow', 10.)
            drone_radius = rospy.get_param(
                'motionplanner/drone_radius_small', 0.20)
            vmax = rospy.get_param(
                'motionplanner/omg_vmax_low', 0.2)
            amax = rospy.get_param(
                'motionplanner/omg_amax_low', 0.3)
        else:
            self.omg_update_time = rospy.get_param(
                'controller/omg_update_time', 0.5)
            safety_margin = rospy.get_param(
                'motionplanner/safety_margin', 0.2)
            safety_weight = rospy.get_param(
                 'motionplanner/safety_weight', 10.)
            drone_radius = rospy.get_param(
                'motionplanner/drone_radius', 0.225)
            vmax = rospy.get_param(
                'motionplanner/omg_vmax', 0.2)
            amax = rospy.get_param(
                'motionplanner/omg_amax', 0.3)

        if self.n_dyn_obst:
            safety_margin = rospy.get_param(
                'motionplanner/safety_margin_dyn_obst', 0.2)
            safety_weight = rospy.get_param(
                 'motionplanner/safety_weight_dyn_obst', 10.)

        self._vehicle = omg.Holonomic3D(
            shapes=omg.Sphere(drone_radius),
            bounds={'vmax': vmax, 'vmin': -vmax,
                    'amax': amax, 'amin': -amax})
        self._vehicle.define_knots(knot_intervals=self.knots)
        self._vehicle.set_options({'safety_distance': safety_margin,
                                   'safety_weight': safety_weight,
                                   'syslimit': 'norm_2'})
        self._vehicle.set_initial_conditions([0., 0., 0.])
        self._vehicle.set_terminal_conditions([0., 0., 0.])

        # Environment.
        room_width = rospy.get_param('motionplanner/room_width', 1.)
        room_depth = rospy.get_param('motionplanner/room_depth', 1.)
        room_height = rospy.get_param('motionplanner/room_height', 1.)
        room_origin_x = 0.
        room_origin_y = 0.
        room_origin_z = room_height/2

        room = {'shape': omg.Cuboid(room_width, room_depth, room_height),
                'position': [room_origin_x, room_origin_y, room_origin_z]}

        # Static obstacles.
        for k, obst in enumerate(data.static_obstacles):
            if obst.obst_type.data == "inf cylinder":
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
                    position = [obst.pose[0], 0., obst.pose[2]]

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

        # Dynamic obstacles.
        for obst in data.dyn_obstacles:
            shape = omg.Circle(obst.shape[0])
            position = [obst.pose[0], obst.pose[1]]
            self._obstacles.append(omg.Obstacle(
                                        {'position': position}, shape=shape))

        # Create the environment as room with obstacles.
        environment = omg.Environment(room=room)
        environment.add_obstacle(self._obstacles)

        # Create problem.
        problem = omg.Point2point(self._vehicle, environment, freeT=False)
        problem.set_options({'solver_options': {'ipopt': {
            'ipopt.linear_solver': 'ma57',
            'ipopt.max_iter': 1000,
            'ipopt.print_level': 0,
            'ipopt.tol': 1e-4,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.warm_start_bound_push': 1e-8,
            'ipopt.warm_start_mult_bound_push': 1e-8,
            'ipopt.mu_init': 1e-5,
            'ipopt.hessian_approximation': 'limited-memory'
            }}})

        if self.n_dyn_obst:
            problem.set_options({
                'hard_term_con': False, 'horizon_time': self.horizon_time,
                'verbose': 1.})
        else:
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
        for k in range(self.n_dyn_obst):
            pos = cmd.dyn_obstacles[k].pose
            vel = cmd.dyn_obstacles[k].velocity
            # Dirty fix necessary to make dynamic obstacle work in OMG-tools.
            # -> NIET OKE, gedverdekke Ruben.
            pos = np.round(pos, 1)
            vel = np.round(vel, 1)
            obst_i = k + self.n_stat_obst
            (self._deployer.problem.environment.obstacles[obst_i].set_state(
                                        {'position': pos, 'velocity': vel}))

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


if __name__ == '__main__':
    motionplanner = MotionPlanner()
    motionplanner.start()
