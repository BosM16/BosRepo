#!/usr/bin/env python

from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from geometry_msgs.msg import (Twist, TwistStamped, Point, PointStamped,
                               Pose, PoseStamped)
from std_msgs.msg import Bool, Empty, String
from visualization_msgs.msg import Marker, MarkerArray
from bebop_demo.msg import Trigger, Trajectories, Obstacle
from vive_localization.msg import PoseMeas

from bebop_demo.srv import GetPoseEst, ConfigMotionplanner

import rospy
import numpy as np
import scipy.io as io
from scipy.signal import butter, filtfilt, lfilter
import tf
import tf2_ros
import tf2_geometry_msgs as tf2_geom

from fabulous.color import (highlight_red, highlight_green, highlight_blue,
                            green, yellow, highlight_yellow)


class Controller(object):

    ############################
    # Initialization functions #
    ############################
    def __init__(self):
        """Initialization of Controller object.
        """
        rospy.init_node("controller")

        self.state = "initialization"
        self.state_dict = {"standby": self.hover,
                           "emergency": self.repeat_safety_brake,
                           "take-off": self.take_off_land,
                           "land": self.take_off_land,
                           "omg standby": self.hover,
                           "omg fly": self.omg_fly,
                           "place hex obstacles": self.place_cyl_hex_obst,
                           "place cyl obstacles": self.place_cyl_hex_obst,
                           "place slalom obstacles": self.place_slalom_obst,
                           "place plate obstacles": self.place_plate_obst,
                           "place window obstacles": self.place_window_obst,
                           "configure motionplanner": self.config_mp,
                           "draw path slow": self.draw_traj,
                           "draw path fast": self.draw_traj,
                           "fly to start": self.fly_to_start,
                           "follow path": self.follow_traj,
                           "undamped spring": self.hover_changed_gains,
                           "viscous fluid": self.hover_changed_gains,
                           "reset PID": self.reset_pid_gains,
                           "drag drone": self.drag_drone,
                           "gamepad flying": self.gamepad_flying,
                           "dodge dyn obst": self.dodge_dyn_obst}

        self._init_params()
        self._init_variables()
        self._marker_setup()

        self._init_vel_model()
        self._init_topics()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def _init_vel_model(self):
        '''Initializes model parameters for conversion of desired velocities to
        angle inputs.Discrete time state space model (Ts=0.01s) of the
        inverted, LPF filtered velocity system.

        '''
        Ax = np.array([
                    [2.924615161772681, -1.426022503893993, 0.927378249329201],
                    [2.0,    0.,    0.],
                    [0.,     0.5,   0.]])
        Ay = np.array([
                    [2.924615161772681, -1.426022503893993, 0.927378249329201],
                    [2.0,    0.,    0.],
                    [0.,     0.5,   0.]])
        Az = np.array([[1.946703849484298, -0.948087691346676],
                       [1.0,    0.]])

        self.A = np.zeros([8, 8])
        self.A[0:3, 0:3] = Ax
        self.A[3:6, 3:6] = Ay
        self.A[6:8, 6:8] = Az

        self.B = np.zeros([8, 3])
        self.B[0, 0] = 0.25
        self.B[3, 1] = 0.25
        self.B[6, 2] = 0.25

        self.C = np.zeros([3, 8])
        self.C[0, 0:3] = [0.093794142767462,
                          -0.091022743092107,
                          0.088262872564127]
        self.C[1, 3:6] = [0.110260524508392,
                          -0.107520541682973,
                          0.104800707877982]
        self.C[2, 6:8] = [0.094457321516314,
                          -0.088810404097729]

        self.D = np.array([[0.011815313012427, 0.0, 0.0],
                           [0.0, 0.013957040852033, 0.0],
                           [0.0, 0.0,  0.011763641499985]])

    def _init_topics(self):
        '''Initializes rostopic Publishers and Subscribers.
        '''
        self.cmd_vel = rospy.Publisher(
            'bebop/cmd_vel', Twist, queue_size=1)
        self.take_off = rospy.Publisher(
            'bebop/takeoff', Empty, queue_size=1)
        self.land = rospy.Publisher(
            'bebop/land', Empty, queue_size=1)
        self._mp_trigger_topic = rospy.Publisher(
            'motionplanner/trigger', Trigger, queue_size=1)
        self.obst_pub = rospy.Publisher(
            'motionplanner/rviz_obst', MarkerArray, queue_size=1)
        self.vhat_vector_pub = rospy.Publisher(
            'motionplanner/vhat_vector', Marker, queue_size=1)
        self.trajectory_desired = rospy.Publisher(
            'motionplanner/desired_path', Marker, queue_size=1)
        self.trajectory_real = rospy.Publisher(
            'motionplanner/real_path', Marker, queue_size=1)
        self.trajectory_drawn = rospy.Publisher(
            'motionplanner/drawn_path', Marker, queue_size=1)
        self.trajectory_smoothed = rospy.Publisher(
            'motionplanner/smoothed_path', Marker, queue_size=1)
        self.current_ff_vel_pub = rospy.Publisher(
            'motionplanner/current_ff_vel', Marker, queue_size=1)
        self.draw_room = rospy.Publisher(
            'motionplanner/room_contours', Marker, queue_size=1)
        self.ctrl_state_finish = rospy.Publisher(
            'controller/state_finish', Empty, queue_size=1)
        self.pos_error_pub = rospy.Publisher(
            'controller/position_error', PointStamped, queue_size=1)

        rospy.Subscriber('motionplanner/result', Trajectories,
                         self.get_mp_result)
        rospy.Subscriber('vive_localization/ready', Empty,
                         self.publish_obst_room)
        rospy.Subscriber('ctrl_keypress/rtrigger', Bool, self.r_trigger)
        rospy.Subscriber('ctrl_keypress/ltrigger', Bool, self.l_trigger)
        rospy.Subscriber('ctrl_keypress/rtrackpad', Bool, self.trackpad_press)
        rospy.Subscriber(
            'vive_localization/c1_pose', PoseStamped, self.get_ctrl_r_pos)
        rospy.Subscriber(
            'vive_localization/c2_pose', PoseStamped, self.get_ctrl_l_pos)
        rospy.Subscriber(
            'vive_localization/c1_velocity', TwistStamped, self.get_ctrl_r_vel)
        rospy.Subscriber(
            'vive_localization/c2_velocity', TwistStamped, self.get_ctrl_l_vel)
        rospy.Subscriber('fsm/state', String, self.switch_state)
        rospy.Subscriber(
            '/bebop/states/ardrone3/PilotingState/FlyingStateChanged',
            Ardrone3PilotingStateFlyingStateChanged, self.bebop_flying_state)
        rospy.Subscriber(
            'vive_localization/pose', PoseMeas, self.new_measurement)

    def _init_params(self):
        '''Initializes (reads and sets) externally configurable parameters
        (rosparams).
        '''
        self.Kp_x = rospy.get_param('controller/Kp_x', 0.6864)
        self.Ki_x = rospy.get_param('controller/Ki_x', 0.6864)
        self.Kd_x = rospy.get_param('controller/Kd_x', 0.6864)
        self.Kp_y = rospy.get_param('controller/Kp_y', 0.6864)
        self.Ki_y = rospy.get_param('controller/Ki_y', 0.6864)
        self.Kd_y = rospy.get_param('controller/Kd_y', 0.6864)
        self.Kp_z = rospy.get_param('controller/Kp_z', 0.5)
        self.Ki_z = rospy.get_param('controller/Ki_z', 1.5792)
        self.K_theta = rospy.get_param('controller/K_theta', 0.3)
        self.max_input = rospy.get_param('controller/max_input', 0.5)
        self.room_width = rospy.get_param('motionplanner/room_width', 1.)
        self.room_depth = rospy.get_param('motionplanner/room_depth', 1.)
        self.room_height = rospy.get_param('motionplanner/room_height', 1.)
        self.drone_radius = rospy.get_param('motionplanner/drone_radius', 0.20)
        self.safety_treshold = rospy.get_param('controller/safety_treshold',
                                               0.5)
        self.pos_nrm_tol = rospy.get_param(
                                       'controller/goal_reached_pos_tol', 0.05)
        self._sample_time = rospy.get_param('controller/sample_time', 0.01)
        self.omg_update_time = rospy.get_param(
            'controller/omg_update_time', 0.5)
        self.rate = rospy.Rate(1./self._sample_time)

        # Setup low pass filter for trajectory drawing task.
        cutoff_freq_LPF = rospy.get_param('controller/LPF_cutoff', 0.5)
        LPF_order = rospy.get_param('controller/LPF_order', 4)

        norm_fc_LPF = cutoff_freq_LPF/(0.5)*self._sample_time
        self.butter_b, self.butter_a = butter(
            LPF_order, norm_fc_LPF, btype='low', analog=False)

    def _init_variables(self):
        '''Initializes variables that are used later on.
        '''
        # State related variables
        self.airborne = False
        self.calc_succeeded = False
        self.target_reached = False
        self.startup = False
        self.state_changed = False
        self.executing_state = False
        self.state_killed = False
        self.trackpad_held = False
        self.r_trigger_held = False
        self.overtime = False

        # Measurement related variables
        self.meas_pos_x = 0.
        self.meas_pos_y = 0.
        self.meas_pos_z = 0.
        self.meas_time = 0.

        # Other
        self._traj = {'u': [0.0], 'v': [0.0], 'w': [0.0],
                      'x': [0.0], 'y': [0.0], 'z': [0.0]}
        self._traj_strg = {'u': [0.0], 'v': [0.0], 'w': [0.0],
                           'x': [0.0], 'y': [0.0], 'z': [0.0]}
        self.X = np.array(
                    [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
        self.desired_yaw = np.pi/2.
        self.real_yaw = 0.0
        self.pos_nrm = np.inf
        self.fb_cmd_prev = Twist()
        self.pos_error_prev = PointStamped()
        self.vel_error_prev = PointStamped()
        self.measurement_valid = False
        self.static_obst = []
        self.dynamic_obst = []
        self.omg_index = 1
        self.difficult_obst = False
        self._goal = Pose()
        self.hover_setpoint = Pose()
        self.ctrl_r_pos = Pose()
        self.ctrl_r_vel = Twist()
        self.ctrl_l_pos = Pose()
        self.ctrl_l_vel = Twist()
        self.draw = False
        self.drag = False

        self.full_cmd = TwistStamped()
        self.full_cmd.header.frame_id = "world_rot"
        self.full_cmd.header.stamp = rospy.Time.now()
        self.ff_velocity = TwistStamped()
        self.ff_cmd = Twist()
        self.drone_vel_est = Point()
        self.drone_pose_est = Pose()

    ##################
    # Main functions #
    ##################

    def start(self):
        '''Configures,
        Starts the controller's periodical loop.
        '''
        self.draw_room_contours()
        self.config_mp()
        print green('----    Controller running     ----')

        while not rospy.is_shutdown():
            if self.state_changed:
                self.state_changed = False
                print yellow(' Controller state changed to: ', self.state)
                self.executing_state = True
                # Execute state function.
                self.state_dict[self.state]()
                self.executing_state = False

                # State has not finished if it has been killed!
                if not self.state_killed:
                    self.ctrl_state_finish.publish(Empty())
                    print yellow('------------------------')
                self.state_killed = False

                # Adjust goal to make sure hover uses PID actions to stay in
                # current place.
                self.full_cmd.header.stamp = rospy.Time.now()
                (self.drone_pose_est, self.drone_vel_est,
                 self.real_yaw, measurement_valid) = self.get_pose_est()
                self.hover_setpoint.position = self.drone_pose_est.position

            if not self.state == "initialization":
                self.hover()
            self.rate.sleep()

    def config_mp(self):
        '''Configures the motionplanner over ConfigMotionplanner Service
        by loading in the room and static obstacles. Waits for Motionplanner to
        set mp_status to configured.
        '''
        if (self.static_obst and
                (self.static_obst[0].obst_type.data == "window plate" or
                 (self.static_obst[0].obst_type.data == "slalom plate" and
                 len(self.static_obst) >= 3))):
            self.difficult_obst = True
        else:
            self.difficult_obst = False

        rospy.wait_for_service("/motionplanner/config_motionplanner")
        config_success = False
        try:
            config_mp_resp = rospy.ServiceProxy(
                "/motionplanner/config_motionplanner", ConfigMotionplanner)
            config_success = config_mp_resp(
                                          static_obstacles=self.static_obst,
                                          dyn_obstacles=self.dynamic_obst,
                                          difficult_obst=self.difficult_obst)
        except rospy.ServiceException, e:
            print highlight_red('Service call failed: %s') % e
            config_success = False

        rospy.Subscriber('motionplanner/goal', Pose, self.set_omg_goal)

        return config_success

    def set_omg_goal(self, goal):
        '''Sets the goal and fires motionplanner.
        Args:
            goal: Pose
        '''
        if self.state == "omg_fly":
            self.safety_brake()

        self.target_reached = False

        self._time = 0.
        self._new_trajectories = False
        self.overtime_counter = 0

        self.full_cmd.header.stamp = rospy.Time.now()
        (self.drone_pose_est, self.drone_vel_est,
         self.real_yaw, measurement_valid) = self.get_pose_est()

        if not self.state == "fly to start":
            self.reset_markers()

        self._goal = goal
        # Used to store calculation time of motionplanner.
        # self.calc_time = {}
        # self.calc_time['time'] = []
        self.fire_motionplanner()

        self._init = True
        self.startup = True

    def fire_motionplanner(self):
        '''Publishes inputs to motionplanner via Trigger topic.
        '''
        trigger = Trigger()
        trigger.goal_pos = self._goal
        trigger.goal_vel = Point()
        trigger.pos_state = self.drone_pose_est
        trigger.vel_state = self.drone_vel_est
        trigger.dyn_obstacles = self.dynamic_obst
        trigger.current_time = self._time

        # self.fire_time = rospy.get_rostime()

        self._mp_trigger_topic.publish(trigger)

    def get_mp_result(self, data):
        '''Store results of motionplanner calculations.

        Args:
            data : calculated trajectories received from 'mp_result' topic,
                   published by motionplanner.
        '''
        self.store_trajectories(data.u_traj, data.v_traj, data.w_traj,
                                data.x_traj, data.y_traj, data.z_traj,
                                data.success)
        # self.receive_time = rospy.get_rostime()
        # self.calc_time['time'].append(
        #                         (self.receive_time - self.fire_time).to_sec())

    def omg_update(self):
        '''
        - Updates the controller with newly calculated trajectories and
        velocity commands.
        - Sends out new velocity command.
        - Retrieves new pose estimate.
        '''
        # Send velocity sample.
        self.full_cmd.header.stamp = rospy.Time.now()
        self.cmd_vel.publish(self.full_cmd.twist)

        # Retrieve new pose estimate from World Model.
        # This is a pose estimate for the first following time instance [k+1]
        # if the velocity command sent above corresponds to time instance [k].
        (self.drone_pose_est, self.drone_vel_est, self.real_yaw,
            measurement_valid) = self.get_pose_est()
        self.publish_vhat_vector(self.drone_pose_est.position,
                                 self.drone_vel_est)

        # Publish pose to plot in rviz.
        self.publish_real(self.drone_pose_est.position.x,
                          self.drone_pose_est.position.y,
                          self.drone_pose_est.position.z)

        if not measurement_valid:
            self.safety_brake()
            return
        # Check for new trajectories. While calculating, hover.
        # Trigger Motionplanner or raise 'overtime'
        if self._init:
            if not self._new_trajectories:
                self.hover()
                return
            # Trick to make sure that new trajectories are loaded in next
            # lines of code.
            self.omg_index = int(self.omg_update_time/self._sample_time)
            self._init = False

        if (self.omg_index >= len(self._traj['u'])-2):
            if self._new_trajectories:
                    # Load fresh trajectories.
                    self.load_trajectories()
                    self._new_trajectories = False
                    self._time += self.omg_index*self._sample_time
                    self.pos_index = self.omg_index
                    self.omg_index = 1

                    # Trigger motion planner.
                    self.fire_motionplanner()

                    # Wait for new set of trajectories when calculation
                    # has failed.
                    if not self.calc_succeeded:
                        self._init = True
                        self.safety_brake()
                        return
            else:
                self.safety_brake()
                self.omg_index += 1
                return

        if (self.omg_index >= int(self.omg_update_time/self._sample_time)):
            if self._new_trajectories:
                # Load fresh trajectories.
                self.load_trajectories()
                self._new_trajectories = False
                self._time += self.omg_index*self._sample_time
                self.pos_index = self.omg_index
                self.omg_index = 1

                # Trigger motion planner.
                self.fire_motionplanner()

                # Wait for new set of trajectories when calculation has failed.
                if not self.calc_succeeded:
                    self._init = True
                    self.safety_brake()
                    return
                # Raise overtime counter when calculations were not ready
                # in time.
                if self.overtime:
                    self.overtime_counter += 1
                    self.overtime = False

            else:
                self.overtime = True
                if self.overtime_counter > 3:
                    self.hover()
                    print highlight_yellow('---- WARNING - OVERTIME  ----')

        # publish current pose and velocity calculated by omg-tools
        pos = PointStamped()
        pos.header.frame_id = "world"
        pos.point = Point(x=self._traj['x'][self.omg_index + 1],
                          y=self._traj['y'][self.omg_index + 1],
                          z=self._traj['z'][self.omg_index + 1])
        vel = TwistStamped()
        vel.header.frame_id = "world"
        vel.twist.linear.x = self._traj['u'][self.omg_index + 1]
        vel.twist.linear.y = self._traj['v'][self.omg_index + 1]
        vel.twist.linear.z = self._traj['w'][self.omg_index + 1]

        self.publish_current_ff_vel(pos, vel)

        # Calculate the desired yaw angle based on the pointing direction of
        # the resulting feedforward velocity vector.
        # self.desired_yaw = np.arctan2(vel.point.y, vel.point.x)

        # Transform feedforward command from frame world to world_rotated.
        self.rotate_vel_cmd(vel)

        # Convert feedforward velocity command to angle input.
        self.convert_vel_cmd()

        # Combine feedback and feedforward commands.
        self.combine_ff_fb(pos, vel)

        self.omg_index += 1
        self.hover_setpoint = self.drone_pose_est

    def draw_update(self, index):
        '''
        - Updates the controller with newly calculated trajectories and
        velocity commands.
        - Sends out new velocity command.
        - Retrieves new pose estimate.
        '''
        # Send velocity sample.
        self.full_cmd.header.stamp = rospy.Time.now()
        self.cmd_vel.publish(self.full_cmd.twist)

        # Retrieve new pose estimate from World Model.
        # This is a pose estimate for the first following time instance [k+1]
        # if the velocity command sent above corresponds to time instance [k].
        (self.drone_pose_est, self.drone_vel_est, self.real_yaw,
            measurement_valid) = self.get_pose_est()
        self.publish_vhat_vector(self.drone_pose_est.position,
                                 self.drone_vel_est)

        # Publish pose to plot in rviz.
        self.publish_real(self.drone_pose_est.position.x,
                          self.drone_pose_est.position.y,
                          self.drone_pose_est.position.z)

        if not measurement_valid:
            self.safety_brake()
            return

        # publish current pose and velocity calculated by omg-tools
        pos = PointStamped()
        pos.header.frame_id = "world"
        pos.point = Point(x=self.drawn_pos_x[index],
                          y=self.drawn_pos_y[index],
                          z=self.drawn_pos_z[index])
        vel = TwistStamped()
        vel.header.frame_id = "world"
        vel.twist.linear.x = self.drawn_vel_filt_x[index]
        vel.twist.linear.y = self.drawn_vel_filt_y[index]
        vel.twist.linear.z = self.drawn_vel_filt_z[index]

        self.publish_current_ff_vel(pos, vel)

        # Calculate the desired yaw angle based on the pointing direction of
        # the resulting feedforward velocity vector.
        # self.desired_yaw = np.arctan2(vel.point.y, vel.point.x)

        # Transform feedforward command from frame world to world_rotated.
        self.rotate_vel_cmd(vel)

        # Convert feedforward velocity command to angle input.
        self.convert_vel_cmd()

        # Combine feedback and feedforward commands.
        vel = TwistStamped()
        vel.header.frame_id = "world"
        vel.twist.linear.x = self.drawn_vel_x[index]
        vel.twist.linear.y = self.drawn_vel_y[index]
        vel.twist.linear.z = self.drawn_vel_z[index]
        self.combine_ff_fb(pos, vel)

    ###################
    # State functions #
    ###################

    def switch_state(self, state):
        '''Switches state according to the general fsm state as received by
        bebop_core.
        '''
        if not (state.data == self.state):
            self.state = state.data
            self.state_changed = True
            # If new state received before old one is finished,
            # kill current state.
            if self.executing_state:
                self.state_killed = True
        else:
            print yellow(' Controller already in the correct state!')

        # When going to standby, remove markers in Rviz from previous task.
        # GEBEURT TOCH NOOIT?
        if state.data == "standby":
            self.reset_markers()

    def hover(self, vel_desired=Twist()):
        '''Drone keeps itself in same location through a PID controller.
        '''
        if self.airborne:
            (self.drone_pose_est, self.drone_vel_est, self.real_yaw,
                measurement_valid) = self.get_pose_est()

            if not measurement_valid:
                self.safety_brake()
                return
            pos_desired = PointStamped()
            pos_desired.point = Point(x=self.hover_setpoint.position.x,
                                      y=self.hover_setpoint.position.y,
                                      z=self.hover_setpoint.position.z)

            fb_cmd = self.feedbeck(pos_desired, vel_desired)

            self.full_cmd.twist = fb_cmd
            self.full_cmd.header.stamp = rospy.Time.now()
            self.cmd_vel.publish(self.full_cmd.twist)

    def take_off_land(self):
        '''Function needed to wait when taking of or landing to make sure no
        control inputs are sent out.
        '''
        self.cmd_vel.publish(Twist())
        self.full_cmd.twist = Twist()

        if self.state == "take-off" and not self.airborne:
            counter = 0
            self.take_off.publish(Empty())
            while not (self.airborne or (counter > 50) or (
                    rospy.is_shutdown() or self.state_killed)):
                counter += 1
                rospy.sleep(0.1)

        elif self.state == "land" and self.airborne:
            self.reset_pid_gains()
            rospy.sleep(0.1)
            self.land.publish(Empty())
            while self.airborne and (
                    not (rospy.is_shutdown() or self.state_killed)):
                rospy.sleep(0.1)

    def place_cyl_hex_obst(self):
        '''The user places either infinitely long cylindrical obstacles or
        finite hexagonal obstacles using the left controller. Dragging the Vive
        controller determines the radius of the obstacle. Obstacles are saved
        and drawn in rviz.
        '''
        self.static_obst = []
        self.publish_obst_room(Empty)

        print highlight_green(' Drag left controller to place obstacle ')
        while not (rospy.is_shutdown() or self.state_killed):
            if self.state_changed:
                self.state_changed = False
                break
            if self.draw:
                self.static_obst.append(None)
                center = Point(x=self.ctrl_l_pos.position.x,
                               y=self.ctrl_l_pos.position.y,
                               z=self.ctrl_l_pos.position.z/2.)
                while self.draw:
                    edge = Point(x=self.ctrl_l_pos.position.x,
                                 y=self.ctrl_l_pos.position.y,
                                 z=center.z)
                    radius = self.position_diff_norm(edge, center)
                    if self.state == "place cyl obstacles":
                        Sjaaakie = Obstacle(obst_type=String(
                                            data="inf cylinder"),
                                            shape=[radius],
                                            pose=[center.x, center.y])
                    else:
                        Sjaaakie = Obstacle(obst_type=String(
                                        data="hexagon"),
                                        shape=[radius, 2.*center.z],
                                        pose=[center.x, center.y, center.z])
                    self.static_obst[-1] = Sjaaakie
                    self.publish_obst_room(Empty)
                    self.rate.sleep()
                print highlight_blue(' Obstacle added ')
            self.rate.sleep()

    def place_slalom_obst(self):
        '''The user places slalom obstacles (plate) with the left controller.
        Dragging the Vive controller determines the orientation of the
        obstacle.
        Obstacles are saved and drawn in rviz.
        '''
        self.static_obst = []
        self.publish_obst_room(Empty)

        print highlight_green(
            ' Drag left controller to place and orient obstacle ')

        while not (rospy.is_shutdown() or self.state_killed):
            if self.state_changed:
                self.state_changed = False
                break
            height = self.room_height
            if self.draw:
                self.static_obst.append(None)
                edge = Point(x=self.ctrl_l_pos.position.x,
                             y=self.ctrl_l_pos.position.y,
                             z=height/2.)
                while self.draw:
                    d = self.ctrl_l_pos.position.y
                    self.rate.sleep()
                if d > edge.y:
                    d = 1  # up
                else:
                    d = -1  # down

                edge.y -= d*0.15
                width = self.room_width/2 - d*edge.y
                thickness = 0.15
                center = Point(x=edge.x,
                               y=edge.y+d*(width - thickness)/2.,
                               z=height/2)
                Sjaaakie = Obstacle(obst_type=String(data="slalom plate"),
                                    shape=[height, width, thickness],
                                    pose=[center.x, center.y, center.z],
                                    direction=d,
                                    edge=[edge.x, edge.y, edge.z])

                self.static_obst[-1] = Sjaaakie
                self.publish_obst_room(Empty)
                print highlight_blue(' Obstacle added ')
            self.rate.sleep()

    def place_plate_obst(self):
        '''The user places plate obstacles with the left controller. Pressing
        the Vive controller button defines the upper corner, while dragging the
        controller determines the orientation and other corners of the plate.
        Obstacles are saved and drawn in rviz.
        '''
        self.static_obst = []
        self.publish_obst_room(Empty)

        print highlight_green(
            ' Drag left controller to place and orient obstacle ')

        while not (rospy.is_shutdown() or self.state_killed):
            if self.state_changed:
                self.state_changed = False
                break

            if self.draw:
                self.static_obst.append(None)
                upper_corner = Point(x=self.ctrl_l_pos.position.x,
                                     y=self.ctrl_l_pos.position.y,
                                     z=self.ctrl_l_pos.position.z)
                while self.draw:
                    lower_corner = Point(x=self.ctrl_l_pos.position.x,
                                         y=self.ctrl_l_pos.position.y,
                                         z=self.ctrl_l_pos.position.z)
                    center = Point(x=(upper_corner.x + lower_corner.x)/2.,
                                   y=(upper_corner.y + lower_corner.y)/2.,
                                   z=(upper_corner.z + lower_corner.z)/2.)
                    thickness = 0.1
                    delta_x = upper_corner.x - lower_corner.x
                    delta_y = upper_corner.y - lower_corner.y

                    # Trick to avoid error when delta_x == 0.
                    if delta_x == 0:
                        delta_x = 0.01
                    orientation = np.arctan2((delta_y),
                                             (delta_x)) + np.pi/2.
                    width = np.sqrt(delta_x**2 + delta_y**2)
                    height = upper_corner.z - lower_corner.z
                    Sjaaakie = Obstacle(obst_type=String(data="plate"),
                                        shape=[height, width, thickness],
                                        pose=[center.x, center.y, center.z],
                                        direction=orientation)

                    self.static_obst[-1] = Sjaaakie
                    self.publish_obst_room(Empty)
                    rospy.sleep(0.05)
                print highlight_blue(' Obstacle added ')
            self.rate.sleep()

    def place_window_obst(self):
        '''Place a rectangular window obstacle by defining the center and the
        upper right corner. Windows can only be placed perpendicular to the
        x-direction.
        '''
        self.static_obst = []
        self.publish_obst_room(Empty)

        print highlight_green(
            ' Drag left controller to place obstacle ')

        while not (rospy.is_shutdown() or self.state_killed):
            if self.state_changed:
                self.state_changed = False
                break

            if self.draw:
                for _ in range(0, 4):
                    self.static_obst.append(None)
                corner1 = Point(x=self.ctrl_l_pos.position.x,
                                y=self.ctrl_l_pos.position.y,
                                z=self.ctrl_l_pos.position.z)
                while self.draw:
                    corner2 = Point(x=corner1.x,
                                    y=self.ctrl_l_pos.position.y,
                                    z=self.ctrl_l_pos.position.z)
                    thickness = 0.1

                    center = Point(x=corner1.x,
                                   y=(corner1.y+corner2.y)/2.,
                                   z=(corner1.z+corner2.z)/2.)

                    window_width = abs(corner1.y - corner2.y)
                    window_height = abs(corner1.z - corner2.z)

                    # left
                    w_p1 = self.room_depth/2. + center.y - window_width/2.
                    h_p1 = self.room_height
                    x_p1 = center.x
                    y_p1 = -(self.room_depth - w_p1)/2.
                    z_p1 = self.room_height/2.
                    plate1 = Obstacle(obst_type=String(data="window plate"),
                                      shape=[h_p1, w_p1, thickness],
                                      pose=[x_p1, y_p1, z_p1])
                    # right
                    w_p2 = self.room_depth/2. - (center.y + window_width/2.)
                    h_p2 = self.room_height
                    x_p2 = center.x
                    y_p2 = (self.room_depth - w_p2)/2.
                    z_p2 = self.room_height/2.
                    plate2 = Obstacle(obst_type=String(data="window plate"),
                                      shape=[h_p2, w_p2, thickness],
                                      pose=[x_p2, y_p2, z_p2])
                    # up
                    w_p3 = self.room_depth
                    h_p3 = self.room_height - (center.z + window_height/2.)
                    x_p3 = center.x
                    y_p3 = center.y
                    z_p3 = self.room_height - h_p3/2.
                    plate3 = Obstacle(obst_type=String(data="window plate"),
                                      shape=[h_p3, w_p3, thickness],
                                      pose=[x_p3, y_p3, z_p3])
                    # down
                    w_p4 = self.room_depth
                    h_p4 = center.z - window_height/2.
                    x_p4 = center.x
                    y_p4 = center.y
                    z_p4 = h_p4/2.
                    plate4 = Obstacle(obst_type=String(data="window plate"),
                                      shape=[h_p4, w_p4, thickness],
                                      pose=[x_p4, y_p4, z_p4])

                    self.static_obst[-4] = plate1
                    self.static_obst[-3] = plate2
                    self.static_obst[-2] = plate3
                    self.static_obst[-1] = plate4
                    self.publish_obst_room(Empty)
                    rospy.sleep(0.05)
                print highlight_blue(' Obstacle added ')
            self.rate.sleep()

    def omg_fly(self):
        '''Fly from start to end point using omg-tools as a motionplanner.
        '''
        self.omg_index = 1
        self.set_omg_update_time()
        self.set_ff_pid_gains()
        # Reset ff model
        self.X = np.array(
                    [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

        while not (self.target_reached or (
                rospy.is_shutdown() or self.state_killed)):

            if self.startup:  # Becomes True when goal is set.
                self.omg_update()
                # Determine whether goal has been reached.
                self.check_goal_reached()
            self.rate.sleep()
        if self.state == "follow path":
            self.hover_setpoint.position = Point(x=self.drawn_pos_x[0],
                                                 y=self.drawn_pos_y[0],
                                                 z=self.drawn_pos_z[0])
        else:
            self.hover_setpoint = self.drone_pose_est
        self.reset_pid_gains()
        self.startup = False

    def draw_traj(self):
        '''Start building a trajectory according to the trajectory of the
        controller.
        '''
        self.reset_markers()
        print highlight_green('---- Start drawing path with left Vive'
                              ' controller while holding trigger ----')
        self.stop_drawing = False
        if self.state == "draw path fast":
            self.max_vel = rospy.get_param('motionplanner/draw_vmax_fast', 0.5)
        else:
            self.max_vel = rospy.get_param('motionplanner/draw_vmax_low', 0.5)

        while not (self.stop_drawing or (
                rospy.is_shutdown() or self.state_killed)):
            if self.draw:
                # Erase previous markers in Rviz.
                self.reset_markers()

                while self.draw and not rospy.is_shutdown():
                    self.rate.sleep()

                print yellow('---- Trigger button has been released,'
                             'path will be calculated ----')

                # Clip positions to make sure path does not lie outside room.
                self.drawn_pos_x = [
                            max(- (self.room_width/2. - self.drone_radius),
                                min((self.room_width/2. - self.drone_radius),
                                (elem))) for elem in self.drawn_pos_x]
                self.drawn_pos_y = [
                            max(- (self.room_depth/2. - self.drone_radius),
                                min((self.room_depth/2. - self.drone_radius),
                                (elem))) for elem in self.drawn_pos_y]
                self.drawn_pos_z = [
                            max((self.drone_radius * 2.5),
                                min((self.room_height - self.drone_radius),
                                (elem))) for elem in self.drawn_pos_z]

                # Add padding to path for filtering purposes
                padding = 10
                self.drawn_pos_x = (
                                [self.drawn_pos_x[0] for i in range(padding)]
                                + self.drawn_pos_x +
                                [self.drawn_pos_x[-1] for i in range(padding)])
                self.drawn_pos_y = (
                                [self.drawn_pos_y[0] for i in range(padding)]
                                + self.drawn_pos_y +
                                [self.drawn_pos_y[-1] for i in range(padding)])
                self.drawn_pos_z = (
                                [self.drawn_pos_z[0] for i in range(padding)]
                                + self.drawn_pos_z +
                                [self.drawn_pos_z[-1] for i in range(padding)])

                # Process the drawn trajectory so the drone is able to follow
                # this path.
                if len(self.drawn_pos_x) > (50 + padding*2):
                    self.diff_interp_traj()
                    self.low_pass_filter_drawn_traj()
                    self.differentiate_traj()
                    (self.drawn_vel_filt_x,
                     self.drawn_vel_filt_y,
                     self.drawn_vel_filt_z) = (
                                            self.pad_lpf(self.drawn_vel_x[:],
                                                         self.drawn_vel_y[:],
                                                         self.drawn_vel_z[:]))
                    padding = 50
                    self.drawn_pos_x, self.drawn_pos_y, self.drawn_pos_z = (
                                            self.pad_in_front(self.drawn_pos_x,
                                                              self.drawn_pos_y,
                                                              self.drawn_pos_z,
                                                              padding))
                    self.drawn_vel_x, self.drawn_vel_y, self.drawn_vel_z = (
                                            self.pad_in_front(self.drawn_vel_x,
                                                              self.drawn_vel_y,
                                                              self.drawn_vel_z,
                                                              padding, True))
                else:
                    print highlight_red(
                                    ' Path too short, draw a longer path! ')

            rospy.sleep(0.1)

    def fly_to_start(self):
        '''Sets goal equal to starting position of trajectory and triggers
        omgtools to fly the drone towards it.
        '''
        # If no path drawn, do nothing.
        if not len(self.drawn_pos_x):
            return

        goal = Pose()
        goal.position.x = self.drawn_pos_x[0]
        goal.position.y = self.drawn_pos_y[0]
        goal.position.z = self.drawn_pos_z[0]
        self.set_omg_goal(goal)
        self.omg_fly()

    def follow_traj(self):
        '''Lets the drone fly along the drawn path.
        '''
        # If no path drawn, do nothing.
        if not len(self.drawn_pos_x):
            return

        # Reset omg path markers in Rviz.
        self._desired_path.points = []
        self.trajectory_desired.publish(self._desired_path)
        self._real_path.points = []
        self.trajectory_real.publish(self._real_path)
        self.current_ff_vel.points = [Point(), Point()]
        self.current_ff_vel_pub.publish(self.current_ff_vel)
        self.vhat_vector.points = [Point(), Point()]
        self.vhat_vector_pub.publish(self.vhat_vector)

        self.set_ff_pid_gains()
        # Reset ff model
        self.X = np.array(
                    [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

        # Preparing hover setpoint for when trajectory is completed.
        self._goal = Pose()
        self._goal.position.x = self.drawn_pos_x[-1]
        self._goal.position.y = self.drawn_pos_y[-1]
        self._goal.position.z = self.drawn_pos_z[-1]

        self.hover_setpoint = self._goal
        self.target_reached = False

        self.full_cmd.twist = Twist()
        self.full_cmd.twist.linear.x = self.drawn_vel_x[0]
        self.full_cmd.twist.linear.y = self.drawn_vel_y[0]
        self.full_cmd.twist.linear.z = self.drawn_vel_z[0]

        index = 1
        while (not self.target_reached and (index < len(self.drawn_vel_filt_x))
               and (not rospy.is_shutdown())):
            if self.state_killed:
                break

            self.draw_update(index)
            index += 1
            # Determine whether goal has been reached.
            if ((len(self.drawn_vel_filt_x) - index) < 100):
                self.check_goal_reached()

            self.rate.sleep()
        self.reset_pid_gains()

    def drag_drone(self):
        '''Adapts hover setpoint to follow vive right controller when trigger
        is pressed.
        '''
        self.set_ff_pid_gains()
        self.drag_velocity = Twist()
        while not (rospy.is_shutdown() or self.state_killed):
            drag_offset = Point(
                x=(self.drone_pose_est.position.x-self.ctrl_l_pos.position.x),
                y=(self.drone_pose_est.position.y-self.ctrl_l_pos.position.y),
                z=(self.drone_pose_est.position.z-self.ctrl_l_pos.position.z))

            while (self.drag and not (
                                    self.state_killed or rospy.is_shutdown())):
                # When trigger pulled, freeze offset controller-drone and adapt
                # hover position and velocity setpoint, until trigger is
                # released.

                # Position setpoint
                self.hover_setpoint.position = Point(
                    x=max(- (self.room_width/2. - self.drone_radius),
                          min((self.room_width/2. - self.drone_radius),
                              (self.ctrl_l_pos.position.x + drag_offset.x))),
                    y=max(- (self.room_depth/2. - self.drone_radius),
                          min((self.room_depth/2. - self.drone_radius),
                              (self.ctrl_l_pos.position.y + drag_offset.y))),
                    z=max(self.drone_radius * 3,
                          min(self.room_height - self.drone_radius,
                              (self.ctrl_l_pos.position.z + drag_offset.z))))
                # Velocity setpoint
                self.drag_velocity.linear.x = (self.ctrl_l_vel.linear.x*(
                    abs(self.ctrl_l_pos.position.x + drag_offset.x) < (
                        self.room_width/2. - self.drone_radius)))
                self.drag_velocity.linear.y = (self.ctrl_l_vel.linear.y*(
                    abs(self.ctrl_l_pos.position.y + drag_offset.y) < (
                        self.room_depth/2. - self.drone_radius)))
                self.drag_velocity.linear.z = (self.ctrl_l_vel.linear.z*(
                    abs(self.ctrl_l_pos.position.z + drag_offset.z) < (
                        self.room_height - self.drone_radius)))

                self.hover(self.drag_velocity)
                self.rate.sleep()

            if self.state_changed:
                self.state_changed = False
                break

            self.hover()
            self.rate.sleep()
        self.reset_pid_gains()

    def hover_changed_gains(self):
        '''Adapts gains for the undamped spring (only Kp) or viscous fluid
        (only Kd) illustration.
        '''
        self.hover_setpoint.position.z = rospy.get_param(
            'controller/standard_height', 1.5)

        if self.state == "undamped spring":
            self.Kp_x = self.Kp_x/4.
            self.Ki_x = 0.
            self.Kd_x = 0.
            self.Kp_y = self.Kp_y/4.
            self.Ki_y = 0.
            self.Kd_y = 0.
            self.Kp_z = self.Kp_z/8.
            self.Ki_z = 0.

        elif self.state == "viscous fluid":
            self.Kp_x = 0.
            self.Ki_x = 0.
            self.Kd_x = self.Kd_x
            self.Kp_y = 0.
            self.Ki_y = 0.
            self.Kd_y = self.Kd_y
            self.Kp_z = self.Kp_z/8.
            self.Ki_z = 0.

        while not (self.state_changed or
                   rospy.is_shutdown() or self.state_killed):
            self.hover()
            self.rate.sleep()

    def set_omg_update_time(self):
        '''Adapts the MPC update rate to the difficulty of the obstacles and
        corresponding computation time.
        '''
        if self.difficult_obst:
            self.omg_update_time = rospy.get_param(
                                        'controller/omg_update_time_slow', 0.5)
            self.pos_nrm_tol = rospy.get_param(
                                   'controller/goal_reached_pos_tol_slow', 0.2)
        else:
            self.omg_update_time = rospy.get_param(
                                             'controller/omg_update_time', 0.5)
            self.pos_nrm_tol = rospy.get_param(
                                       'controller/goal_reached_pos_tol', 0.05)

    def set_ff_pid_gains(self):
        '''Sets pid gains to a lower setting for combination with feedforward
        flight to keep the controller stable.
        '''
        if self.state in {"omg fly", "fly to start"} and self.difficult_obst:
            self.Kp_x = rospy.get_param('controller/Kp_omg_low_x', 0.6864)
            self.Ki_x = rospy.get_param('controller/Ki_omg_low_x', 0.6864)
            self.Kd_x = rospy.get_param('controller/Kd_omg_low_x', 0.6864)
            self.Kp_y = rospy.get_param('controller/Kp_omg_low_y', 0.6864)
            self.Ki_y = rospy.get_param('controller/Ki_omg_low_y', 0.6864)
            self.Kd_y = rospy.get_param('controller/Kd_omg_low_y', 0.6864)
            self.Kp_z = rospy.get_param('controller/Kp_omg_low_z', 0.5)
            self.Ki_z = rospy.get_param('controller/Ki_omg_low_z', 1.5792)

        elif self.state in {"omg fly", "fly to start", "dodge dyn obst"}:
            self.Kp_x = rospy.get_param('controller/Kp_omg_x', 0.6864)
            self.Ki_x = rospy.get_param('controller/Ki_omg_x', 0.6864)
            self.Kd_x = rospy.get_param('controller/Kd_omg_x', 0.6864)
            self.Kp_y = rospy.get_param('controller/Kp_omg_y', 0.6864)
            self.Ki_y = rospy.get_param('controller/Ki_omg_y', 0.6864)
            self.Kd_y = rospy.get_param('controller/Kd_omg_y', 0.6864)
            self.Kp_z = rospy.get_param('controller/Kp_omg_z', 0.5)
            self.Ki_z = rospy.get_param('controller/Ki_omg_z', 1.5792)

        elif self.state in {"follow path", "drag drone"}:
            self.Kp_x = rospy.get_param('controller/Kp_dt_x', 0.6864)
            self.Ki_x = rospy.get_param('controller/Ki_dt_x', 0.6864)
            self.Kd_x = rospy.get_param('controller/Kd_dt_x', 0.6864)
            self.Kp_y = rospy.get_param('controller/Kp_dt_y', 0.6864)
            self.Ki_y = rospy.get_param('controller/Ki_dt_y', 0.6864)
            self.Kd_y = rospy.get_param('controller/Kd_dt_y', 0.6864)
            self.Kp_z = rospy.get_param('controller/Kp_dt_z', 0.5)
            self.Ki_z = rospy.get_param('controller/Ki_dt_z', 1.5792)

    def reset_pid_gains(self):
        '''Resets the PID gains to the rosparam vaules after tasks "undamped
        spring" or "viscous fluid".
        '''
        self.Kp_x = rospy.get_param('controller/Kp_x', 0.6864)
        self.Ki_x = rospy.get_param('controller/Ki_x', 0.6864)
        self.Kd_x = rospy.get_param('controller/Kd_x', 0.6864)
        self.Kp_y = rospy.get_param('controller/Kp_y', 0.6864)
        self.Ki_y = rospy.get_param('controller/Ki_y', 0.6864)
        self.Kd_y = rospy.get_param('controller/Kd_y', 0.6864)
        self.Kp_z = rospy.get_param('controller/Kp_z', 0.5)
        self.Ki_z = rospy.get_param('controller/Ki_z', 1.5792)

    def gamepad_flying(self):
        '''Sets up a ros subscriber to read out the inputs given by the gamepad
        and removes this subscriber when the controller switches states.
        '''
        self.meas = {}
        (self.meas['meas_pos_x'], self.meas['meas_pos_y'],
         self.meas['meas_pos_z']) = [], [], []
        (self.meas['est_pos_x'], self.meas['est_pos_y'],
         self.meas['est_pos_z']) = [], [], []
        (self.meas['est_vel_x'], self.meas['est_vel_y'],
         self.meas['est_vel_z']) = [], [], []
        (self.meas['input_x'], self.meas['input_y'],
         self.meas['input_z']) = [], [], []
        self.meas['meas_time'], self.meas['est_time'] = [], []

        self.meas['meas_pos_x'].append(self.meas_pos_x)
        self.meas['meas_pos_y'].append(self.meas_pos_y)
        self.meas['meas_pos_z'].append(self.meas_pos_z)
        self.meas['meas_time'].append(self.meas_time)

        self.gamepad_input = rospy.Subscriber('bebop/cmd_vel',
                                              Twist, self.retrieve_gp_input)

        while not (rospy.is_shutdown() or self.state_killed):
            if self.state_changed:
                self.state_changed = False
                break
            rospy.sleep(0.1)
        self.gamepad_input.unregister()
        print yellow(' Length of measurement: ', len(self.meas['meas_pos_x']))
        io.savemat('../kalman_check.mat', self.meas)

    def dodge_dyn_obst(self):
        '''Uses OMG-tools to dodge a moving obstacle coming towards the drone
        and returns back to original position when possible.
        '''
        radius = 0.50
        self.dynamic_obst = [Obstacle(obst_type=String(
                            data="inf cylinder"),
                            shape=[radius],
                            pose=[self.ctrl_r_pos.position.x,
                                  self.ctrl_r_pos.position.y],
                            velocity=[self.ctrl_r_vel.linear.x,
                                      self.ctrl_r_vel.linear.y])]
        self.static_obst = []
        self.reset_markers()
        self.config_mp()
        self.set_omg_goal(self.drone_pose_est)
        self.omg_index = 1
        self.set_omg_update_time()
        self.set_ff_pid_gains()

        while not (self.state_changed or (
                rospy.is_shutdown() or self.state_killed)):
            # Becomes True when goal is set.
            # Update dynamic obstacle info for when motionplanner is fired.
            self.dynamic_obst = [Obstacle(obst_type=String(
                                  data="inf cylinder"),
                                  shape=[radius],
                                  pose=[self.ctrl_r_pos.position.x,
                                        self.ctrl_r_pos.position.y],
                                  velocity=[self.ctrl_r_vel.linear.x,
                                            self.ctrl_r_vel.linear.y])]
            self.omg_update()
            self.rate.sleep()
        self.dynamic_obst = []
        self.config_mp()
        self.hover_setpoint = self.drone_pose_est
        self.reset_pid_gains()
        if not self.state_killed:
            self.state_changed = False
        self.state_killed = False

    ####################
    # Helper functions #
    ####################

    def check_goal_reached(self):
        '''Determines whether goal is reached.
        Returns:
            not stop: boolean whether goal is reached. If not, controller
                      proceeds to goal.
        '''
        pos_nrm = self.position_diff_norm(self.drone_pose_est.position,
                                          self._goal.position)

        self.target_reached = (pos_nrm < self.pos_nrm_tol)
        if self.target_reached:
            # io.savemat('../mpc_calc_time.mat', self.calc_time)
            print yellow('=========================')
            print yellow('==== Target Reached! ====')
            print yellow('=========================')

    def rotate_vel_cmd(self, vel):
        '''Transforms the velocity commands from the global world frame to the
        rotated world frame world_rot.
        '''
        self.ff_velocity = self.transform_twist(vel, "world", "world_rot")

    def convert_vel_cmd(self):
        '''Converts a velocity command to a desired input angle according to
        the state space representation of the inverse velocity model.
        '''
        u = np.array([[self.ff_velocity.twist.linear.x],
                      [self.ff_velocity.twist.linear.y],
                      [self.ff_velocity.twist.linear.z]])

        Y = np.matmul(self.C, self.X) + np.matmul(self.D, u)
        self.X = np.matmul(self.A, self.X) + np.matmul(self.B, u)
        self.ff_cmd.linear.x = Y[0, 0]
        self.ff_cmd.linear.y = Y[1, 0]
        self.ff_cmd.linear.z = Y[2, 0]

    def combine_ff_fb(self, pos_desired, vel_desired):
        '''Combines the feedforward and feedback commands to generate the full
        input angle command.
        '''
        # Transform feedback desired position and velocity from world frame to
        # world_rot frame
        fb_cmd = self.feedbeck(pos_desired, vel_desired.twist)

        if self.state == 'follow path':
            self.full_cmd.twist.linear.x = max(min((
                    self.ff_cmd.linear.x + fb_cmd.linear.x),
                    self.max_input), - self.max_input)
            self.full_cmd.twist.linear.y = max(min((
                    self.ff_cmd.linear.y + fb_cmd.linear.y),
                    self.max_input), - self.max_input)
            self.full_cmd.twist.linear.z = max(min((
                    self.ff_cmd.linear.z + fb_cmd.linear.z),
                    self.max_input), - self.max_input)
            self.full_cmd.twist.angular.z = max(min((
                    self.ff_cmd.angular.z + fb_cmd.angular.z),
                    self.max_input), - self.max_input)
        else:
            self.full_cmd.twist.linear.x = max(min((
                    fb_cmd.linear.x),
                    self.max_input), - self.max_input)
            self.full_cmd.twist.linear.y = max(min((
                    fb_cmd.linear.y),
                    self.max_input), - self.max_input)
            self.full_cmd.twist.linear.z = max(min((
                    fb_cmd.linear.z),
                    self.max_input), - self.max_input)
            self.full_cmd.twist.angular.z = max(min((
                    fb_cmd.angular.z),
                    self.max_input), - self.max_input)

    def feedbeck(self, pos_desired, vel_desired):
        '''Whenever the target is reached, apply position feedback to the
        desired end position to remain in the correct spot and compensate for
        drift.
        Tustin discretized PID controller for x and y, PI for z.
        '''
        fb_cmd = Twist()

        if (self.state in {"undamped spring", "viscous fluid"}):
            # # PD
            pos_error = PointStamped()
            pos_error.header.frame_id = "world"
            pos_error.point.x = (pos_desired.point.x -
                                 self.drone_pose_est.position.x)
            pos_error.point.y = (pos_desired.point.y -
                                 self.drone_pose_est.position.y)
            pos_error.point.z = (pos_desired.point.z -
                                 self.drone_pose_est.position.z)

            vel_error = PointStamped()
            vel_error.header.frame_id = "world"
            vel_error.point.x = vel_desired.linear.x - self.drone_vel_est.x
            vel_error.point.y = vel_desired.linear.y - self.drone_vel_est.y

            pos_error = self.transform_point(pos_error, "world", "world_rot")
            vel_error = self.transform_point(vel_error, "world", "world_rot")

            fb_cmd.linear.x = max(- self.max_input, min(self.max_input, (
                    self.Kp_x*pos_error.point.x +
                    self.Kd_x*vel_error.point.x)))
            fb_cmd.linear.y = max(- self.max_input, min(self.max_input, (
                    self.Kp_y*pos_error.point.y +
                    self.Kd_y*vel_error.point.y)))
            fb_cmd.linear.z = max(- self.max_input, min(self.max_input, (
                    self.Kp_z*pos_error.point.z)))
        else:
            # # PID
            pos_error_prev = self.pos_error_prev
            pos_error = PointStamped()
            pos_error.header.frame_id = "world"
            pos_error.point.x = (pos_desired.point.x
                                 - self.drone_pose_est.position.x)
            pos_error.point.y = (pos_desired.point.y
                                 - self.drone_pose_est.position.y)
            pos_error.point.z = (pos_desired.point.z
                                 - self.drone_pose_est.position.z)

            vel_error_prev = self.vel_error_prev
            vel_error = PointStamped()
            vel_error.header.frame_id = "world"
            vel_error.point.x = vel_desired.linear.x - self.drone_vel_est.x
            vel_error.point.y = vel_desired.linear.y - self.drone_vel_est.y

            pos_error = self.transform_point(pos_error, "world", "world_rot")
            vel_error = self.transform_point(vel_error, "world", "world_rot")

            fb_cmd.linear.x = max(- self.max_input, min(self.max_input, (
                    self.fb_cmd_prev.linear.x +
                    (self.Kp_x + self.Ki_x*self._sample_time/2) *
                    pos_error.point.x +
                    (-self.Kp_x + self.Ki_x*self._sample_time/2) *
                    pos_error_prev.point.x +
                    self.Kd_x*(vel_error.point.x - vel_error_prev.point.x))))

            fb_cmd.linear.y = max(- self.max_input, min(self.max_input, (
                    self.fb_cmd_prev.linear.y +
                    (self.Kp_y + self.Ki_y*self._sample_time/2) *
                    pos_error.point.y +
                    (-self.Kp_y + self.Ki_y*self._sample_time/2) *
                    pos_error_prev.point.y +
                    self.Kd_y*(vel_error.point.y - vel_error_prev.point.y))))

            fb_cmd.linear.z = max(- self.max_input, min(self.max_input, (
                    self.fb_cmd_prev.linear.z +
                    (self.Kp_z + self.Ki_z*self._sample_time/2) *
                    pos_error.point.z +
                    (-self.Kp_z + self.Ki_z*self._sample_time/2) *
                    pos_error_prev.point.z)))

        # Add theta feedback to remain at zero yaw angle
        angle_error = ((((self.desired_yaw - self.real_yaw) -
                         np.pi) % (2*np.pi)) - np.pi)
        # print 'desired yaw angle', self.desired_yaw
        # print 'real yaw angle', self.real_yaw
        # print 'angle error', angle_error
        K_theta = self.K_theta + (np.pi - abs(angle_error))/np.pi*0.2
        # print 'K_theta', K_theta
        fb_cmd.angular.z = (K_theta*angle_error)
        # fb_cmd.angular.z = (self.K_theta*angle_error)

        self.pos_error_prev = pos_error
        self.vel_error_prev = vel_error
        self.fb_cmd_prev = fb_cmd

        # Publish the position error.
        self.pos_error_pub.publish(pos_error)

        return fb_cmd

    def safety_brake(self):
        '''Brake as emergency measure: Bebop brakes automatically when
            /bebop/cmd_vel topic receives all zeros.
        '''
        self.full_cmd.twist = Twist()
        self.cmd_vel.publish(self.full_cmd.twist)

    def repeat_safety_brake(self):
        '''More permanent emergency measure: keep safety braking until new task
        (eg. land) is given.
        '''
        while not (rospy.is_shutdown() or self.state_killed):
            self.safety_brake()
            self.rate.sleep()

    def get_pose_est(self):
        '''Retrieves a new pose estimate from world model.
        '''
        # This service is provided as soon as vive is ready. (See bebop_test)
        rospy.wait_for_service("/world_model/get_pose")
        try:
            pose_est = rospy.ServiceProxy(
                "/world_model/get_pose", GetPoseEst)
            resp = pose_est(self.full_cmd)

            yhat = resp.pose_est.point
            pose = Pose()
            pose.position.x = yhat.x
            pose.position.y = yhat.y
            pose.position.z = yhat.z

            vhat = resp.vel_est.point
            yaw = resp.yaw
            measurement_valid = resp.measurement_valid

            return pose, vhat, yaw, measurement_valid

        except rospy.ServiceException, e:
            print highlight_red('Service call failed: %s') % e
            return

    def load_trajectories(self):
        self._traj['u'] = self._traj_strg['u'][:]
        self._traj['v'] = self._traj_strg['v'][:]
        self._traj['w'] = self._traj_strg['w'][:]
        self._traj['x'] = self._traj_strg['x'][:]
        self._traj['y'] = self._traj_strg['y'][:]
        self._traj['z'] = self._traj_strg['z'][:]

    def store_trajectories(self, u_traj, v_traj, w_traj,
                           x_traj, y_traj, z_traj, success):
        '''Stores the trajectories and indicate that new trajectories have
        been calculated.

        Args:
            u_traj : trajectory speed in z-direction
            v_traj : trajectory speed in x-direction
            w_traj : trajectory speed in y-direction
            x_traj : trajectory position in x-direction
            y_traj : trajectory position in y-direction
            z_traj : trajectory position in z-direction
        '''
        self._traj_strg = {}
        u_traj, v_traj, w_traj = self.pad_lpf(list(u_traj),
                                              list(v_traj),
                                              list(w_traj))
        self._traj_strg = {'u': u_traj, 'v': v_traj, 'w': w_traj,
                           'x': x_traj, 'y': y_traj, 'z': z_traj}
        self._new_trajectories = True
        self.calc_succeeded = success

        x_traj = self._traj_strg['x'][:]
        y_traj = self._traj_strg['y'][:]
        z_traj = self._traj_strg['z'][:]
        self.publish_desired(x_traj, y_traj, z_traj)

    def transform_twist(self, twist, _from, _to):
        '''Transforms twist (geometry_msgs/Twist) from frame "_from" to
        frame "_to".
        Arguments:
            - _from, _to = string, name of frame
        '''
        cmd_vel = PointStamped()
        cmd_vel.header = twist.header
        cmd_vel.point = twist.twist.linear
        cmd_vel_rotated = self.transform_point(cmd_vel, _from, _to)

        twist_rotated = TwistStamped()
        twist_rotated.header.stamp = twist.header.stamp
        twist_rotated.twist.linear = cmd_vel_rotated.point

        return twist_rotated

    def transform_point(self, point, _from, _to):
        '''Transforms point from _from frame to _to frame.
        '''
        transform = self.tfBuffer.lookup_transform(
            _to, _from, rospy.Time(0), rospy.Duration(0.1))
        point_transformed = tf2_geom.do_transform_point(point, transform)

        return point_transformed

    def get_ctrl_r_pos(self, ctrl_pose):
        '''Retrieves the position of the right hand controller.
        '''
        self.ctrl_r_pos = ctrl_pose.pose

    def get_ctrl_r_vel(self, ctrl_vel):
        '''Retrieves the velocity of the right hand controller.
        '''
        self.ctrl_r_vel = ctrl_vel.twist

    def get_ctrl_l_pos(self, ctrl_pose):
        '''Retrieves the position of the left hand controller and executes
        drawing when trigger is pressed.
        '''
        self.ctrl_l_pos = ctrl_pose.pose
        if (self.state in {"draw path slow", "draw path fast"} and self.draw):
            self.draw_ctrl_path()

    def get_ctrl_l_vel(self, ctrl_vel):
        '''Retrieves the velocity of the right hand controller.
        '''
        self.ctrl_l_vel = ctrl_vel.twist

    def trackpad_press(self, trackpad_pressed):
        '''If state is equal to state in list and trackpad is pressed,
        set self.state_changed to true to switch states.
        '''
        if trackpad_pressed.data and not self.trackpad_held:
            if (self.state in {"draw path slow", "draw path fast"}):
                self.stop_drawing = True
            elif (self.state in {"drag drone",
                                 "viscous fluid",
                                 "undamped spring",
                                 "place hex obstacles",
                                 "place cyl obstacles",
                                 "place slalom obstacles",
                                 "place plate obstacles",
                                 "place window obstacles",
                                 "gamepad flying",
                                 "dodge dyn obst"}):
                self.state_changed = True
            self.trackpad_held = True

        elif not trackpad_pressed.data and self.trackpad_held:
            self.trackpad_held = False

    def r_trigger(self, button_pushed):
        '''When button is pushed on the right hand controller, depending on the
        current state, either sets goal to be equal to the position of the
        controller.
        '''

        if (((self.state == "omg fly") or (self.state == "omg standby"))
           and button_pushed.data and not self.r_trigger_held):
            self.target_reached = False
            goal = Pose()
            goal.position.x = self.ctrl_r_pos.position.x
            goal.position.y = self.ctrl_r_pos.position.y
            goal.position.z = self.ctrl_r_pos.position.z
            self.set_omg_goal(goal)
            self.r_trigger_held = True

        elif (not button_pushed.data and self.r_trigger_held):
            self.r_trigger_held = False

    def l_trigger(self, button_pushed):
        '''When button is pushed on the left hand controller, depending on the
        current state, allows path that the controller describes to be saved or
        enables dragging of the drone.
        '''

        if self.state in {"draw path slow", "draw path fast"}:
            # Start drawing and saving path
            if (button_pushed.data and not self.draw):
                self.drawn_pos_x = []
                self.drawn_pos_y = []
                self.drawn_pos_z = []
                self.draw = True
                print highlight_blue(' Drawing ... ')

            if (not button_pushed.data and self.draw):
                self.draw = False

        elif self.state == "drag drone":
            if (button_pushed.data and not self.drag):
                self.drag = True
            elif (not button_pushed.data and self.drag):
                self.drag = False

        if (self.state in {"place cyl obstacles",
                           "place slalom obstacles",
                           "place hex obstacles",
                           "place plate obstacles",
                           "place window obstacles"}):

            if (button_pushed.data and not self.draw):
                self.draw = True
            if (not button_pushed.data and self.draw):
                self.draw = False

    def bebop_flying_state(self, flying_state):
        '''Checks whether the drone is standing on the ground or flying and
        changes the self.airborne variable accordingly.
        '''
        if flying_state.state == 0:
            self.airborne = False
        elif flying_state.state == 2:
            self.airborne = True

    def diff_interp_traj(self):
        '''Differentiate and interpolate obtained trajectory to obtain
        feedforward velocity commands.
        '''
        self.differentiate_traj()

        # Search for the highest velocity in the trajectory to determine the
        # step size needed for interpolation.
        highest_vel = max(max(self.drawn_vel_x),
                          max(self.drawn_vel_y),
                          max(self.drawn_vel_z))
        self.interpolate_drawn_traj(self.max_vel/highest_vel)

    def differentiate_traj(self):
        '''Numerically differentiates position traject to recover a list of
        feedforward velocities.
        '''
        self.drawn_vel_x = (
                        np.diff(self.drawn_pos_x)/self._sample_time).tolist()
        self.drawn_vel_y = (
                        np.diff(self.drawn_pos_y)/self._sample_time).tolist()
        self.drawn_vel_z = (
                        np.diff(self.drawn_pos_z)/self._sample_time).tolist()

    def interpolate_drawn_traj(self, step):
        '''Linearly interpolates a list so that it contains the desired amount
        of elements where the element distance is equal to step.
        '''
        self.drawn_pos_x = np.interp(np.arange(0, len(self.drawn_pos_x), step),
                                     range(len(self.drawn_pos_x)),
                                     self.drawn_pos_x).tolist()
        self.drawn_pos_y = np.interp(np.arange(0, len(self.drawn_pos_y), step),
                                     range(len(self.drawn_pos_y)),
                                     self.drawn_pos_y).tolist()
        self.drawn_pos_z = np.interp(np.arange(0, len(self.drawn_pos_z), step),
                                     range(len(self.drawn_pos_z)),
                                     self.drawn_pos_z).tolist()

    def low_pass_filter_drawn_traj(self):
        '''Low pass filter the trajectory drawn with the controller in order to
        be suitable for the drone to track it.
        '''
        self.drawn_pos_x = filtfilt(
            self.butter_b, self.butter_a, self.drawn_pos_x, padlen=50).tolist()
        self.drawn_pos_y = filtfilt(
            self.butter_b, self.butter_a, self.drawn_pos_y, padlen=50).tolist()
        self.drawn_pos_z = filtfilt(
            self.butter_b, self.butter_a, self.drawn_pos_z, padlen=50).tolist()

        # Plot the smoothed trajectory in Rviz.
        self.draw_smoothed_path()

    def position_diff_norm(self, point1, point2):
        '''Returns the norm of the difference vector between two given points.
        point1 and point2 are geometry_msgs/Point objects.
        '''
        norm = np.linalg.norm(np.array([point1.x, point1.y, point1.z])
                              - np.array([point2.x, point2.y, point2.z]))
        return norm

    def retrieve_gp_input(self, gp_input):
        '''Reads out the commands sent by the gamepad and sends these to the
        kalman filter to update the state estimation.
        '''
        self.full_cmd.twist.linear = gp_input.linear
        self.full_cmd.header.stamp = rospy.get_rostime()
        (self.drone_pose_est, self.drone_vel_est, self.real_yaw,
            measurement_valid) = self.get_pose_est()
        self.publish_vhat_vector(self.drone_pose_est.position,
                                 self.drone_vel_est)

        # Saves data to be able to compare the velocity estimation to the
        # numerically differentiated velocity afterward.
        self.meas['est_pos_x'].append(self.drone_pose_est.position.x)
        self.meas['est_pos_y'].append(self.drone_pose_est.position.y)
        self.meas['est_pos_z'].append(self.drone_pose_est.position.z)
        self.meas['est_vel_x'].append(self.drone_vel_est.x)
        self.meas['est_vel_y'].append(self.drone_vel_est.y)
        self.meas['est_vel_z'].append(self.drone_vel_est.z)
        self.meas['est_time'].append(rospy.get_time())
        self.meas['input_x'].append(self.full_cmd.twist.linear.x)
        self.meas['input_x'].append(self.full_cmd.twist.linear.y)
        self.meas['input_x'].append(self.full_cmd.twist.linear.z)

        if self.meas['meas_time'][-1] != self.meas_time:
            self.meas['meas_pos_x'].append(self.meas_pos_x)
            self.meas['meas_pos_y'].append(self.meas_pos_y)
            self.meas['meas_pos_z'].append(self.meas_pos_z)
            self.meas['meas_time'].append(self.meas_time)

    def new_measurement(self, data):
        '''Reads out vive pose and saves this data when flying with the gamepad.
        '''
        if (self.state == "gamepad flying"):
            self.meas_pos_x = data.meas_world.pose.position.x
            self.meas_pos_y = data.meas_world.pose.position.y
            self.meas_pos_z = data.meas_world.pose.position.z
            self.meas_time = rospy.get_time()

    def pad_lpf(self, x_vec, y_vec, z_vec):
        '''Adds padding based on derivative of curve at the end, since padding
        will be removed due to phase shift of low pass filter which is applied
        after padding.
        '''
        # Reverse vector.
        x_vec.reverse()
        y_vec.reverse()
        z_vec.reverse()
        # Add padding and filter.
        dx = (3./2.*x_vec[-1] - 2.*x_vec[-2] + 1./2.*x_vec[-3])
        dy = (3./2.*y_vec[-1] - 2.*y_vec[-2] + 1./2.*y_vec[-3])
        dz = (3./2.*z_vec[-1] - 2.*z_vec[-2] + 1./2.*z_vec[-3])

        padlen = 50
        x_pad = [x_vec[-1] + (np.arange(dx, dx*(padlen + 1), dx).tolist())[i]
                 for i in range(padlen)]
        y_pad = [y_vec[-1] + (np.arange(dy, dy*(padlen + 1), dy).tolist())[i]
                 for i in range(padlen)]
        z_pad = [z_vec[-1] + (np.arange(dz, dz*(padlen + 1), dz).tolist())[i]
                 for i in range(padlen)]

        x_vec = lfilter(self.butter_b, self.butter_a, x_vec + x_pad).tolist()
        y_vec = lfilter(self.butter_b, self.butter_a, y_vec + y_pad).tolist()
        z_vec = lfilter(self.butter_b, self.butter_a, z_vec + z_pad).tolist()

        # Again reverse vector.
        x_vec.reverse()
        y_vec.reverse()
        z_vec.reverse()
        # Cutoff last part since this is created due to lpf
        x_vec = x_vec[: -padlen]
        y_vec = y_vec[: -padlen]
        z_vec = z_vec[: -padlen]

        return x_vec, y_vec, z_vec

    def pad_in_front(self, x_vec, y_vec, z_vec, pad, zeros=False):
        '''Adds padding of len pad in front of vectors.
        '''
        if zeros:
            x_vec = [0. for i in range(pad)] + x_vec
            y_vec = [0. for i in range(pad)] + y_vec
            z_vec = [0. for i in range(pad)] + z_vec
        else:
            x_vec = [x_vec[0] for i in range(pad)] + x_vec
            y_vec = [y_vec[0] for i in range(pad)] + y_vec
            z_vec = [z_vec[0] for i in range(pad)] + z_vec

        return x_vec, y_vec, z_vec

    #######################################
    # Functions for plotting Rviz markers #
    #######################################

    def _marker_setup(self):
        '''Setup markers to display the desired and real path of the drone in
        rviz, along with the current position in the omg-tools generated
        position list.
        '''
        # Obstacles
        self.rviz_obst = MarkerArray()

        # Desired path
        self._desired_path = Marker()
        self._desired_path.header.frame_id = 'world'
        self._desired_path.ns = "trajectory_desired"
        self._desired_path.id = 0
        self._desired_path.type = 4  # Line List.
        self._desired_path.action = 0
        self._desired_path.scale.x = 0.03
        self._desired_path.color.r = 1.0
        self._desired_path.color.g = 0.0
        self._desired_path.color.b = 0.0
        self._desired_path.color.a = 1.0
        self._desired_path.lifetime = rospy.Duration(0)

        self.pos_index = 0
        self.old_len = 0

        # Real path
        self._real_path = Marker()
        self._real_path.header.frame_id = 'world'
        self._real_path.ns = "trajectory_real"
        self._real_path.id = 1
        self._real_path.type = 4  # Line List.
        self._real_path.action = 0
        self._real_path.scale.x = 0.03
        self._real_path.color.r = 0.0
        self._real_path.color.g = 1.0
        self._real_path.color.b = 0.0
        self._real_path.color.a = 1.0
        self._real_path.lifetime = rospy.Duration(0)

        # feedforward position and velocity
        self.current_ff_vel = Marker()
        self.current_ff_vel.header.frame_id = 'world'
        self.current_ff_vel.ns = "current_ff_vel"
        self.current_ff_vel.id = 2
        self.current_ff_vel.type = 0  # Arrow
        self.current_ff_vel.action = 0
        self.current_ff_vel.scale.x = 0.06  # shaft diameter
        self.current_ff_vel.scale.y = 0.1  # head diameter
        self.current_ff_vel.scale.z = 0.15  # head length
        self.current_ff_vel.color.r = 0.0
        self.current_ff_vel.color.g = 0.0
        self.current_ff_vel.color.b = 1.0
        self.current_ff_vel.color.a = 1.0
        self.current_ff_vel.lifetime = rospy.Duration(0)

        # Controller drawn path
        self.drawn_path = Marker()
        self.drawn_path.header.frame_id = 'world'
        self.drawn_path.ns = "drawn_path"
        self.drawn_path.id = 4
        self.drawn_path.type = 4  # Line List.
        self.drawn_path.action = 0
        self.drawn_path.scale.x = 0.03
        self.drawn_path.color.r = 1.0
        self.drawn_path.color.g = 0.86
        self.drawn_path.color.b = 0.0
        self.drawn_path.color.a = 1.0
        self.drawn_path.lifetime = rospy.Duration(0)

        # Smoother version of the path
        self.smooth_path = Marker()
        self.smooth_path.header.frame_id = 'world'
        self.smooth_path.ns = "smooth_path"
        self.smooth_path.id = 5
        self.smooth_path.type = 4  # Line List.
        self.smooth_path.action = 0
        self.smooth_path.scale.x = 0.03
        self.smooth_path.color.r = 1.0
        self.smooth_path.color.g = 0.38
        self.smooth_path.color.b = 0.0
        self.smooth_path.color.a = 1.0
        self.smooth_path.lifetime = rospy.Duration(0)

        # Room contours
        self.room_contours = Marker()
        self.room_contours.header.frame_id = 'world'
        self.room_contours.ns = "room_contours"
        self.room_contours.id = 6
        self.room_contours.type = 4  # Line List.
        self.room_contours.action = 0
        self.room_contours.scale.x = 0.03
        self.room_contours.color.r = 0.8
        self.room_contours.color.g = 0.8
        self.room_contours.color.b = 0.8
        self.room_contours.color.a = 1.0
        self.room_contours.lifetime = rospy.Duration(0)

        # Vhat vector
        self.vhat_vector = Marker()
        self.vhat_vector.header.frame_id = 'world'
        self.vhat_vector.ns = "vhat_vector"
        self.vhat_vector.id = 7
        self.vhat_vector.type = 0  # Arrow.
        self.vhat_vector.action = 0
        self.vhat_vector.scale.x = 0.06  # shaft diameter
        self.vhat_vector.scale.y = 0.1  # head diameter
        self.vhat_vector.scale.z = 0.15  # head length
        self.vhat_vector.color.r = 1.0
        self.vhat_vector.color.g = 1.0
        self.vhat_vector.color.b = 0.3
        self.vhat_vector.color.a = 1.0
        self.vhat_vector.lifetime = rospy.Duration(0)

    def reset_markers(self):
        '''Resets all Rviz markers (except for obstacles).
        '''
        self._desired_path.points = []
        self.trajectory_desired.publish(self._desired_path)
        self.drawn_path.points = []
        self.trajectory_drawn.publish(self.drawn_path)
        self._real_path.points = []
        self.trajectory_real.publish(self._real_path)
        self.smooth_path.points = []
        self.trajectory_smoothed.publish(self.smooth_path)
        self.current_ff_vel.points = [Point(), Point()]
        self.current_ff_vel_pub.publish(self.current_ff_vel)
        self.vhat_vector.points = [Point(), Point()]
        self.vhat_vector_pub.publish(self.vhat_vector)

    def publish_desired(self, x_traj, y_traj, z_traj):
        '''Publish planned x and y trajectory to topic for visualisation in
        rviz.
        '''
        # Still has to be adapted to remove old path when new goal has been
        # set.
        self._desired_path.header.stamp = rospy.get_rostime()

        # Delete points in path that have not been used before new list was
        # calculated.
        self._desired_path.points = self._desired_path.points[
                                            0:(self.old_len + self.pos_index)]

        # After a while list becomes really long so only keep last XXXX values.
        if len(self._desired_path.points) > 1000:
            self._desired_path.points = self._desired_path.points[-1000:]
        self.old_len = len(self._desired_path.points)

        # Add new calculated pos list to old one.
        new_pos = [0]*len(x_traj)
        for k in range(len(x_traj)):
            new_pos[k] = Point(x=x_traj[k], y=y_traj[k], z=z_traj[k])
        self._desired_path.points += new_pos

        self.trajectory_desired.publish(self._desired_path)

    def publish_real(self, x_pos, y_pos, z_pos):
        '''Publish real x and y trajectory to topic for visualisation in
        rviz.
        '''
        self._real_path.header.stamp = rospy.get_rostime()

        point = Point(x=x_pos, y=y_pos, z=z_pos)
        # After a while list becomes really long so only keep last XXXX values.
        if len(self._real_path.points) > 1000:
            self._real_path.points = self._real_path.points[1:] + [point]
        else:
            self._real_path.points.append(point)

        self.trajectory_real.publish(self._real_path)

    def publish_current_ff_vel(self, pos, vel):
        '''Publish current omg-tools velocity input vector where origin of the
        vector is equal to current omg-tools position.
        '''
        self.current_ff_vel.header.stamp = rospy.get_rostime()

        point_start = Point(x=pos.point.x, y=pos.point.y, z=pos.point.z)
        point_end = Point(x=(pos.point.x + vel.twist.linear.x),
                          y=(pos.point.y + vel.twist.linear.y),
                          z=(pos.point.z + vel.twist.linear.z))
        self.current_ff_vel.points = [point_start, point_end]

        self.current_ff_vel_pub.publish(self.current_ff_vel)

    def publish_vhat_vector(self, pos, vel):
        '''Publish current vhat estimate from the kalman filter as a vector
        with origin equal to the current position estimate.
        '''
        self.vhat_vector.header.stamp = rospy.get_rostime()

        point_start = Point(x=pos.x, y=pos.y, z=pos.z)
        point_end = Point(x=(pos.x + vel.x),
                          y=(pos.y + vel.y),
                          z=(pos.z + vel.z))
        self.vhat_vector.points = [point_start, point_end]

        self.vhat_vector_pub.publish(self.vhat_vector)

    def publish_obst_room(self, empty):
        '''Publish static obstacles as well as the boundary of the room.
        '''
        # Delete markers
        marker = Marker()
        marker.ns = "obstacles"
        marker.action = 3  # 3 deletes markers
        self.rviz_obst.markers = [marker]
        self.obst_pub.publish(self.rviz_obst)

        # self.rviz_obst = MarkerArray()
        # self.obst_pub.publish(self.rviz_obst)
        j = 0
        window_plate = 0
        for i, obstacle in enumerate(self.static_obst):
            # Marker setup
            if obstacle.obst_type.data == 'slalom plate':
                j += 1
                d = obstacle.direction
                green_marker = Marker()
                red_marker = Marker()
                green_marker.header.frame_id = 'world'
                red_marker.header.frame_id = 'world'
                green_marker.ns = "obstacles"
                red_marker.ns = "obstacles"
                green_marker.id = i+j+7
                red_marker.id = i+(j-1)+7
                green_marker.type = 3  # Cylinder
                red_marker.type = 3  # Cylinder
                green_marker.action = 0
                red_marker.action = 0
                green_marker.scale.x = obstacle.shape[2]  # x-diameter
                green_marker.scale.y = obstacle.shape[2]  # y-diameter
                green_marker.scale.z = self.room_height  # height
                red_marker.scale.x = obstacle.shape[2]  # x-diameter
                red_marker.scale.y = obstacle.shape[2]  # y-diameter
                red_marker.scale.z = self.room_height  # height

                green_marker.color.r = 0.0
                green_marker.color.g = 1.0
                green_marker.color.b = 0.0
                green_marker.color.a = 0.5
                red_marker.color.r = 1.0
                red_marker.color.g = 0.0
                red_marker.color.b = 0.0
                red_marker.color.a = 0.5
                green_marker.lifetime = rospy.Duration(0)
                red_marker.lifetime = rospy.Duration(0)

                green_marker.pose.position = Point(
                                  x=obstacle.edge[0],
                                  y=obstacle.edge[1] + d*0.5*obstacle.shape[2],
                                  z=obstacle.edge[2])
                red_marker.pose.position = Point(
                                  x=obstacle.edge[0],
                                  y=obstacle.edge[1] + d*1.5*obstacle.shape[2],
                                  z=obstacle.edge[2])
                green_marker.header.stamp = rospy.get_rostime()
                red_marker.header.stamp = rospy.get_rostime()

                # Append marker to marker array:
                self.rviz_obst.markers.append(green_marker)
                self.rviz_obst.markers.append(red_marker)

            elif obstacle.obst_type.data == 'window plate':
                obstacle_marker = Marker()
                obstacle_marker.header.frame_id = 'world'
                obstacle_marker.ns = "obstacles"
                obstacle_marker.id = i+j+7
                obstacle_marker.action = 0
                obstacle_marker.color.r = 0.188
                obstacle_marker.color.g = 0.525
                obstacle_marker.color.b = 0.82
                obstacle_marker.color.a = 0.5
                obstacle_marker.lifetime = rospy.Duration(0)

                obstacle_marker.type = 1  # Cuboid
                obstacle_marker.scale.x = obstacle.shape[2]  # thickness
                # shape = [height, width, thickness]
                # Window plates come by 4. You need the dimensions of two
                # overlapping plates to know the edge of the window hole
                # corresponding to the current plate.
                window_plate = i % 4
                if window_plate == 0:
                    left = obstacle
                    up = self.static_obst[i+2]
                    down = self.static_obst[i+3]
                    pose = [left.pose[0],
                            -(self.room_depth/2. - left.shape[1]
                              + left.shape[2]/2.),
                            ((self.room_height - up.shape[0]) +
                             down.shape[0])/2.]
                    obstacle_marker.pose.position = Point(x=pose[0],
                                                          y=pose[1],
                                                          z=pose[2])

                    obstacle_marker.scale.y = left.shape[2]
                    obstacle_marker.scale.z = (
                        self.room_height - (up.shape[0] + down.shape[0]))
                elif window_plate == 1:
                    right = obstacle
                    up = self.static_obst[i+1]
                    down = self.static_obst[i+2]
                    pose = [right.pose[0],
                            (self.room_depth/2. - right.shape[1]
                             + right.shape[2]/2.),
                            ((self.room_height - up.shape[0]) +
                             down.shape[0])/2.]
                    obstacle_marker.pose.position = Point(x=pose[0],
                                                          y=pose[1],
                                                          z=pose[2])
                    obstacle_marker.scale.y = right.shape[2]
                    obstacle_marker.scale.z = (
                        self.room_height - (up.shape[0] + down.shape[0]))
                elif window_plate == 2:
                    left = self.static_obst[i-2]
                    right = self.static_obst[i-1]
                    up = obstacle
                    pose = [up.pose[0],
                            (left.shape[1] - right.shape[1])/2.,
                            (self.room_height - up.shape[0] + up.shape[2]/2.)]
                    obstacle_marker.pose.position = Point(x=pose[0],
                                                          y=pose[1],
                                                          z=pose[2])
                    obstacle_marker.scale.y = (
                        self.room_depth - (left.shape[1] + right.shape[1])
                        + 2*up.shape[2])
                    obstacle_marker.scale.z = up.shape[2]
                elif window_plate == 3:
                    left = self.static_obst[i-3]
                    right = self.static_obst[i-2]
                    down = obstacle
                    pose = [down.pose[0],
                            (left.shape[1] - right.shape[1])/2.,
                            down.shape[0] - down.shape[2]/2.]
                    obstacle_marker.pose.position = Point(x=pose[0],
                                                          y=pose[1],
                                                          z=pose[2])
                    obstacle_marker.scale.y = (
                        self.room_depth - (left.shape[1] + right.shape[1])
                        + 2*down.shape[2])
                    obstacle_marker.scale.z = down.shape[2]

                obstacle_marker.header.stamp = rospy.get_rostime()
                # Append marker to marker array:
                self.rviz_obst.markers.append(obstacle_marker)

            else:
                obstacle_marker = Marker()
                obstacle_marker.header.frame_id = 'world'
                obstacle_marker.ns = "obstacles"
                obstacle_marker.id = i+j+7
                obstacle_marker.action = 0
                obstacle_marker.color.r = 1.0
                obstacle_marker.color.g = 1.0
                obstacle_marker.color.b = 1.0
                obstacle_marker.color.a = 0.5
                obstacle_marker.lifetime = rospy.Duration(0)

                if obstacle.obst_type.data == 'inf cylinder':
                    obstacle_marker.type = 3  # Cylinder
                    obstacle.shape = [obstacle.shape[0], self.room_height]
                    obstacle.pose = [obstacle.pose[0],
                                     obstacle.pose[1],
                                     self.room_height/2]
                    obstacle_marker.scale.x = obstacle.shape[0] * 2  # x-diam
                    obstacle_marker.scale.y = obstacle.shape[0] * 2  # y-diam
                    obstacle_marker.scale.z = obstacle.shape[1]  # height
                elif obstacle.obst_type.data == 'hexagon':
                    obstacle_marker.type = 3  # Cylinder
                    obstacle_marker.scale.x = obstacle.shape[0] * 2  # x-diam
                    obstacle_marker.scale.y = obstacle.shape[0] * 2  # y-diam
                    obstacle_marker.scale.z = obstacle.shape[1]  # height
                elif obstacle.obst_type.data == 'plate':
                    obstacle_marker.type = 1  # Cuboid
                    obstacle_marker.scale.x = obstacle.shape[2]  # thickness
                    obstacle_marker.scale.y = obstacle.shape[1]  # width
                    obstacle_marker.scale.z = obstacle.shape[0]  # height
                    quaternion = tf.transformations.quaternion_from_euler(
                                                    0., 0., obstacle.direction)
                    obstacle_marker.pose.orientation.x = quaternion[0]
                    obstacle_marker.pose.orientation.y = quaternion[1]
                    obstacle_marker.pose.orientation.z = quaternion[2]
                    obstacle_marker.pose.orientation.w = quaternion[3]

                obstacle_marker.pose.position = Point(x=obstacle.pose[0],
                                                      y=obstacle.pose[1],
                                                      z=obstacle.pose[2])
                obstacle_marker.header.stamp = rospy.get_rostime()

                # Append marker to marker array:
                self.rviz_obst.markers.append(obstacle_marker)

        self.reset_markers()
        self.obst_pub.publish(self.rviz_obst)
        self.draw_room_contours()

    def draw_ctrl_path(self):
        '''Publish real x and y trajectory to topic for visualisation in
        rviz.
        '''
        self.drawn_path.header.stamp = rospy.get_rostime()

        point = Point(x=self.ctrl_l_pos.position.x,
                      y=self.ctrl_l_pos.position.y,
                      z=self.ctrl_l_pos.position.z)
        self.drawn_path.points.append(point)

        self.drawn_pos_x += [point.x]
        self.drawn_pos_y += [point.y]
        self.drawn_pos_z += [point.z]

        self.trajectory_drawn.publish(self.drawn_path)

    def draw_smoothed_path(self):
        '''Publish the smoothed x and y trajectory to topic for visualisation
        in rviz.
        '''
        self.smooth_path.header.stamp = rospy.get_rostime()
        self.smooth_path.points = []

        for index in range(len(self.drawn_pos_x)):
            point = Point(x=self.drawn_pos_x[index],
                          y=self.drawn_pos_y[index],
                          z=self.drawn_pos_z[index])
            self.smooth_path.points.append(point)

        self.trajectory_smoothed.publish(self.smooth_path)

    def draw_room_contours(self):
        '''Publish the edges of the room for visualization in rviz.
        '''
        self.room_contours.header.stamp = rospy.get_rostime()

        bottom_left = Point(x=-self.room_width/2.,
                            y=-self.room_depth/2.)
        bottom_right = Point(x=self.room_width/2.,
                             y=-self.room_depth/2.)
        top_right = Point(x=self.room_width/2.,
                          y=self.room_depth/2.)
        top_left = Point(x=-self.room_width/2.,
                         y=self.room_depth/2.)
        corners = [bottom_left, bottom_right, top_right, top_left, bottom_left]
        for point in corners:
            self.room_contours.points.append(point)

        self.draw_room.publish(self.room_contours)


if __name__ == '__main__':
    controller = Controller()
    controller.start()
