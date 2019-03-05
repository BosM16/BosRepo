#!/usr/bin/env python

from geometry_msgs.msg import (Twist, TwistStamped, Point, PointStamped,
                               Pose, PoseStamped)
from std_msgs.msg import Bool, Empty, String
from visualization_msgs.msg import Marker

from bebop_demo.msg import Trigger, Trajectories, Obstacle

from bebop_demo.srv import GetPoseEst, ConfigMotionplanner

import rospy
import numpy as np
from scipy.signal import butter, lfilter
import tf
import tf2_ros
import tf2_geometry_msgs as tf2_geom
import time

from fabulous.color import highlight_red, highlight_green, magenta, green


class VelCommander(object):

    def __init__(self):
        """Initialization of Controller object.

        Args:
            sample_time : Period between computed velocity samples.
            omg_update_time : Period of problem solving.
        """
        rospy.init_node("vel_commander_node")

        self.airborne = False
        self.calc_succeeded = False
        self.target_reached = False
        self.startup = False
        self.state = "initialization"
        self.state_dict = {"standby": self.hover,
                           "emergency": self.repeat_safety_brake,
                           "take-off": self.take_off_land,
                           "land": self.take_off_land,
                           "omg standby": self.hover,
                           "omg fly": self.omg_fly,
                           "draw path": self.draw_traj,
                           "fly to start": self.fly_to_start,
                           "follow path": self.follow_traj,
                           "drag drone": self.drag_drone}
        self.state_changed = False
        self.executing_state = False
        self.state_killed = False

        rospy.set_param(
            "/bebop/bebop_driver/SpeedSettingsMaxRotationSpeedCurrent", 360.0)

        self.Kp_x = rospy.get_param('vel_cmd/Kp_x', 0.6864)
        self.Ki_x = rospy.get_param('vel_cmd/Ki_x', 0.6864)
        self.Kd_x = rospy.get_param('vel_cmd/Kd_x', 0.6864)
        self.Kp_y = rospy.get_param('vel_cmd/Kp_y', 0.6864)
        self.Ki_y = rospy.get_param('vel_cmd/Ki_y', 0.6864)
        self.Kd_y = rospy.get_param('vel_cmd/Kd_y', 0.6864)
        self.Kp_z = rospy.get_param('vel_cmd/Kp_z', 0.5)
        self.Ki_z = rospy.get_param('vel_cmd/Ki_z', 1.5792)
        self.K_theta = rospy.get_param('vel_cmd/K_theta', 0.3)
        self.max_input = rospy.get_param('vel_cmd/max_input', 0.5)
        self.max_vel = rospy.get_param('motionplanner/vmax', 0.5)
        self.safety_treshold = rospy.get_param('vel_cmd/safety_treshold', 0.5)
        self.pos_nrm_tol = rospy.get_param(
                                        'vel_cmd/goal_reached_pos_tol', 0.05)
        # self.angle_nrm_tol = rospy.get_param(
        #                                 'vel_cmd/goal_reached_angle_tol', 0.05)

        self._sample_time = rospy.get_param('vel_cmd/sample_time', 0.01)
        self._update_time = rospy.get_param('vel_cmd/update_time', 0.5)
        self.rate = rospy.Rate(1./self._sample_time)
        self.omg_index = 1

        # Setup low pass filter for trajectory drawing task.
        cutoff_freq_LPF = rospy.get_param('vel_cmd/LPF_cutoff', 0.5)
        LPF_order = rospy.get_param('vel_cmd/LPF_order', 4)

        norm_fc_LPF = cutoff_freq_LPF/(0.5)*self._sample_time
        self.butter_b, self.butter_a = butter(
            LPF_order, norm_fc_LPF, btype='low', analog=False)

        self.X = np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])
        self.desired_yaw = np.pi/2.
        self.real_yaw = 0.0
        self.pos_nrm = np.inf
        self.x_error = 0.
        self.y_error = 0.
        self.z_error = 0.
        self.feedback_cmd_prev = Twist()
        self.pos_error_prev = PointStamped()
        self.vel_error_prev = PointStamped()
        self.measurement_valid = False
        self.safe = False
        self._goal = Pose()
        self.hover_setpoint = Pose()
        self.ctrl_r_pos = Pose()
        self.draw = False
        self.drag = False

        self.cmd_twist_convert = TwistStamped()
        self.cmd_twist_convert.header.frame_id = "world_rot"
        self.cmd_twist_convert.header.stamp = rospy.Time.now()
        self.feedforward_cmd = Twist()
        self.vhat = Point()
        self._trigger = Trigger()

        # Obstacle setup
        self.Sjaaakie = [0.35, 1.3, 1.5, 1.5, 0.75]

        # Marker setup
        self.marker_setup()

        # Coefficients for inverted model of velocity to input angle
        self.initialize_vel_model()

        self._drone_est_pose = Pose()
        self.vive_frame_pose = PoseStamped()

        self._traj = {'u': [0.0], 'v': [0.0], 'w': [0.0],
                      'x': [0.0], 'y': [0.0], 'z': [0.0]}
        self._traj_strg = {'u': [0.0], 'v': [0.0], 'w': [0.0],
                           'x': [0.0], 'y': [0.0], 'z': [0.0]}

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self._mp_trigger_topic = rospy.Publisher(
            'motionplanner/trigger', Trigger, queue_size=1)
        self.trajectory_desired = rospy.Publisher(
            'motionplanner/desired_path', Marker, queue_size=1)
        self.trajectory_real = rospy.Publisher(
            'motionplanner/real_path', Marker, queue_size=1)
        self.trajectory_smoothed = rospy.Publisher(
            'motionplanner/smoothed_path', Marker, queue_size=1)
        self.current_ff_vel_pub = rospy.Publisher(
            'motionplanner/current_ff_vel', Marker, queue_size=1)
        self.obst_pub = rospy.Publisher(
            'motionplanner/rviz_obst', Marker, queue_size=1)
        self.ctrl_state_finish = rospy.Publisher(
            'controller/state_finish', Empty, queue_size=1)
        self.take_off = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)
        self.land = rospy.Publisher('bebop/land', Empty, queue_size=1)

        rospy.Subscriber('motionplanner/result', Trajectories,
                         self.get_mp_result)
        rospy.Subscriber('vive_localization/ready', Empty, self.publish_obst)
        rospy.Subscriber('ctrl_keypress/rtrigger', Bool, self.r_trigger)
        rospy.Subscriber(
            'vive_localization/c1_pose', PoseStamped, self.get_ctrl_r_pos)
        rospy.Subscriber('fsm/state', String, self.switch_state)

    def initialize_vel_model(self):
        '''Initializes model parameters for conversion of desired velocities to
        angle inputs.
        State space model x[k+1] = A*x[k] + B*u[k] in observable canonical
        form, corresponding to discrete time transfer function

                      b0
        G(z) = -----------------
                s^2 + a1*s + a0

        with sampling time equal to vel_cmd_Ts (0.01s).
        '''
        Ax = np.array([[1.947, -0.9481],
                       [1.0000, 0.]])
        Ay = np.array([[1.947, -0.9481],
                       [1.0000, 0.]])
        Az = np.array([[0.9391]])

        self.A = np.zeros([5, 5])
        self.A[0:2, 0:2] = Ax
        self.A[2:4, 2:4] = Ay
        self.A[4:5, 4:5] = Az

        self.B = np.zeros([5, 3])
        self.B[0, 0] = 1
        self.B[2, 1] = 1
        self.B[4, 2] = 1

        self.C = np.zeros([3, 5])
        self.C[0, 0:2] = [0.004232, -0.005015]
        self.C[1, 2:4] = [-0.003704, 0.002797]
        self.C[2, 4:5] = [-0.0002301]

        self.D = np.array([[0.6338, 0.0, 0.0],
                           [0.0, 0.7709, 0.0],
                           [0.0, 0.0, 1.036]])

    def start(self):
        '''Configures,
        Starts the controller's periodical loop.
        '''
        self.configure()
        print green('----    Controller running     ----')

        while not rospy.is_shutdown():
            if self.state_changed:
                self.state_changed = False

                self.executing_state = True
                # Execute state function.
                self.state_dict[self.state]()
                self.executing_state = False

                # State has not finished if it has been killed!
                if not self.state_killed:
                    self.ctrl_state_finish.publish(Empty())
                    print magenta('---- Publish state finished ----')
                self.state_killed = False

                # Adjust goal to make sure hover uses PD actions to stay in
                # current place.
                self.cmd_twist_convert.header.stamp = rospy.Time.now()
                (self._drone_est_pose, self.vhat,
                 self.real_yaw, measurement_valid) = self.get_pose_est()
                self.hover_setpoint.position = self._drone_est_pose.position

            if not self.state == "initialization":
                self.hover()
            rospy.sleep(0.01)

    def configure(self):
        '''Configures the controller by loading in the room and static
        obstacles.
        Sends Settings to Motionplanner.
        Settings constists of
            - environment
        Waits for Motionplanner to set mp_status to configured.
        '''

        # List containing obstacles of type Obstacle()
        Sjaaakie = Obstacle(shape=self.Sjaaakie[0:2], pose=self.Sjaaakie[2:])
        # self.obstacles = [Sjaaakie]
        self.obstacles = []
        rospy.wait_for_service("/motionplanner/config_motionplanner")
        config_success = False
        try:
            config_mp = rospy.ServiceProxy(
                "/motionplanner/config_motionplanner", ConfigMotionplanner)
            config_success = config_mp(self.obstacles)
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

        self.target_reached = False

        self._time = 0.
        self._new_trajectories = False

        self.cmd_twist_convert.header.stamp = rospy.Time.now()
        (self._drone_est_pose, self.vhat,
         self.real_yaw, measurement_valid) = self.get_pose_est()

        self.marker_setup()

        self._goal = goal
        self.fire_motionplanner()

        self._init = True
        self.startup = True

        # print magenta('---- Motionplanner goal set! ----')

    def fire_motionplanner(self):
        '''Publishes inputs to motionplanner via Trigger topic.
        '''
        self._trigger.goal_pos = self._goal
        self._trigger.goal_vel = Point()
        self._trigger.pos_state = self._drone_est_pose
        self._trigger.vel_state = self.vhat
        self._trigger.current_time = self._time
        self._mp_trigger_topic.publish(self._trigger)

    def get_mp_result(self, data):
        '''Store results of motionplanner calculations.

        Args:
            data : calculated trajectories received from 'mp_result' topic,
                   published by motionplanner.
        '''
        u_traj = data.u_traj
        v_traj = data.v_traj
        w_traj = data.w_traj
        x_traj = data.x_traj
        y_traj = data.y_traj
        z_traj = data.z_traj
        self.store_trajectories(u_traj, v_traj, w_traj, x_traj, y_traj, z_traj)

    def omg_update(self):
        '''
        - Updates the controller with newly calculated trajectories and
        velocity commands.
        - Sends out new velocity command.
        - Retrieves new pose estimate.
        '''
        # Send velocity sample.
        self.cmd_twist_convert.header.stamp = rospy.Time.now()
        self.cmd_vel.publish(self.cmd_twist_convert.twist)

        # Retrieve new pose estimate from World Model.
        # This is a pose estimate for the first following time instance [k+1]
        # if the velocity command sent above corresponds to time instance [k].
        (self._drone_est_pose,
         self.vhat, self.real_yaw, measurement_valid) = self.get_pose_est()

        # Publish pose to plot in rviz.
        self.publish_real(self._drone_est_pose.position.x,
                          self._drone_est_pose.position.y,
                          self._drone_est_pose.position.z)

        if not measurement_valid:
            self.safety_brake()
            return
        # Check for new trajectories. Trigger Motionplanner or raise
        # 'overtime'
        if self._init:
            if not self._new_trajectories:
                return
            self.omg_index = int(self._update_time/self._sample_time)
            self._init = False

        if ((self.omg_index >= int(self._update_time/self._sample_time))
                or (self.omg_index >= len(self._traj['u'])-2)):
            if self._new_trajectories:
                # Load fresh trajectories.
                self.load_trajectories()
                self._new_trajectories = False
                self._time += self.omg_index*self._sample_time
                self.pos_index = self.omg_index
                self.omg_index = 1
                # Trigger motion planner.
                self.fire_motionplanner()

            else:
                self.calc_succeeded = False
                print highlight_red('---- ! Overtime !  ----')
                self.safety_brake()
                return

        # publish current pose and velocity calculated by omg-tools
        pos = PointStamped()
        pos.header.frame_id = "world"
        pos.point = Point(x=self._traj['x'][self.omg_index + 1],
                          y=self._traj['y'][self.omg_index + 1],
                          z=self._traj['z'][self.omg_index + 1])
        vel = PointStamped()
        vel.header.frame_id = "world"
        vel.point = Point(x=self._traj['u'][self.omg_index + 1],
                          y=self._traj['v'][self.omg_index + 1],
                          z=self._traj['w'][self.omg_index + 1])

        self.publish_current_ff_vel(pos, vel)

        # Transform feedforward command from frame world to world_rotated.
        self.rotate_vel_cmd(vel)

        # Convert feedforward velocity command to angle input.
        self.convert_vel_cmd()

        # Combine feedback and feedforward commands.
        pos_fb = pos
        vel_fb = PointStamped()
        vel_fb.header.frame_id = "world"
        vel_fb.point = Point(x=self._traj['u'][self.omg_index],
                             y=self._traj['v'][self.omg_index],
                             z=self._traj['w'][self.omg_index])
        self.combine_ff_fb(pos_fb, vel_fb)

        self.omg_index += 1

    def draw_update(self, index):
        '''
        - Updates the controller with newly calculated trajectories and
        velocity commands.
        - Sends out new velocity command.
        - Retrieves new pose estimate.
        '''
        # Send velocity sample.
        self.cmd_twist_convert.header.stamp = rospy.Time.now()
        self.cmd_vel.publish(self.cmd_twist_convert.twist)

        # Retrieve new pose estimate from World Model.
        # This is a pose estimate for the first following time instance [k+1]
        # if the velocity command sent above corresponds to time instance [k].
        (self._drone_est_pose,
         self.vhat, self.real_yaw, measurement_valid) = self.get_pose_est()

        # Publish pose to plot in rviz.
        self.publish_real(self._drone_est_pose.position.x,
                          self._drone_est_pose.position.y,
                          self._drone_est_pose.position.z)

        if not measurement_valid:
            self.safety_brake()
            return

        # publish current pose and velocity calculated by omg-tools
        pos = Point(x=self.drawn_pos_x[index],
                    y=self.drawn_pos_y[index],
                    z=self.drawn_pos_z[index])
        vel = Point(x=self.drawn_vel_x[index],
                    y=self.drawn_vel_y[index],
                    z=self.drawn_vel_z[index])
        self.publish_current_ff_vel(pos, vel)

        # Transform feedforward command from frame world to world_rotated.
        self.rotate_vel_cmd(vel)

        # Convert feedforward velocity command to angle input.
        self.convert_vel_cmd()

        # Combine feedback and feedforward commands.
        pos_fb = pos
        vel_fb = Point(x=self.drawn_vel_x[index - 1],
                       y=self.drawn_vel_y[index - 1],
                       z=self.drawn_vel_z[index - 1])
        self.combine_ff_fb(pos_fb, vel_fb)

    def check_goal_reached(self):
        '''Determines whether goal is reached.
        Returns:
            not stop: boolean whether goal is reached. If not, controller
                      proceeds to goal.
        '''
        pos_nrm = np.linalg.norm(np.array([self._drone_est_pose.position.x,
                                           self._drone_est_pose.position.y,
                                           self._drone_est_pose.position.z])
                                 - np.array([self._goal.position.x,
                                             self._goal.position.y,
                                             self._goal.position.z]))

        self.target_reached = (pos_nrm < self.pos_nrm_tol)

        if self.target_reached:
            print magenta('---- Target Reached! ----')

####################
# State functions #
####################

    def switch_state(self, state):
        '''Switches state according to the general fsm state as received by
        bebop_core.
        '''
        if not (state.data == self.state):
            self.state = state.data
            self.state_changed = True
            print magenta(' Controller state changed to:', self.state)
        if self.executing_state:
            self.state_killed = True

    def hover(self):
        '''When state is equal to the standby state, drone keeps itself in same
        location through a PD controller.
        '''
        if self.airborne:
            (self._drone_est_pose,
             self.vhat, self.real_yaw, measurement_valid) = self.get_pose_est()

            # print 'pose + meas valid\n', self._drone_est_pose, '\n', measurement_valid
            if not measurement_valid:
                self.safety_brake()
                return
            pos_desired = PointStamped()
            pos_desired.point = Point(x=self.hover_setpoint.position.x,
                                      y=self.hover_setpoint.position.y,
                                      z=self.hover_setpoint.position.z)
            vel_desired = PointStamped()
            vel_desired.point = Point(x=0.0,
                                      y=0.0,
                                      z=0.0)
            feedback_cmd = self.feedback(pos_desired, vel_desired)

            self.cmd_twist_convert.twist = feedback_cmd
            self.cmd_twist_convert.header.stamp = rospy.Time.now()
            self.cmd_vel.publish(self.cmd_twist_convert.twist)

    def take_off_land(self):
        '''Function needed to wait when taking of or landing to make sure no
        control inputs are sent out.
        '''
        self.cmd_vel.publish(Twist())

        if self.state == "take-off" and not self.airborne:
            self.take_off.publish(Empty())
            rospy.sleep(3.)
            self.airborne = True
        elif self.state == "land" and self.airborne:
            rospy.sleep(0.1)
            self.land.publish(Empty())
            rospy.sleep(8.)
            self.airborne = False

    def omg_fly(self):
        '''Fly from start to end point using omg-tools as a motionplanner.
        '''
        # Preparing omg standby hover setpoint for when omgtools finishes.
        self.hover_setpoint = self._goal
        self.omg_index = 1

        while not (rospy.is_shutdown() or self.target_reached):
            if self.state_killed:
                break

            if self.startup:  # Becomes True when goal is set.
                self.omg_update()
                # Determine whether goal has been reached.
                self.check_goal_reached()
            self.rate.sleep()

        self.startup = False

    def draw_traj(self):
        '''Start building a trajectory according to the trajectory of the
        controller.
        '''
        # While loops needed to ensure that only differentiating path when path
        # is drawn and trigger button has been released.
        while not (self.draw or rospy.is_shutdown()):
            self.rate.sleep()

        while self.draw and not rospy.is_shutdown():
            self.rate.sleep()

        print magenta('----trigger button has been released,'
                      'path will be calculated----')

        # Process the drawn trajectory.
        self.low_pass_filter_drawn_traj()
        self.differentiate_traj()

    def fly_to_start(self):
        '''Sets goal equal to starting position of trajectory and triggers
        omgtools to fly the drone towards it.
        '''
        goal = Pose()
        goal.position.x = self.drawn_pos_x[0]
        goal.position.y = self.drawn_pos_y[0]
        goal.position.z = self.drawn_pos_z[0]
        self.set_omg_goal(goal)
        self.omg_fly()

    def follow_traj(self):
        '''Lets the drone fly along the drawn path.
        '''

        # Preparing hover setpoint for when trajectory is completed.
        self._goal = Pose()
        self._goal.position.x = self.drawn_pos_x[-1]
        self._goal.position.y = self.drawn_pos_y[-1]
        self._goal.position.z = self.drawn_pos_z[-1]

        self.hover_setpoint = self._goal
        self.target_reached = False

        self.cmd_twist_convert.twist = Twist()
        self.cmd_twist_convert.twist.linear.x = self.drawn_vel_x[0]
        self.cmd_twist_convert.twist.linear.y = self.drawn_vel_y[0]
        self.cmd_twist_convert.twist.linear.z = self.drawn_vel_z[0]

        index = 1
        while (not self.target_reached and (index < len(self.drawn_vel_x))
               and (not rospy.is_shutdown())):
            if self.state_killed:
                break

            self.draw_update(index)
            index += 1
            # Determine whether goal has been reached.
            self.check_goal_reached()

            self.rate.sleep()

    def drag_drone(self):
        '''Adapts hover setpoint to follow vive right controller when trigger
        is pressed.
        '''
        while not (rospy.is_shutdown() or self.state_killed):
            drag_offset = Point(
                x=(self._drone_est_pose.position.x-self.ctrl_r_pos.position.x),
                y=(self._drone_est_pose.position.y-self.ctrl_r_pos.position.y),
                z=(self._drone_est_pose.position.z-self.ctrl_r_pos.position.z))

            while (self.drag and not (
                                    self.state_killed or rospy.is_shutdown())):
                # When trigger pulled, freeze offset controller-drone and adapt
                # hover setpoint, until trigger is released.
                # print magenta('in den drag while, drag = ', self.drag)
                self.hover_setpoint.position = Point(
                    x=(self.ctrl_r_pos.position.x + drag_offset.x),
                    y=(self.ctrl_r_pos.position.y + drag_offset.y),
                    z=(self.ctrl_r_pos.position.z + drag_offset.z))
                self.hover()
                self.rate.sleep()

            self.hover()
            self.rate.sleep()

####################
# Helper functions #
####################

    def rotate_vel_cmd(self, vel):
        '''Transforms the velocity commands from the global world frame to the
        rotated world frame world_rot.
        '''
        self.feedforward_cmd.linear.x = vel.point.x
        self.feedforward_cmd.linear.y = vel.point.y
        self.feedforward_cmd.linear.z = vel.point.z
        self.feedforward_cmd = self.transform_twist(
                                    self.feedforward_cmd, "world", "world_rot")

    def convert_vel_cmd(self):
        '''Converts a velocity command to a desired input angle according to
        the state space representation of the inverse velocity model.
        '''
        u = np.array([[self.feedforward_cmd.linear.x],
                      [self.feedforward_cmd.linear.y],
                      [self.feedforward_cmd.linear.z]])
        self.X = np.matmul(self.A, self.X) + np.matmul(self.B, u)
        Y = np.matmul(self.C, self.X) + np.matmul(self.D, u)
        self.feedforward_cmd.linear.x = Y[0, 0]
        self.feedforward_cmd.linear.y = Y[1, 0]
        self.feedforward_cmd.linear.z = Y[2, 0]

    def combine_ff_fb(self, pos_desired, vel_desired):
        '''Combines the feedforward and feedback commands to generate the full
        input angle command.
        '''
        # Transform feedback desired position and velocity from world frame to
        # world_rot frame
        feedback_cmd = self.feedback(pos_desired, vel_desired)

        self.cmd_twist_convert.twist.linear.x = max(min((
                        self.feedforward_cmd.linear.x + feedback_cmd.linear.x),
                        self.max_input), - self.max_input)
        self.cmd_twist_convert.twist.linear.y = max(min((
                        self.feedforward_cmd.linear.y + feedback_cmd.linear.y),
                        self.max_input), - self.max_input)
        self.cmd_twist_convert.twist.linear.z = max(min((
                        self.feedforward_cmd.linear.z + feedback_cmd.linear.z),
                        self.max_input), - self.max_input)
        self.cmd_twist_convert.twist.angular.z = max(min((
                    self.feedforward_cmd.angular.z + feedback_cmd.angular.z),
                    self.max_input), - self.max_input)

    def feedback(self, pos_desired, vel_desired):
        '''Whenever the target is reached, apply position feedback to the
        desired end position to remain in the correct spot and compensate for
        drift.
        Tustin discretized PID controller for x and y, PI for z.
        '''
        feedback_cmd = Twist()

        # # PD
        # pos_error = PointStamped()
        # pos_error.header.frame_id = "world"
        # pos_error.point.x = pos_desired.point.x - self._drone_est_pose.position.x
        # pos_error.point.y = pos_desired.point.y - self._drone_est_pose.position.y
        # pos_error.point.z = pos_desired.point.z - self._drone_est_pose.position.z
        #
        # vel_error = PointStamped()
        # vel_error.header.frame_id = "world"
        # vel_error.point.x = vel_desired.point.x - self.vhat.x
        # vel_error.point.y = vel_desired.point.y - self.vhat.y
        #
        # pos_error = self.transform_point(pos_error, "world", "world_rot")
        # vel_error = self.transform_point(vel_error, "world", "world_rot")
        # print 'pos error', pos_error.point
        # print 'vel error\n', self.vhat, vel_error.point
        #
        # feedback_cmd.linear.x = max(- self.max_input, min(self.max_input, (
        #         self.Kp_x*pos_error.point.x +
        #         self.Kd_x*vel_error.point.x)))
        # feedback_cmd.linear.y = max(- self.max_input, min(self.max_input, (
        #         self.Kp_y*pos_error.point.y +
        #         self.Kd_y*vel_error.point.y)))
        # feedback_cmd.linear.z = max(- self.max_input, min(self.max_input, (
        #         self.Kp_z*pos_error.point.z)))

        # # PID
        pos_error_prev = self.pos_error_prev
        pos_error = PointStamped()
        pos_error.header.frame_id = "world"
        pos_error.point.x = (pos_desired.point.x
                             - self._drone_est_pose.position.x)
        pos_error.point.y = (pos_desired.point.y
                             - self._drone_est_pose.position.y)
        pos_error.point.z = (pos_desired.point.z
                             - self._drone_est_pose.position.z)

        vel_error_prev = self.vel_error_prev
        vel_error = PointStamped()
        vel_error.header.frame_id = "world"
        vel_error.point.x = vel_desired.point.x - self.vhat.x
        vel_error.point.y = vel_desired.point.y - self.vhat.y

        # print magenta('error before transform\n', pos_error)
        pos_error = self.transform_point(pos_error, "world", "world_rot")
        vel_error = self.transform_point(vel_error, "world", "world_rot")
        # print magenta('error after transform\n', pos_error)

        feedback_cmd.linear.x = max(- self.max_input, min(self.max_input, (
                self.feedback_cmd_prev.linear.x +
                (self.Kp_x + self.Ki_x*self._sample_time/2) *
                pos_error.point.x +
                (-self.Kp_x + self.Ki_x*self._sample_time/2) *
                pos_error_prev.point.x +
                self.Kd_x*(vel_error.point.x - vel_error_prev.point.x))))

        feedback_cmd.linear.y = max(- self.max_input, min(self.max_input, (
                self.feedback_cmd_prev.linear.y +
                (self.Kp_y + self.Ki_y*self._sample_time/2) *
                pos_error.point.y +
                (-self.Kp_y + self.Ki_y*self._sample_time/2) *
                pos_error_prev.point.y +
                self.Kd_y*(vel_error.point.y - vel_error_prev.point.y))))

        feedback_cmd.linear.z = max(- self.max_input, min(self.max_input, (
                self.feedback_cmd_prev.linear.z +
                (self.Kp_z + self.Ki_z*self._sample_time/2) *
                pos_error.point.z +
                (-self.Kp_z + self.Ki_z*self._sample_time/2) *
                pos_error_prev.point.z)))

        # Add theta feedback to remain at zero yaw angle
        feedback_cmd.angular.z = (
                            self.K_theta*(self.desired_yaw - self.real_yaw))

        self.pos_error_prev = pos_error
        self.vel_error_prev = vel_error
        # print magenta('* 1. feedback cmd\n'), feedback_cmd.linear
        # print magenta('* 2. feedback pos errors\n'), pos_desired.x - self._drone_est_pose.position.x
        # print magenta('* 3. vhat\n'), self.vhat
        # print '* 3. goal (pos des)\n', pos_desired
        self.feedback_cmd_prev = feedback_cmd

        return feedback_cmd

    def safety_brake(self):
        '''Brake as emergency measure: Bebop brakes automatically when
            /bebop/cmd_vel topic receives all zeros.
        '''
        print highlight_red(' Safety brake activated')
        self.cmd_twist_convert.twist = Twist()
        self.cmd_vel.publish(self.cmd_twist_convert.twist)

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
            resp = pose_est(self.cmd_twist_convert)

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
                           x_traj, y_traj, z_traj):
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
        self._traj_strg = {'u': u_traj, 'v': v_traj, 'w': w_traj,
                           'x': x_traj, 'y': y_traj, 'z': z_traj}
        self._new_trajectories = True

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
        cmd_vel.header.frame_id = "world"
        cmd_vel.point.x = twist.linear.x
        cmd_vel.point.y = twist.linear.y
        cmd_vel.point.z = twist.linear.z
        cmd_vel_rotated = self.transform_point(cmd_vel, _from, _to)

        twist_rotated = Twist()
        twist_rotated.linear.x = cmd_vel_rotated.point.x
        twist_rotated.linear.y = cmd_vel_rotated.point.y
        twist_rotated.linear.z = cmd_vel_rotated.point.z

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
        if self.draw:
            self.draw_ctrl_path()

    def r_trigger(self, button_pushed):
        '''When button is pushed on the right hand controller, depending on the
        current state, either sets goal to be equal to the position of the
        controller, or allows path that the controller describes to be saved.
        '''

        if (((self.state == "omg fly") or (self.state == "omg standby"))
           and button_pushed.data):
            self.target_reached = False
            goal = Pose()
            goal.position.x = self.ctrl_r_pos.position.x
            goal.position.y = self.ctrl_r_pos.position.y
            goal.position.z = self.ctrl_r_pos.position.z
            self.set_omg_goal(goal)

        elif self.state == "draw path":
            # Start drawing and saving path
            if (button_pushed.data and not self.draw):
                self.drawn_pos_x = []
                self.drawn_pos_y = []
                self.drawn_pos_z = []
                self.draw = True
                print highlight_green('---- Start drawing path while keeping'
                                      ' trigger pushed ----')

            if (not button_pushed.data and self.draw):
                self.draw = False

        elif self.state == "drag drone":
            if (button_pushed.data and not self.drag):
                print magenta('trigger, drag op true')
                self.drag = True
            elif (not button_pushed.data and self.drag):
                self.drag = False

    def differentiate_traj(self):
        '''Differentiate obtained trajectory to obtain feedforward velocity
        commands.
        '''
        max_vel_ok = False
        self.interpolate_list()

        while not (rospy.is_shutdown() or max_vel_ok):
            self.drawn_vel_x = np.diff(self.drawn_pos_x)/self._sample_time
            self.drawn_vel_y = np.diff(self.drawn_pos_y)/self._sample_time
            self.drawn_vel_z = np.diff(self.drawn_pos_z)/self._sample_time

            max_vel_ok = (
                    all(vel <= self.max_vel for vel in self.drawn_vel_x) and
                    all(vel <= self.max_vel for vel in self.drawn_vel_y) and
                    all(vel <= self.max_vel for vel in self.drawn_vel_z))

            # Check if max velocity in list is not above max possible velocity.
            if not max_vel_ok:
                self.interpolate_list()

    def interpolate_list(self):
        '''Linearly interpolates a list so that it contains twice the number of
        elements.
        '''
        drawn_pos_x_interp = (2*len(self.drawn_pos_x)-1)*[0]
        drawn_pos_y_interp = (2*len(self.drawn_pos_y)-1)*[0]
        drawn_pos_z_interp = (2*len(self.drawn_pos_z)-1)*[0]

        drawn_pos_x_interp[0::2] = self.drawn_pos_x
        drawn_pos_y_interp[0::2] = self.drawn_pos_y
        drawn_pos_z_interp[0::2] = self.drawn_pos_z

        drawn_pos_x_interp[1::2] = [elem / 2.0 for elem in
                                    [a + b for a, b in
                                     zip(self.drawn_pos_x[:-1],
                                         self.drawn_pos_x[1:])]]
        drawn_pos_y_interp[1::2] = [elem / 2.0 for elem in
                                    [a + b for a, b in
                                     zip(self.drawn_pos_y[:-1],
                                         self.drawn_pos_y[1:])]]
        drawn_pos_z_interp[1::2] = [elem / 2.0 for elem in
                                    [a + b for a, b in
                                     zip(self.drawn_pos_z[:-1],
                                         self.drawn_pos_z[1:])]]

        self.drawn_pos_x = drawn_pos_x_interp
        self.drawn_pos_y = drawn_pos_y_interp
        self.drawn_pos_z = drawn_pos_z_interp

    def low_pass_filter_drawn_traj(self):
        '''Low pass filter the trajectory drawn with the controller in order to
        be suitable for the drone to track it.
        '''
        self.drawn_pos_x = lfilter(
            self.butter_b, self.butter_a, self.drawn_pos_x)
        self.drawn_pos_y = lfilter(
            self.butter_b, self.butter_a, self.drawn_pos_y)
        self.drawn_pos_z = lfilter(
            self.butter_b, self.butter_a, self.drawn_pos_z)

        # Plot the smoothed trajectory in Rviz.
        self.draw_smoothed_path()

#######################################
# Functions for plotting Rviz markers #
#######################################

    def marker_setup(self):
        '''Setup markers to display the desired and real path of the drone in
        rviz, along with the current position in the omg-tools generated
        position list.
        '''
        # Desired path
        self._desired_path = Marker()
        self._desired_path.header.frame_id = 'world'
        self._desired_path.ns = "trajectory_desired"
        self._desired_path.id = 0
        self._desired_path.type = 4  # Line List.
        self._desired_path.action = 0
        self._desired_path.scale.x = 0.05
        self._desired_path.scale.y = 0.05
        self._desired_path.scale.z = 0.0
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
        self._real_path.scale.x = 0.05
        self._real_path.scale.y = 0.05
        self._real_path.scale.z = 0.0
        self._real_path.color.r = 0.0
        self._real_path.color.g = 1.0
        self._real_path.color.b = 0.0
        self._real_path.color.a = 1.0
        self._real_path.lifetime = rospy.Duration(0)

        # omg-tools position and velocity
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

        # Obstacle
        self.rviz_obst = Marker()
        self.rviz_obst.header.frame_id = 'world'
        self.rviz_obst.ns = "obstacle"
        self.rviz_obst.id = 3
        self.rviz_obst.type = 3  # Cylinder
        self.rviz_obst.action = 0
        self.rviz_obst.scale.x = self.Sjaaakie[0] * 2  # x-diameter
        self.rviz_obst.scale.y = self.Sjaaakie[0] * 2  # y-diameter
        self.rviz_obst.scale.z = self.Sjaaakie[1]  # height
        self.rviz_obst.pose.orientation.w = 1.0
        self.rviz_obst.color.r = 1.0
        self.rviz_obst.color.g = 1.0
        self.rviz_obst.color.b = 1.0
        self.rviz_obst.color.a = 0.5
        self.rviz_obst.lifetime = rospy.Duration(0)

        # Controller drawn path
        self.drawn_path = Marker()
        self.drawn_path.header.frame_id = 'world'
        self.drawn_path.ns = "drawn_path"
        self.drawn_path.id = 4
        self.drawn_path.type = 4  # Line List.
        self.drawn_path.action = 0
        self.drawn_path.scale.x = 0.05
        self.drawn_path.scale.y = 0.05
        self.drawn_path.scale.z = 0.0
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
        self.smooth_path.scale.x = 0.05
        self.smooth_path.scale.y = 0.05
        self.smooth_path.scale.z = 0.0
        self.smooth_path.color.r = 1.0
        self.smooth_path.color.g = 0.38
        self.smooth_path.color.b = 0.0
        self.smooth_path.color.a = 1.0
        self.smooth_path.lifetime = rospy.Duration(0)

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
        point_end = Point(x=(pos.point.x + 2.5*vel.point.x),
                          y=(pos.point.y + 2.5*vel.point.y),
                          z=(pos.point.z + 2.5*vel.point.z))
        self.current_ff_vel.points = [point_start, point_end]

        self.current_ff_vel_pub.publish(self.current_ff_vel)

    def publish_obst(self, empty):
        '''Publish static obstacles.
        '''
        self.rviz_obst.header.stamp = rospy.get_rostime()

        point = Point(
                    x=self.Sjaaakie[2], y=self.Sjaaakie[3], z=self.Sjaaakie[4])
        self.rviz_obst.pose.position = point

        self.obst_pub.publish(self.rviz_obst)

    def draw_ctrl_path(self):
        '''Publish real x and y trajectory to topic for visualisation in
        rviz.
        '''
        self.drawn_path.header.stamp = rospy.get_rostime()

        point = Point(x=self.ctrl_r_pos.position.x,
                      y=self.ctrl_r_pos.position.y,
                      z=self.ctrl_r_pos.position.z)
        self.drawn_path.points.append(point)

        self.drawn_pos_x += [point.x]
        self.drawn_pos_y += [point.y]
        self.drawn_pos_z += [point.z]

        self.trajectory_desired.publish(self.drawn_path)

    def draw_smoothed_path(self):
        '''Publish the smoothed x and y trajectory to topic for visualisation
        in rviz.
        '''
        self.smooth_path.header.stamp = rospy.get_rostime()
        self.smooth_path.points = []

        for index in len(self.drawn_pos_x):
            point = Point(x=self.drawn_pos_x[i],
                          y=self.drawn_pos_y[i],
                          z=self.drawn_pos_z[i])
            self.smooth_path.points.append(point)

        self.trajectory_smoothed.publish(self.smooth_path)


if __name__ == '__main__':
    vel_command = VelCommander()
    vel_command.start()
