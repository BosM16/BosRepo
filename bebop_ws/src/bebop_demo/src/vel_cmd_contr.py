#!/usr/bin/env python

from geometry_msgs.msg import (Twist, TwistStamped, Point, PointStamped,
                               Pose, PoseStamped)
from std_msgs.msg import Bool, Empty, String
from visualization_msgs.msg import Marker, MarkerArray

from bebop_demo.msg import Trigger, Trajectories, Obstacle

from bebop_demo.srv import GetPoseEst, ConfigMotionplanner

import rospy
import numpy as np
import tf
import tf2_ros
import tf2_geometry_msgs as tf2_geom
import time


class VelCommander(object):

    def __init__(self):
        """Initialization of Controller object.

        Args:
            sample_time : Period between computed velocity samples.
            update_time : Period of problem solving.
        """
        rospy.init_node("vel_commander_node")

        self.calc_succeeded = False
        self.target_reached = False
        self.startup = False
        self._index = 1
        self.state = "initialization"
        self.state_dict = {"standby": self.hover,
                           "take-off": self.take_off_land,
                           "land": self.take_off_land,
                           "omg standby": self.omg_standby,
                           "omg fly": self.omg_fly,
                           "draw path": self.draw_traj,
                           "fly to start": self.fly_to_start,
                           "place window obstacles": self.place_window_obst,
                           "follow path": self.follow_traj}
        self.state_changed = False
        self.executing_state = False
        self.state_killed = False

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
        self.room_width = rospy.get_param('motionplanner/room_width', 1.)
        self.room_depth = rospy.get_param('motionplanner/room_depth', 1.)
        self.room_height = rospy.get_param('motionplanner/room_height', 1.)
        # self.angle_nrm_tol = rospy.get_param(
        #                                 'vel_cmd/goal_reached_angle_tol', 0.05)

        self._sample_time = rospy.get_param('vel_cmd/sample_time', 0.01)
        self._update_time = rospy.get_param('vel_cmd/update_time', 0.5)
        self.rate = rospy.Rate(1./self._sample_time)

        self.X = np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])
        self.desired_yaw = 0.0
        self.real_yaw = 0.0
        self.pos_nrm = np.inf
        self.x_error = 0.
        self.y_error = 0.
        self.z_error = 0.
        self.measurement_valid = False
        self.safe = False
        self._goal = Pose()
        self.hover_setpoint = Pose()
        self.ctrl_r_pos = Pose()

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
        self.current_ff_vel_pub = rospy.Publisher(
            'motionplanner/current_ff_vel', Marker, queue_size=1)
        self.obst_pub = rospy.Publisher(
            'motionplanner/rviz_obst', MarkerArray, queue_size=1)
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
        print '-----------------------------------------'
        print '- Controller & Motionplanner Configured -'
        print '-        Velocity Control Started       -'
        print '-----------------------------------------'

        while not rospy.is_shutdown():
            if self.state_changed:
                self.state_changed = False

                self.executing_state = True
                # Execute state function.
                self.state_dict[self.state]()
                self.executing_state = False

                # State has not finished when it has been killed!
                if not self.state_killed:
                    self.ctrl_state_finish.publish(Empty())
                    print 'PUBLISH FINISHED'
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
            print "Service call failed: %s" % e
            config_success = False

        rospy.Subscriber('motionplanner/goal', Pose, self.set_goal)

        return config_success

    def set_goal(self, goal):
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

        print '-----------------------'
        print 'Motionplanner goal set!'
        print '-----------------------'

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

    def update(self):
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
        # old_pose = self._drone_est_pose
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
            self._index = int(self._update_time/self._sample_time)
            self._init = False

        if ((self._index >= int(self._update_time/self._sample_time))
                or (self._index >= len(self._traj['u'])-2)):
            if self._new_trajectories:
                # Load fresh trajectories.
                self.load_trajectories()
                self._new_trajectories = False
                self._time += self._index*self._sample_time
                self.pos_index = self._index
                self._index = 1
                # Trigger motion planner.
                self.fire_motionplanner()

            else:
                self.calc_succeeded = False
                print '-- !! -------- !! --'
                print '-- !! Overtime !! --'
                self.safety_brake()
                return

        # publish current pose and velocity calculated by omg-tools
        self.publish_current_ff_vel()

        # Transform feedforward command from frame world to world_rotated.
        self.rotate_vel_cmd()

        # Convert feedforward velocity command to angle input.
        self.convert_vel_cmd()

        # Combine feedback and feedforward commands.
        self.combine_ff_fb()

        self._index += 1

    def proceed(self):
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
            print '-------------------'
            print '- Target Reached! -'
            print '-------------------'

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
            print "controller state changed to:", self.state
        if self.executing_state:
            self.state_killed = True

    def hover(self):
        '''When state is equal to the standby state, drone keeps itself in same
        location through a PD controller.
        '''
        (self._drone_est_pose,
         self.vhat, self.real_yaw, measurement_valid) = self.get_pose_est()
        if not measurement_valid:
            self.safety_brake()
            return
        pos_desired = Point(x=self.hover_setpoint.position.x,
                            y=self.hover_setpoint.position.y,
                            z=self.hover_setpoint.position.z)
        vel_desired = Point(x=0.0,
                            y=0.0,
                            z=0.0)
        feedback_cmd = self.transform_twist(
            self.feedback(pos_desired, vel_desired), "world", "world_rot")

        self.cmd_twist_convert.twist = feedback_cmd
        self.cmd_twist_convert.header.stamp = rospy.Time.now()
        self.cmd_vel.publish(self.cmd_twist_convert.twist)

    def omg_standby(self):
        '''As long as no goal has been set, remain at current position through
        the use of Pd control.
        '''
        self.hover()

    def take_off_land(self):
        '''Function needed to wait when taking of or landing to make sure no
        control inputs are sent out.
        '''
        self.cmd_vel.publish(Twist())

        if self.state == "take-off":
            self.take_off.publish(Empty())
            rospy.sleep(3.)
        elif self.state == "land":
            self.land.publish(Empty())
            rospy.sleep(8.)

    def omg_fly(self):
        '''Fly from start to end point using omg-tools as a motionplanner.
        '''
        # Preparing omg standby hover setpoint for when omgtools finishes.
        self.hover_setpoint = self._goal

        while not self.target_reached:
            if self.state_killed == True:
                break

            if self.startup:  # Becomes True when goal is set.
                self.update()
                # Determine whether goal has been reached.
                self.proceed()
            self.rate.sleep()

        self.startup = False

    def draw_traj(self):
        '''Start building a trajectory according to the trajectory of the
        controller.
        '''
        while self.draw:
            self.draw_ctrl_path()
            rospy.rate.sleep() # NOT CORRECT RATE!!! should be placed in location function.

        print ('----trigger button has been released,'
               'path will be calculated----')
        self.differentiate_traj()

    def fly_to_start(self):
        '''Sets goal equal to starting position of trajectory and triggers
        omgtools to fly the drone towards it.
        '''
        goal = Pose()
        goal.position.x = self.drawn_pos_x[-1]
        goal.position.y = self.drawn_pos_y[-1]
        goal.position.z = self.drawn_pos_z[-1]
        self.set_goal(goal)
        self.omg_fly()

    def follow_traj(self):
        '''Lets the drone fly along the drawn path.
        '''
        # Gekopieerd uit andere branch, moet nog helemaal herwerkt worden!
        while self.progress: # While not list of velocities at end.
            if self.state_change:
                self.state_killed = True
                break
            self.update()
            # Determine whether goal has been reached.
            self.progress = self.proceed()
            self.rate.sleep()

    def place_window_obst(self):
        '''Place a rectangular window obstacle by defining the center and the
        upper right corner. Windows can only be placed perpendicular to the
        x-direction.
        '''
        self.obstacles = []
        self.publish_obst_room(Empty)

        print (
            ' Drag left controller to place obstacle ')

        for _ in range(0, 4):
            self.obstacles.append(None)
        corner1 = Point(x=0,
                        y=-0.5,
                        z=1.8)
        corner2 = Point(x=0,
                        y=0.3,
                        z=1.4)
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

        self.obstacles[-4] = plate1
        self.obstacles[-3] = plate2
        self.obstacles[-2] = plate3
        self.obstacles[-1] = plate4
        self.publish_obst_room(Empty)
        print (' Obstacle added ')


####################
# Helper functions #
####################

    def rotate_vel_cmd(self):
        '''Transforms the velocity commands from the global world frame to the
        rotated world frame world_rot.
        '''
        self.feedforward_cmd.linear.x = self._traj['u'][self._index + 1]
        self.feedforward_cmd.linear.y = self._traj['v'][self._index + 1]
        self.feedforward_cmd.linear.z = self._traj['w'][self._index + 1]
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

    def combine_ff_fb(self):
        '''Combines the feedforward and feedback commands to generate the full
        input angle command.
        '''
        pos_desired = Point(x=self._traj['x'][self._index + 1],
                            y=self._traj['y'][self._index + 1],
                            z=self._traj['z'][self._index + 1])
        vel_desired = Point(x=self._traj['u'][self._index + 1],
                            y=self._traj['v'][self._index + 1],
                            z=self._traj['w'][self._index + 1])
        # Transform desired position and velocity from world frame to
        # world_rot frame
        feedback_cmd = self.transform_twist(
                self.feedback(pos_desired, vel_desired), "world", "world_rot")

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
        Lead compensator/controller?
        '''
        feedback_cmd = Twist()

        feedback_cmd.linear.x = (
                self.Kp_x*(pos_desired.x - self._drone_est_pose.position.x) +
                self.Kd_x*(vel_desired.x - self.vhat.x))
        feedback_cmd.linear.y = (
                self.Kp_y*(pos_desired.y - self._drone_est_pose.position.y) +
                self.Kd_y*(vel_desired.y - self.vhat.y))
        feedback_cmd.linear.z = (
                self.Kp_z*(pos_desired.z - self._drone_est_pose.position.z))

        # Add theta feedback to remain at zero yaw angle
        feedback_cmd.angular.z = (
                            self.K_theta*(self.desired_yaw - self.real_yaw))

        return feedback_cmd

    def safety_brake(self):
        '''Brake as emergency measure: Bebop brakes automatically when
            /bebop/cmd_vel topic receives all zeros.
        '''
        self.cmd_twist_convert.twist = Twist()
        self.cmd_vel.publish(self.cmd_twist_convert.twist)

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
            print "Service call failed: %s" % e
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
        print '--------------- NEW TRAJECTORIES AVAILABLE ---------------'

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

    def r_trigger(self, button_pushed):
        '''When button is pushed on the right hand controller, depending on the
        current state, either sets goal to be equal to the position of the
        controller, or allows path that the controller describes to be saved.
        '''

        if ((self.state == "omg fly") or (self.state == "omg standby")):
            self.target_reached = False
            goal = Pose()
            goal.position.x = self.ctrl_r_pos.position.x
            goal.position.y = self.ctrl_r_pos.position.y
            goal.position.z = self.ctrl_r_pos.position.z
            self.set_goal(goal)

        elif self.state == "draw path":
            # Start drawing and saving path
            if button_pushed.data:
                self.draw = True
                print '----start drawing path while keeping trigger pushed----'

            # Stop drawing, fly to first point in list with PD controller and
            # then follow drawn trajectory. Feedforward velocities generated by
            # differentiating velocities. Is it necessary to check max vel?
            # Smoothing of path?
            else:
                self.draw = False

    def differentiate_traj(self):
        '''Differentiate obtained trajectory to obtain feedforward velocity
        commands.
        '''
        max_vel_ok = False
        self.interpolate_list()

        while not max_vel_ok:
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
        drawn_pos_x_interp = np.zeros((1, 2*len(self.drawn_pos_x)-1))
        drawn_pos_y_interp = np.zeros((1, 2*len(self.drawn_pos_y)-1))
        drawn_pos_z_interp = np.zeros((1, 2*len(self.drawn_pos_z)-1))

        drawn_pos_x_interp[0::2] = self.drawn_pos_x
        drawn_pos_y_interp[0::2] = self.drawn_pos_y
        drawn_pos_z_interp[0::2] = self.drawn_pos_z

        drawn_pos_x_interp[1::2] = (self.drawn_pos_x[:-1] +
                                    self.drawn_pos_x[1:]) // 2
        drawn_pos_y_interp[1::2] = (self.drawn_pos_y[:-1] +
                                    self.drawn_pos_y[1:]) // 2
        drawn_pos_z_interp[1::2] = (self.drawn_pos_z[:-1] +
                                    self.drawn_pos_z[1:]) // 2

        self.drawn_pos_x = drawn_pos_x_interp
        self.drawn_pos_y = drawn_pos_y_interp
        self.drawn_pos_z = drawn_pos_z_interp

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
        self.rviz_obst = MarkerArray()

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
        self.drawn_path.color.r = 0.5
        self.drawn_path.color.g = 0.5
        self.drawn_path.color.b = 0.5
        self.drawn_path.color.a = 1.0
        self.drawn_path.lifetime = rospy.Duration(0)

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

    def publish_current_ff_vel(self):
        '''Publish current omg-tools velocity input vector where origin of the
        vector is equal to current omg-tools position.
        '''
        self.current_ff_vel.header.stamp = rospy.get_rostime()

        x_pos = self._traj['x'][self._index]
        y_pos = self._traj['y'][self._index]
        z_pos = self._traj['z'][self._index]
        x_vel = self._traj['u'][self._index]
        y_vel = self._traj['v'][self._index]
        z_vel = self._traj['w'][self._index]

        point_start = Point(x=x_pos, y=y_pos, z=z_pos)
        point_end = Point(x=(x_pos + 2.5*x_vel),
                          y=(y_pos + 2.5*y_vel),
                          z=(z_pos + 2.5*z_vel))
        self.current_ff_vel.points = [point_start, point_end]

        self.current_ff_vel_pub.publish(self.current_ff_vel)

    def publish_obst(self, empty):
        '''Publish static obstacles.
        '''
        # self.rviz_obst.header.stamp = rospy.get_rostime()
        #
        # point = Point(
        #             x=self.Sjaaakie[2], y=self.Sjaaakie[3], z=self.Sjaaakie[4])
        # self.rviz_obst.pose.position = point
        #
        # self.obst_pub.publish(self.rviz_obst)

    def draw_ctrl_path(self):
        '''Publish real x and y trajectory to topic for visualisation in
        rviz.
        '''
        self.drawn_path.header.stamp = rospy.get_rostime()

        point = Point(x=self.ctrl_r_pos.position.x,
                      y=self.ctrl_r_pos.position.y,
                      z=self.ctrl_r_pos.position.z)
        self.drawn_path.points.append(point)

        self.drawn_pos_x = []
        self.drawn_pos_y = []
        self.drawn_pos_z = []
        self.drawn_pos_x += [point.x]
        self.drawn_pos_y += [point.y]
        self.drawn_pos_z += [point.z]

        self.trajectory_real.publish(self.drawn_path)

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
        for i, obstacle in enumerate(self.obstacles):
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
                print 'in de elif marker stuff'
                obstacle_marker = Marker()
                obstacle_marker.header.frame_id = 'world'
                obstacle_marker.ns = "obstacles"
                obstacle_marker.id = i+j+7
                obstacle_marker.action = 0
                obstacle_marker.color.r = 0.0
                obstacle_marker.color.g = 1.0
                obstacle_marker.color.b = 0.0
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
                    print 'plate 1'
                    left = obstacle
                    up = self.obstacles[i+2]
                    down = self.obstacles[i+3]
                    print left, up, down
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
                    print 'plate 2'
                    right = obstacle
                    up = self.obstacles[i+1]
                    down = self.obstacles[i+2]
                    print right, up, down
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
                    print 'plate 3'
                    left = self.obstacles[i-2]
                    right = self.obstacles[i-1]
                    up = obstacle
                    print left, right, up
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
                    print 'plate 4'
                    left = self.obstacles[i-3]
                    right = self.obstacles[i-2]
                    down = obstacle
                    print left, right, down
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
                print 'append de marker'
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

                if obstacle.obst_type.data == 'inf_cylinder':
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

        print 'publish de marker'
        self.obst_pub.publish(self.rviz_obst)


if __name__ == '__main__':
    vel_command = VelCommander()
    vel_command.start()
