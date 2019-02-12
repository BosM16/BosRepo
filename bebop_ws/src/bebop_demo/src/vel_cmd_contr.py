#!/usr/bin/env python

from geometry_msgs.msg import (Twist, TwistStamped, Point, PointStamped,
                               Pose, PoseStamped)
from std_msgs.msg import Bool, Empty
from visualization_msgs.msg import Marker

from bebop_demo.msg import Trigger, Trajectories, Obstacle

from bebop_demo.srv import GetPoseEst, ConfigMotionplanner

import rospy
import numpy as np
import tf
import tf2_ros
import tf2_geometry_msgs as tf2_geom


class VelCommander(object):

    def __init__(self):
        """Initialization of Controller object.

        Args:
            sample_time : Period between computed velocity samples.
            update_time : Period of problem solving.
        """
        rospy.init_node("vel_commander_node")

        self.calc_succeeded = False
        self.progress = True
        self.startup = False
        self.target_reached = False
        self._index = 1

        self.K_x = rospy.get_param('vel_cmd/K_x', 0.6864)
        self.K_z = rospy.get_param('vel_cmd/K_z', 0.5)
        self.K_v = rospy.get_param('vel_cmd/K_v', 1.5792)
        self.K_theta = rospy.get_param('vel_cmd/K_theta', 0.3)
        self.safety_treshold = rospy.get_param('vel_cmd/safety_treshold', 0.5)
        self.pos_nrm_tol = rospy.get_param(
                                        'vel_cmd/goal_reached_pos_tol', 0.05)
        # self.angle_nrm_tol = rospy.get_param(
        #                                 'vel_cmd/goal_reached_angle_tol', 0.05)
        self.input_nrm_tol = rospy.get_param(
                                        'vel_cmd/goal_reached_input_tol', 0.03)

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

        self.cmd_twist_convert = TwistStamped()
        self.cmd_twist_convert.header.frame_id = "world_rot"
        self.cmd_twist_convert.header.stamp = rospy.Time.now()
        self.feedforward_cmd = Twist()
        self.vhat = Point()
        self._trigger = Trigger()

        # Obstacle setup
        self.Sjaaakie = [0.35, 1.5, 1.5, 1.5, 0.75]

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
        self.omg_vel_pub = rospy.Publisher(
            'motionplanner/omg_vel', Marker, queue_size=1)
        self.obst_pub = rospy.Publisher(
            'motionplanner/rviz_obst', Marker, queue_size=1)

        rospy.Subscriber('motionplanner/result', Trajectories,
                         self.get_mp_result)
        rospy.Subscriber('vive_localization/ready', Empty, self.publish_obst)

    def initialize_vel_model(self):
        '''Initializes model parameters for conversion of desired velocities to
        angle inputs.
        State space model x[k+1] = A*x[k] + B*u[k] in observable canonical
        form, corresponding to discrete time transfer function

                   b1*z + b0
        G(z) = -----------------
                z^2 + a1*z + a0

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
        self.obstacles = [Sjaaakie]
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

    def start(self):
        '''Configures,
        Starts the controller's periodical loop.
        '''
        rate = self.rate
        configured = self.configure()
        print '-----------------------------------------'
        print '- Controller & Motionplanner Configured -'
        print '-        Velocity Control Started       -'
        print '-----------------------------------------'

        while not rospy.is_shutdown():
            while (self.progress):
                if (self.startup):
                    self.update()
                    self.progress = self.proceed()
                rate.sleep()
            (self._drone_est_pose,
             self.vhat, self.real_yaw, measurement_valid) = self.get_pose_est()
            pos_desired = Point(x=self._goal.position.x,
                                y=self._goal.position.y,
                                z=self._goal.position.z)
            vel_desired = Point(x=0.0,
                                y=0.0,
                                z=0.0)
            feedback_cmd = self.transform_twist(
                self.feedback(pos_desired, vel_desired), "world", "world_rot")
            self.cmd_twist_convert.twist = feedback_cmd
            self.cmd_twist_convert.header.stamp = rospy.Time.now()
            self.cmd_vel.publish(self.cmd_twist_convert.twist)
            rate.sleep()

    def set_goal(self, goal):
        '''Sets the goal and fires motionplanner.
        Args:
            goal: Pose
        '''
        self.target_reached = False
        self._goal = goal

        self.progress = True
        self._time = 0.
        self._new_trajectories = False

        self.cmd_twist_convert.header.stamp = rospy.Time.now()
        self._drone_est_pose = self.get_pose_est()[0]
        self.marker_setup()

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
        self.publish_omg_vel()

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
        stop_linear = True
        stop = True

        pos_nrm = np.linalg.norm(np.array([self._drone_est_pose.position.x,
                                           self._drone_est_pose.position.y,
                                           self._drone_est_pose.position.z])
                                 - np.array([self._goal.position.x,
                                             self._goal.position.y,
                                             self._goal.position.z]))

        input_nrm = np.linalg.norm(
            np.array([self.cmd_twist_convert.twist.linear.x,
                     self.cmd_twist_convert.twist.linear.y,
                     self.cmd_twist_convert.twist.linear.z]))

        stop_linear = (pos_nrm < self.pos_nrm_tol) and (
                            input_nrm < self.input_nrm_tol)

        if (stop_linear):
            self.target_reached = True
            print '-------------------'
            print '- Target Reached! -'
            print '-------------------'
        stop *= (stop_linear)

        return not stop

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
        the state space representation of the inverse velocity model:


        R: DIT MOET WEG, HEEFT HET IZN OM STATE SPACE REPRES TE ZETTEN? NIET ECHT DENK IK
        - for second order velocity/input relation (x and y):
            j[k+1] = 1/b1*{ -b0*j[k] + a0*v[k] + a1*v[k+1] + v[k+2] }
                   = 1/b1*(-b0, a0, a1, 1)*(j[k], v[k], v[k+1], v[k+2])'

        (- for first order velocity/input relation (z):
            j[k+1] = 1/b0*{ a0*v[k+1] + v[k+2] } (later, 3d flight))

        where j is the input signal applied to the bebop and v the desired
        velocity.
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

        self.cmd_twist_convert.twist.linear.x = (
                        self.feedforward_cmd.linear.x + feedback_cmd.linear.x)
        self.cmd_twist_convert.twist.linear.y = (
                        self.feedforward_cmd.linear.y + feedback_cmd.linear.y)
        self.cmd_twist_convert.twist.linear.z = (
                        self.feedforward_cmd.linear.z + feedback_cmd.linear.z)
        self.cmd_twist_convert.twist.angular.z = (
                    self.feedforward_cmd.angular.z + feedback_cmd.angular.z)

    def feedback(self, pos_desired, vel_desired):
        '''Whenever the target is reached, apply position feedback to the
        desired end position to remain in the correct spot and compensate for
        drift.
        Lead compensator/controller?
        '''
        feedback_cmd = Twist()
        feedback_cmd.linear.x = (
                self.K_x*(pos_desired.x - self._drone_est_pose.position.x) +
                self.K_v*(vel_desired.x - self.vhat.x))
        feedback_cmd.linear.y = (
                self.K_x*(pos_desired.y - self._drone_est_pose.position.y) +
                self.K_v*(vel_desired.y - self.vhat.y))
        feedback_cmd.linear.z = (
                self.K_z*(pos_desired.z - self._drone_est_pose.position.z))

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
        self.omg_vel = Marker()
        self.omg_vel.header.frame_id = 'world'
        self.omg_vel.ns = "omg_vel"
        self.omg_vel.id = 3
        self.omg_vel.type = 0  # Arrow
        self.omg_vel.action = 0
        self.omg_vel.scale.x = 0.06  # shaft diameter
        self.omg_vel.scale.y = 0.1  # head diameter
        self.omg_vel.scale.z = 0.15  # head length
        self.omg_vel.color.r = 0.0
        self.omg_vel.color.g = 0.0
        self.omg_vel.color.b = 1.0
        self.omg_vel.color.a = 1.0
        self.omg_vel.lifetime = rospy.Duration(0)

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

    def publish_omg_vel(self):
        '''Publish current omg-tools velocity input vector where origin of the
        vector is equal to current omg-tools position.
        '''
        self.omg_vel.header.stamp = rospy.get_rostime()

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
        self.omg_vel.points = [point_start, point_end]

        self.omg_vel_pub.publish(self.omg_vel)

    def publish_obst(self, empty):
        '''Publish static obstacles.
        '''
        self.rviz_obst.header.stamp = rospy.get_rostime()

        point = Point(
                    x=self.Sjaaakie[2], y=self.Sjaaakie[3], z=self.Sjaaakie[4])
        self.rviz_obst.pose.position = point

        self.obst_pub.publish(self.rviz_obst)


if __name__ == '__main__':
    vel_command = VelCommander()
    vel_command.start()
