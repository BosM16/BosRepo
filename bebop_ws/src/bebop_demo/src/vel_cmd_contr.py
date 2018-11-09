#!/usr/bin/env python

from geometry_msgs.msg import Twist, TwistStamped, Point, Pose2D
from std_msgs.msg import Bool, Empty
from bebop_demo.srv import GetPoseEst
from visualization_msgs.msg import MarkerArray

from bebop_demo.msg import (
    Trigger, Trajectory, Pose2D, Obstacle, Room, Settings)

import rospy
import numpy as np


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
        self._mp_status = False

        self.origin = Pose2D()
        self.origin.x = 3.0
        self.origin.y = 1.5
        self.room.width = 6.0  # width (x-axis)
        self.room.height = 3.0  # heighth (y-axis)

        self.feedback_gain = rospy.get_param('vel_cmd/feedback_gain', 0.3)

        self.pos_nrm_tol = rospy.get_param(
                                        'vel_cmd/goal_reached_pos_tol', 0.05)
        self.angle_nrm_tol = rospy.get_param(
                                        'vel_cmd/goal_reached_angle_tol', 0.05)
        self.input_nrm_tol = rospy.get_param(
                                        'vel_cmd/goal_reached_input_tol', 0.03)

        self._sample_time = rospy.get_param('vel_cmd/sample_time', 0.01)
        self._update_time = rospy.get_param('vel_cmd/update_time', 0.5)
        self.rate = rospy.Rate(1./self._sample_time)

        self.pos_nrm = np.inf

        self._cmd_twist = TwistStamped()
        self.cmd_twist_convert = Twist()
        self._cmd_twist.header.frame_id = "world_rot"
        self._trigger = Trigger()

        # Marker setup
        self.marker_setup()

        # Coefficients for inverted model of velocity to input angle
        # X-direction
        self.input_old_x = 0.

        b0 = 0.009437
        b1 = -0.007635
        a0 = 0.9459
        a1 = -1.946

        self.coeffs_x = np.array([-b0, a0, a1, 1])/b1

        # Y-direction
        self.input_old_y = 0.

        b0 = 0.0177
        b1 = -0.01557
        a0 = 0.9338
        a1 = -1.933

        self.coeffs_y = np.array([-b0, a0, a1, 1])/b1

        # # Z-direction
        # b0 = 0.05301
        # a0 = -0.946
        #
        # self.coeffs_x = np.array([a0, 1])/b0

        self._robot_est_pose = Point()
        self._robot_est_pose.x = 0.
        self._robot_est_pose.y = 0.
        self._robot_est_pose.z = 0.

        self._robobst = []
        self._robobst_est_pose = [[0., 0.] for k in range(len(self._robobst))]
        self._robobst_est_velocity = [[0., 0.] for k in range(
            len(self._robobst))]

        self._traj = {'v': [0.0], 'w': [0.0], 'x': [0.0], 'y': [0.0]}
        self._traj_strg = {'v': [0.0], 'w': [0.0], 'x': [0.0], 'y': [0.0]}
        self._inputs_applied = {'jx': [], 'jy': []}

        self.cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self._mp_trigger_topic = rospy.Publisher(
            'mp_trigger', Trigger, queue_size=1)
        self._mp_configure_topic = rospy.Publisher(
            'mp_configure', Settings, queue_size=1)
        self.trajectory_desired = rospy.Publisher(
            'desired_path', Marker, queue_size=1)
        self.trajectory_real = rospy.Publisher(
            'real_path', Marker, queue_size=1)

        rospy.Subscriber('demo', Pose2D, self.planning)
        rospy.Subscriber('mp_result', RobotTrajectory, self.get_mp_result)
        rospy.Subscriber('mp_feedback', Bool, self.get_mp_feedback)

        # markers
        self.trajectory_marker = rospy.Publisher(
            'motionplanner_traj_marker', Marker, queue_size=1)

    def start(self):
        """Configures,
        Starts the controller's periodical loop.
        """
        rate = self.rate

        self.configure()
        print '-----------------------------'
        print '- Velocity Control Started! -'
        print '-----------------------------'

        while not rospy.is_shutdown():
            while (self.progress):
                if (self.startup):
                    self.update()
                    self.progress = self.proceed()
                rate.sleep()
            print '-------------------'
            print '- Target Reached! -'
            print '-------------------'
            rate.sleep()

    def configure(self):
        """Configures the controller by loading in the room and static
        obstacles.
        Sends Settings to Motionplanner.
        Settings constists of
            - environment
        Waits for Motionplanner to set mp_status to configured.

        NOTE: This would be better as a service!
        """

        self.st = Settings()
        # environment
        self.st.robobst = self._robobst  # dynamic
        self.st.obstacles = []  # static TODO: service call to get obstacles.
        self.st.room = Room(
            position=[
                self.origin.x, self.origin.y,
                self.origin.theta],
            shape=[self.room.width, self.room.height])

        # REPLACE THIS BY SERVICE
        # set motionplanner
        self._mp_configure_topic.publish(self.st)
        self._settings = self.st
        # wait for motionplanner to finish initialization
        while (not self._mp_status):
            self.rate.sleep()
        print '----- Controller Configured -----'

    def planning(self, goal):
        '''Send initial position and first waypoint for the Point2point problem
        to the motionplanner.

        Args:
            goal : geometry_msgs/Pose2D - Terminal position of the p2p problem.
        '''
        self._goal = goal
        # Pose2D(0.5, 0.5)
        self.set_goal()
        self.progress = True
        print 'GOAL SET'

    def get_mp_feedback(self, data):
        '''Sets motionplanner status. If False, then controller.start() will
        wait with starting the controller until True.

        Args:
            data : boolean received from 'mp_feedback' topic, published by
                   motionplanner.
        '''
        self._mp_status = data

    def get_mp_result(self, data):
        '''Store results of motionplanner calculations.

        Args:
            data : calculated trajectories received from 'mp_result' topic,
                   published by motionplanner.
        '''
        v_traj = data.v_traj
        w_traj = data.w_traj
        x_traj = data.x_traj
        y_traj = data.y_traj
        self.store_trajectories(v_traj, w_traj, x_traj, y_traj)

    def update(self):
        '''
        - Updates the controller with newly calculated trajectories and
        velocity commands.
        - Sends out new velocity command.
        - Retrieves new pose estimate.
        '''
        # Send velocity sample.
        self._cmd_twist.header.stamp = rospy.Time.now()
        self.cmd_vel.publish(self.cmd_twist_convert)
        # Store applied commands.
        self._inputs_applied['jx'].append(
                                        self._cmd_twist_convert.twist.linear.x)
        self._inputs_applied['jy'].append(
                                        self._cmd_twist_convert.twist.linear.y)

        # Retrieve new pose estimate from World Model.
        # This is a pose estimate for the first following time instance [k+1]
        # if the velocity command sent above corresponds to time instance [k].
        self._robot_est_pose = self.get_pose_est()

        # Check for new trajectories. Trigger Motionplanner or raise
        # 'overtime'
        pose0 = [self._robot_est_pose.x, self._robot_est_pose.y]

        if self._init:
            if not self._new_trajectories:
                return
            self._index = int(self._update_time/self._sample_time)
            self._init = False

        if ((self._index >= int(self._update_time/self._sample_time))
                or (self._index >= len(self._traj['v']))):
            if self._new_trajectories:
                # Load fresh trajectories.
                self.load_trajectories()
                self._new_trajectories = False
                self._time += self._index*self._sample_time
                self.pos_index = self._index
                self._index = 1
                # Trigger motion planner.
                self.fire_motionplanner(self._time, pose0)
                point = self._robot_est_pose
                self.__publish_marker(
                    self._traj['x'][:], self._traj['y'][:], point)
                self._
            else:
                self.calc_succeeded = False
                print '-- !! -------- !! --'
                print '-- !! Overtime !! --'
                # Brake as emergency measure: Bebop brakes automatically when
                # /bebop/cmd_vel topic receives all zeros.
                self._cmd_twist.twist.linear.x = 0.
                self._cmd_twist.twist.linear.y = 0.
                self._cmd_twist.twist.linear.z = 0.
                self.cmd_vel.publish(self._cmd_twist.twist)
                return

        # Convert feedforward velocity command to angle input.
        self._cmd_twist.twist.linear.x = self._traj['v'][self._index]
        self._cmd_twist.twist.linear.y = self._traj['w'][self._index]
        self.convert_vel_cmd(self._index)

        # Combine feedback and feedforward commands.
        self.calc_vel_cmd()

        self._index += 1

    def convert_vel_cmd(self, index):
        '''Converts a velocity command to a desired input angle according to
        the difference equations:

        - for second order velocity/input relation (x and y):
            j[k+1] = 1/b1*{ -b0*j[k] + a0*v[k] + a1*v[k+1] + v[k+2] }

        (- for first order velocity/input relation (z):
            j[k+1] = 1/b0*{ a0*v[k+1] + v[k+2] } (later, 3d flight))

        where j is the input signal applied to the bebop and v the desired
        velocity.
        '''

        self._cmd_twist.twist
        self.cmd_twist_convert

        # Convert velocity command in x-direction
        phi = np.array([[self.input_old_x],
                        [self._traj['v'][index]],
                        [self._traj['v'][index + 1]],
                        [self._traj['v'][index + 2]]])
        self.cmd_twist_convert.linear.x = np.matmul(self.coeffs_x, phi)

        # Convert velocity command in y-direction
        phi = np.array([[self.input_old_x],
                        [self._traj['w'][index]],
                        [self._traj['w'][index + 1]],
                        [self._traj['w'][index + 2]]])
        self.cmd_twist_convert.linear.y = np.matmul(self.coeffs_y, phi)

    def calc_vel_cmd(self):
        '''Combines the feedforward and feedback commands to generate a
        velocity command and publishes this command.
        '''

        # Summing feedforward and feedback part of the controller.
        x_error = self._traj['x'][self._index] - self._robot_est_pose.x
        y_error = self._traj['y'][self._index] - self._robot_est_pose.y

        # Safety feature, if position measurement stops working, set velocity
        # command equal to zero
        safety_treshold = 0.5
        if (x_error > safety_treshold) or (y_error > safety_treshold):
            self._cmd_twist_convert.twist = Twist()
            return

        # Combine feedforward and feedback part
        self._cmd_twist_convert.twist.linear.x = (
            x_error * self.feedback_gain +
            self._cmd_twist_convert.twist.linear.x)

        self._cmd_twist_convert.twist.linear.y = (
            y_error * self.feedback_gain +
            self._cmd_twist_convert.twist.linear.y)

    def get_pose_est(self):
        '''Retrieves a new pose estimate from world model.
        '''

        rospy.wait_for_service("/world_model/get_pose")
        try:
            pose_est = rospy.ServiceProxy(
                "/world_model/get_pose", GetPoseEst)
            xhat = pose_est(self._cmd_twist)
            self.__publish_real(xhat.point.x, xhat.point.y)
            return xhat.point
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return

    def load_trajectories(self):
        self._traj['v'] = self._traj_strg['v'][:]
        self._traj['w'] = self._traj_strg['w'][:]
        self._traj['x'] = self._traj_strg['x'][:]
        self._traj['y'] = self._traj_strg['y'][:]

    def store_trajectories(self, v_traj, w_traj, x_traj, y_traj):
        '''Stores the trajectories and indicate that new trajectories have
        been calculated.

        Args:
            v_traj : trajectory speed in x-direction
            w_traj : trajectory speed in y-direction
            x_traj : trajectory position in x-direction
            y_traj : trajectory position in y-direction
        '''
        self._traj_strg = {}
        self._traj_strg = {
            'v': v_traj, 'w': w_traj, 'x': x_traj, 'y': y_traj}
        self._new_trajectories = True

        x_traj = self._traj_strg['x'][:]
        y_traj = self._traj_strg['y'][:]
        self.__publish_desired(x_traj, y_traj)

    def proceed(self):
        '''Determines whether goal is reached.
        Returns:
            not stop: boolean whether goal is reached. If not, controller
                      proceeds to goal.
        '''
        if len(self._inputs_applied['jx']) == 0:
            return True

        stop_linear = True
        stop = True

        pos_nrm = np.linalg.norm(np.array(
            [self._robot_est_pose.x, self._robot_est_pose.y])
            - np.array([self._goal.x, self._goal.y]))
        input_nrm = np.linalg.norm(
            [self._inputs_applied['jx'][-1],
                self._inputs_applied['jy'][-1]])
        angle_nrm = np.abs(self.desired_angle - self.current_angle)

        stop_linear = (pos_nrm < self.pos_nrm_tol) and (
                            input_nrm < self.input_nrm_tol)

        if (self.stop_linear):
            self._cmd_twist.twist.linear.x = 0.
            self._cmd_twist.twist.linear.y = 0.
        stop *= (self.stop_linear and angle_nrm < self.angle_nrm_tol)

        return not stop

    def set_goal(self):
        self._time = 0.
        self._new_trajectories = False
        self.fire_motionplanner()
        self._init = True
        self.startup = True

    def fire_motionplanner(self):
        '''Publishes inputs to motionplanner via Trigger topic.
        '''
        self._trigger.goal = self._goal
        self._trigger.state = self._robot_est_pose
        self._trigger.current_time = self.time
        self._mp_trigger_topic.publish(self._trigger)

    def marker_setup(self):
        '''Setup markers to display the desired and real path of the drone in
        rviz.
        '''
        # Desired path
        self._desired_path = Marker()
        self._desired_path.header.frame_id = '/World'
        self._desired_path.ns = "trajectory_desired"
        self._desired_path.id = 0
        self._desired_path.type = 4  # Line List.
        self._desired_path.action = 0
        # self._desired_path.pose.position.z = 0.
        # self._desired_path.pose.orientation.x = 0
        # self._desired_path.pose.orientation.y = 0
        # self._desired_path.pose.orientation.z = 0
        self._desired_path.scale.x = 0.2
        self._desired_path.scale.y = 0.2
        self._desired_path.scale.z = 1.0
        self._desired_path.color.r = 1.0
        self._desired_path.color.g = 0.0
        self._desired_path.color.b = 0.0
        self._desired_path.color.a = 1.0
        self._desired_path.lifetime = rospy.Duration(0)

        self.pos_index = 0
        self.old_len = 0

        # Real path
        self._real_path = Marker()
        self._real_path.header.frame_id = '/World'
        self._real_path.ns = "trajectory_real"
        self._real_path.id = 1
        self._real_path.type = 4  # Line List.
        self._real_path.action = 0
        # self._real_path.pose.position.z = 0.
        # self._real_path.pose.orientation.x = 0
        # self._real_path.pose.orientation.y = 0
        # self._real_path.pose.orientation.z = 0
        self._real_path.scale.x = 0.2
        self._real_path.scale.y = 0.2
        self._real_path.scale.z = 1.0
        self._real_path.color.r = 0.0
        self._real_path.color.g = 1.0
        self._real_path.color.b = 0.0
        self._real_path.color.a = 1.0
        self._real_path.lifetime = rospy.Duration(0)

    def __publish_desired(self, x_traj, y_traj):
        '''Publish planned x and y trajectory to topic for visualisation in
        rviz.
        '''
        # Still has to be adapted to remove old path when new goal has been set.
        self._desired_path.stamp = rospy.get_rostime()

        # Delete points in path that have not been used before new list was
        # calculated.
        self._desired_path.points = self._desired_path.points[
                                            0:(self.old_len + self.pos_index)]
        self.old_len = len(self._desired_path.points)

        # Add new calculated pos list to old one.
        new_pos = np.zeros(len(x_traj))
        for k in range(len(x_traj)):
            new_pos[k] = Point(x=x_traj[k], y=y_traj[k])
        self._desired_path.points + new_pos.tolist()

        self.trajectory_desired.publish(self._desired_path)

    def __publish_real(self, x_pos, y_pos):
        '''Publish real x and y trajectory to topic for visualisation in
        rviz.
        '''
        self._real_path.header.stamp = rospy.get_rostime()

        point = Point(x=x_pos, y=y_pos)
        self._real_path.points.append(point)

        self.trajectory_real.publish(self._real_path)


if __name__ == '__main__':
    vel_command = VelCommander()
    vel_command.start()
