#!/usr/bin/env python

from geometry_msgs.msg import Twist, TwistStamped, Point, Pose2D
from std_msgs.msg import Bool, Empty
from bebop_demo.srv import GetPoseEst
from visualization_msgs.msg import MarkerArray

from bebop_demo.msg import (
    Trigger, RobotTrajectory, RobotPose, Obstacle, Room, Settings)

import rospy


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
        self.stop_linear = False
        self._mp_status = False

        self.origin = Pose2D()
        self.origin.x = 3.0
        self.origin.y = 1.5
        self.room.width = 6.0  # width volgens x-as
        self.room.height = 3.0  # heigth volgens y-as

        self.feedback_gain = 0.3

        self._sample_time = rospy.get_param('vel_cmd/sample_time', 0.01)
        self._update_time = rospy.get_param('vel_cmd/update_time', 0.5)
        self.rate = rospy.Rate(1./self._sample_time)

        self.pos_nrm = np.inf

        self._cmd_twist = TwistStamped()
        self._cmd_twist.header.frame_id = "world_rot"
        self._trigger = Trigger()

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
        self._vel_traj_applied = {'v': [], 'w': []}

        self.cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self._mp_trigger_topic = rospy.Publisher(
            'mp_trigger', Trigger, queue_size=1)
        self._mp_configure_topic = rospy.Publisher(
            'mp_configure', Settings, queue_size=1)

        rospy.Subscriber('demo', Robotpose, self.planning)
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
            - sample time
            - update time
            - environment
        Waits for Motionplanner to set mp_status to configured.

        NOTE: This would be better as a service!
        """

        self.st = Settings()
        # timing
        self.st.sample_time = self._sample_time
        self.st.update_time = self._update_time
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
        """Send initial position and first waypoint for the Point2point problem
        to the motionplanner.

        Args:
            waypoints : list of all waypoints coming from the global planner.
        """
        self._goal = goal
        # RobotPose([0.5, 0.5])
        self.set_goal()
        self.progress = True
        print 'flying'

    def get_mp_feedback(self, data):
        """Sets motionplanner status. If False, then controller.start() will
        wait with starting the controller until True.

        Args:
            data : boolean received from 'mp_feedback' topic, published by
                   motionplanner.
        """
        self._mp_status = data

    def get_mp_result(self, data):
        """Store results of motionplanner calculations.

        Args:
            data : calculated trajectories received from 'mp_result' topic,
                   published by motionplanner.
        """
        v_traj = data.v_traj
        w_traj = data.w_traj
        x_traj = data.x_traj
        y_traj = data.y_traj
        self.store_trajectories(v_traj, w_traj, x_traj, y_traj)

    def update(self):
        """
        - Updates the controller with newly calculated trajectories and
        velocity commands.
        - Sends out new velocity command.
        - Retrieves new pose estimate.
        """
        # Send velocity sample.
        self._cmd_twist.header.stamp = rospy.Time.now()
        self.cmd_vel.publish(self._cmd_twist.twist)
        self._vel_traj_applied['v'].append(self._cmd_twist.twist.linear.x)
        self._vel_traj_applied['w'].append(self._cmd_twist.twist.linear.y)

        # Retrieve new pose estimate from World Model.
        self.get_pose_est()  # opgelet: dit is een predictie voor het volgende
        # tijdstip.

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
                self._cmd_twist.twist.linear.x = 0.
                self._cmd_twist.twist.linear.y = 0.
                self.cmd_vel.publish(self._cmd_twist.twist)
                return

        self.calc_vel_cmd()

        self._index += 1

    def calc_vel_cmd(self):
        '''Combines the feedforward and feedback commands to generate a
        velocity command and publishes this command.
        '''
        # Feedforward velocity command.
        self._cmd_twist.twist.linear.x = self._traj['v'][self._index]
        self._cmd_twist.twist.linear.y = self._traj['w'][self._index]
        # Summing feedforward and feedback part of the controller.
        self._cmd_twist.twist.linear.x = ((
            self._traj['x'][self._index]
            - self._robot_est_pose.x) * self.feedback_gain
            + self._cmd_twist.twist.linear.x)
        self._cmd_twist.twist.linear.y = ((
            self._traj['y'][self._index]
            - self._robot_est_pose.y) * self.feedback_gain
            + self._cmd_twist.twist.linear.y)

    def get_pose_est(self):
        '''Retrieves a new pose estimate from world model.
        '''

        rospy.wait_for_service("/world_model/get_pose")
        try:
            pose_est = rospy.ServiceProxy(
                "/world_model/get_pose", GetPoseEst)
            xhat = pose_est(self._cmd_twist)
            self._robot_est_pose = xhat.point
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def __publish_marker(self, x_traj, y_traj, position):
        '''Publish planned x and y trajectory to topic for visualisation in
        rviz.
        '''
        for i in range(0, 2):
            traj_marker.markers[i] = MarkerArray()
            traj_marker.markers[i].header.frame_id = '/map'
            traj_marker.markers[i].header.stamp = rospy.get_rostime()
            traj_marker.markers[i].ns = "trajectory"
            traj_marker.markers[i].id = i
            traj_marker.markers[i].type = 4  # Line List.
            traj_marker.markers[i].action = 0
            traj_marker.markers[i].pose.position.z = 0.
            traj_marker.markers[i].pose.orientation.x = 0
            traj_marker.markers[i].pose.orientation.y = 0
            traj_marker.markers[i].pose.orientation.z = 0
            traj_marker.markers[i].scale.x = 0.2
            traj_marker.markers[i].scale.y = 0.2
            traj_marker.markers[i].scale.z = 1.0
            traj_marker.markers[i].color.r = 0.0
            traj_marker.markers[i].color.g = 0.0
            traj_marker.markers[i].color.b = 0.0
            traj_marker.markers[i].color.a = 1.0
            traj_marker.markers[i].lifetime = rospy.Duration(0)

            traj_marker.markers[0].color.r = 1.0
            traj_marker.markers[1].color.b = 1.0

        for k in range(len(x_traj)):

            point = Point()
            point.x = x_traj[k]
            point.y = y_traj[k]
            traj_marker.markers[0].points.append(point)

        traj_marker.markers[1].points.append(point)

        self.trajectory_marker.publish(traj_marker)

    def load_trajectories(self):
        self._traj['v'] = self._traj_strg['v'][:]
        self._traj['w'] = self._traj_strg['w'][:]
        self._traj['x'] = self._traj_strg['x'][:]
        self._traj['y'] = self._traj_strg['y'][:]

    def store_trajectories(self, v_traj, w_traj, x_traj, y_traj):
        """Stores the trajectories and indicate that new trajectories have
        been calculated.

        Args:
            v_traj : trajectory speed in x-direction
            w_traj : trajectory speed in y-direction
            x_traj : trajectory position in x-direction
            y_traj : trajectory position in y-direction
        """
        self._traj_strg = {}
        self._traj_strg = {
            'v': v_traj, 'w': w_traj, 'x': x_traj, 'y': y_traj}
        self._new_trajectories = True

    def proceed(self):
        """Determines whether goal is reached.
        Returns:
            not stop: boolean whether goal is reached. If not, controller
                      proceeds to goal.
        """
        if len(self._vel_traj_applied['v']) == 0:
            return True
        stop = True
        self.pos_nrm = np.linalg.norm(np.array(
            [self._robot_est_pose.x, self._robot_est_pose.y])
            - np.array(self._goal.pose))
        self.vel_nrm = np.linalg.norm(
            [self._vel_traj_applied['v'][-1],
                self._vel_traj_applied['w'][-1]])
        self.angle_nrm = np.abs(self.desired_angle - self.current_angle)
        self.stop_linear = self.pos_nrm < 0.1 and self.vel_nrm < 0.1
        if (self.stop_linear):
            self._cmd_twist.twist.linear.x = 0.
            self._cmd_twist.twist.linear.y = 0.
        stop *= (self.stop_linear and self.angle_nrm < 0.05)
        return not stop

    def set_goal(self):
        self._time = 0.
        pose0 = [self._robot_est_pose.x, self._robot_est_pose.y]
        self._new_trajectories = False
        self.fire_motionplanner(self._time, pose0)
        self._init = True
        self.startup = True

    def fire_motionplanner(self, time, pose0):
        """Publishes inputs to motionplanner via Trigger topic.
        """
        self._trigger.goal = self._goal
        self._trigger.state = RobotPose(pose0[:])
        self._trigger.current_time = time
        self._mp_trigger_topic.publish(self._trigger)


if __name__ == '__main__':
    vel_command = VelCommander()
    vel_command.start()
