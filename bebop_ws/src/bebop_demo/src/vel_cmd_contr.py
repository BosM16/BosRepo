#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose, Pose2D
from std_msgs.msg import Bool, Empty, UInt8
import rospy


class VelCommander(object):
    _cmd_twist = Twist()
    _trigger = Trigger()

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
        self.feedback_gain = 0.3
        self._sample_time = rospy.get_param('sample_time', 0.01)
        self._update_time = rospy.get_param('update_time', 0.5)
        self.rate = rospy.Rate(1./self._sample_time)
        self.pos_nrm = np.inf
        self._robobst = []
        self._robot_est_pose = Pose2D()
        self._robot_est_pose.x = 0.
        self._robot_est_pose.y = 0.
        self._robot_est_pose.theta = 0.
        self._robobst_est_pose = [[0., 0.] for k in range(len(self._robobst))]
        self._robobst_est_velocity = [[0., 0.] for k in range(
            len(self._robobst))]
        self._traj = {'v': [], 'w': [], 'x': [], 'y': []}
        self._traj_strg = {'v': [], 'w': [], 'x': [], 'y': []}
        self._vel_traj_applied = {'v': [], 'w': []}

        self.cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('demo', Empty, self.flying)
        self._mp_trigger_topic = rospy.Publisher(
            'mp_trigger', Trigger, queue_size=1)
        self._mp_configure_topic = rospy.Publisher(
            'mp_configure', Settings, queue_size=1)
        rospy.Subscriber('mp_result', RobotTrajectory, self.get_mp_result)
        rospy.Subscriber('mp_feedback', Bool, self.get_mp_feedback)

    def get_mp_feedback(self, data):
        """Sets motionplanner status. If False, then controller.start() will wait
        with starting the controller until True.

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
        """Update the controller with newly calculated trajectories and velocity
        commands.
        """
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
                self._index = 0
                # Trigger motion planner.
                self.fire_motionplanner(self._time, pose0)
            else:
                self.calc_succeeded = False
                print 'overtime!'
                self._cmd_twist.linear.x = 0.
                self._cmd_twist.linear.y = 0.
                return

        # update position with feedforward position (temporary for testing).
        self._robot_est_pose.x = self._traj['x'][self._index]
        self._robot_est_pose.y = self._traj['y'][self._index]
        # Feedforward velocity command.
        self._cmd_twist.linear.x = self._traj['v'][self._index]
        self._cmd_twist.linear.y = self._traj['w'][self._index]
        # Summing feedforward and feedback part of the controller.
        self._cmd_twist.linear.x = ((
            self._traj['x'][self._index]
            - self._robot_est_pose.x) * self.feedback_gain
            + self._cmd_twist.linear.x)
        self._cmd_twist.linear.y = ((
            self._traj['y'][self._index]
            - self._robot_est_pose.y) * self.feedback_gain
            + self._cmd_twist.linear.y)
        # send velocity sample
        self.cmd_vel.publish(self._cmd_twist)
        self._vel_traj_applied['v'].append(self._cmd_twist.linear.x)
        self._vel_traj_applied['w'].append(self._cmd_twist.linear.y)
        self._index += 1

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
            self._cmd_twist.linear.x = 0.
            self._cmd_twist.linear.y = 0.
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
        self._trigger.dyn_obstacles = [Obstacle(
            pose=self._robobst_est_pose[k], velocity=(
                self._robobst_est_velocity[k]))
                    for k in range(len(self._robobst))]
        self._trigger.current_time = time
        self._mp_trigger_topic.publish(self._trigger)

    def start(self):
        """Starts the controller's periodical loop.
        """
        rate = self.rate
        while (not self._mp_status):
            rate.sleep()
        print 'controller started!'
        k = 0

        self.st.room = Room(
            position=[
                self.proc_map.origin.x, self.proc_map.origin.y,
                self.proc_map.origin.theta],
            shape=[self.proc_map.width, self.proc_map.height])
        obstacles = []
        self.st.obstacles = obstacles

        while (True):
            while (self.progress):
                if (self.startup):
                    k += 1
                    self.update()
                    self.progress = self.proceed()
                rate.sleep()
            print 'target reached!'
            rate.sleep()

    def configure(self):
        """Configures the controller by loading in the room and static obstacles.
        """
        print 'configure controller'
        self.st = Settings()
        # timing
        self.st.sample_time = self._sample_time
        self.st.update_time = self._update_time
        # environment
        self.st.robobst = self._robobst
        # set motionplanner
        self._mp_configure_topic.publish(self.st)
        self._settings = self.st
        # wait for motionplanner to finish initialization
        while (not self._mp_status):
            self.rate.sleep()

    def planning(self):
        """Send initial position and first waypoint for the Point2point problem
        to the motionplanner.

        Args:
            waypoints : list of all waypoints coming from the global planner.
        """
        self._goal = RobotPose([0.5, 0.5])

        return True

    def flying(self):
        self.progress = True
        print 'flying'

if __name__ == '__main__':
    controller = Controller()
    controller.start()
    controller.configure()
    controller.planning()
    controller.set_goal()
