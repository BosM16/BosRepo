#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped, Pose, Point, PointStamped
import numpy as np
import rospy
import tf2_ros


class WorldModel(object):
    '''
    Contains all information of the world: the position of the obstacles, and
    the position and pose of the drone.
    '''

    def __init__(self):
        """
        Initialization of WorldModel object.
        """
        # Parameters.
        self.vel_cmd_Ts = rospy.get_param('vel_cmd/sample_time', 0.01)  # s
        self.max_vel = 0.4  # m/s
        self.max_accel = 0.2  # m/s**2
        self.t0 = rospy.get_time()
        rospy.set_param('vel_cmd/max_vel', self.max_vel)
        rospy.set_param('vel_cmd/max_accel', self.max_accel)

        self.R = np.identity(3)  # measurement noise covariance
        self.Q = 1e-2*np.identity(9)  # process noise covariance

        # Variables.
        self.xhat_t0 = PointStamped()
        self.xhat_t0.header.frame_id = "world"
        self.xhat = PointStamped()
        self.xhat.header.frame_id = "world"

        self.xhat_r_t0 = PointStamped()
        self.xhat_r_t0.header.frame_id = "world_rot"

        self.xhat_r = PointStamped()
        self.xhat_r.header.frame_id = "world_rot"

        self.Phat_t0 = np.zeros(9)
        self.Phat = np.zeros(9)
        self.X = np.zeros((9, 1))

        # State space model matrices for position Kalman filter in
        # continuous time!! Are then converted to discrete time further on
        # depending on Ts
        self.A = np.identity(9)  # continuous A matrix
        self.B = np.zeros([9, 3])  # continuous B matrix
        self.C = np.zeros([3, 9])
        self.C[0, 0] = 1
        self.C[1, 3] = 1
        self.C[2, 6] = 1

        # Transformations.
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def predict_pos_update(self, vel_cmd_stamped, Ts):
        """
        Prediction step of the kalman filter. Update the position of the drone
        using the reference velocity commands.
        vel_cmd_stamped = TwistStamped
        """
        vel_cmd = vel_cmd_stamped.twist

        u = np.array([[vel_cmd.linear.x],
                      [vel_cmd.linear.y],
                      [vel_cmd.linear.z]])

        self.X = np.matmul(Ts*self.A + np.identity(9), self.X) +
            np.matmul(Ts*self.B, u)

        self.xhat_r.point.x = self.X[0, 0]
        self.xhat_r.point.y = self.X[3, 0]
        self.xhat_r.point.z = self.X[6, 0]

        self.Phat = np.matmul(self.A, np.matmul(self.Phat, np.transpose(self.A))) + self.Q

    def correct_pos_update(self, pos_meas):
        """
        Correction step of the kalman filter. Update the position of the drone
        using the measurements.
        """

        y = np.array([[pos_meas.pose.position.x],
                      [pos_meas.pose.position.y],
                      [pos_meas.pose.position.z]])

        nu = y - np.matmul(self.C, self.X)
        S = np.matmul(self.C, np.matmul(self.Phat, np.transpose(self.C))) + self.R
        L = np.matmul(self.Phat, np.matmul(np.transpose(self.C), np.linalg.inv(S)))
        self.X = self.X + np.matmul(L, nu)
        self.Phat = np.matmul(
            (np.identity(3) - np.matmul(L, self.C)), self.Phat)

        self.xhat_r.point.x = self.X[0, 0]
        self.xhat_r.point.y = self.X[3, 0]
        self.xhat_r.point.z = self.X[6, 0]

        self.xhat_r_t0 = self.xhat_r

        # Time of last measurement is new t0.
        self.t0 = self.get_timestamp(pos_meas)

    def get_timestamp(self, stamped_var):
        time = stamped_var.header.stamp.to_sec()
        return time


if __name__ == '__main__':
    world_model = WorldModel()
