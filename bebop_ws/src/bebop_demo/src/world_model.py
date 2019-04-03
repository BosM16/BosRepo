#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped, PointStamped
import numpy as np
import rospy
import tf2_ros


class WorldModel(object):
    '''
    Contains all available information of the world:
        - Drone dynamic model
        - Position of the obstacles
        - Position and pose of the drone
    Executes predictions and corrections on the drone position estimate using
    Kalman algorithm.
    '''

    def __init__(self):
        """
        Initialization of WorldModel object.
        """
        # Parameters.
        self.max_vel = rospy.get_param('motionplanner/vmax', 0.4)  # m/s
        self.max_accel = rospy.get_param('motionplanner/amax', 0.2)  # m/s**2
        self.drone_radius = rospy.get_param('motionplanner/drone_radius', 0.2)

        # Variables.
        self.yhat = PointStamped()
        self.yhat.header.frame_id = "world"
        self.yhat_r = PointStamped()
        self.yhat_r.header.frame_id = "world_rot"
        self.yhat_r_t0 = PointStamped()
        self.yhat_r_t0.header.frame_id = "world_rot"
        self.vhat = PointStamped()  # Store velocities from kalman filter
        self.vhat.header.frame_id = "world"
        self.vhat_r = PointStamped()  # Store velocities from kalman filter
        self.vhat_r.header.frame_id = "world_rot"
        self.state = "initialization"
        self.model = Model()

        # Staat hier voor uitlezen van obstakels in de toekomst!
        # rospy.Subscriber(
        #     'vive_localization/c1_pose', PoseStamped, self.right_ctrl_update)
        # rospy.Subscriber(
        #     'vive_localization/c2_pose', PoseStamped, self.left_ctrl_update)


class Model(object):

    def __init__(self):
        self.initialize_model()

    def initialize_model(self):
        '''Initializes the model to be used in the Kalman filter.
        State space model x'(t) = A*x(t) + B*u(t) in observable canonical
        form, corresponding to transfer function

                  b2*s^2 + b1*s + b0
        G(s) = --------------------------
                s^3 + a2*s^2 + a1*s + a0

        State space model matrices for position Kalman filter are in
        continuous time!! Are then converted to discrete time further on
        depending on varying Ts.
        '''

        a2x = 6.167
        a1x = 1.523
        a0x = 0.0
        Ax = np.array([[0., 1., 0.],
                       [0., 0., 1.],
                       [-a0x, -a1x, -a2x]])
        a2y = 5.147
        a1y = 2.116
        a0y = 0.0
        Ay = np.array([[0., 1., 0.],
                       [0., 0., 1.],
                       [-a0y, -a1y, -a2y]])
        a1z = 6.26
        a0z = 0.0
        Az = np.array([[0., 1.],
                       [-a0z, -a1z]])

        self.A = np.zeros([8, 8])  # continuous A matrix
        self.A[0:3, 0:3] = Ax
        self.A[3:6, 3:6] = Ay
        self.A[6:8, 6:8] = Az

        self.B = np.zeros([8, 3])  # continuous B matrix
        self.B[2, 0] = 1
        self.B[5, 1] = 1
        self.B[7, 2] = 1

        b2x = 0.0
        b1x = 0.0
        b0x = 22.51
        Bx = np.array([b0x, b1x, b2x])
        b2y = 0.0
        b1y = 0.0
        b0y = 18.96
        By = np.array([b0y, b1y, b2y])
        b1z = 0.0
        b0z = 6.066
        Bz = np.array([b0z, b1z])
        self.C = np.zeros([3, 8])
        self.C[0, 0:3] = Bx
        self.C[1, 3:6] = By
        self.C[2, 6:8] = Bz

        Bx = np.array([-b2x*a0x, b0x - a1x*b2x, b1x - b2x*a2x])
        By = np.array([-b2y*a0y, b0y - a1y*b2y, b1y - b2y*a2y])
        Bz = np.array([-b1z*a0z, b0z - a1z*b1z])
        self.C_vel = np.zeros([3, 8])
        self.C_vel[0, 0:3] = Bx
        self.C_vel[1, 3:6] = By
        self.C_vel[2, 6:8] = Bz

        self.D_vel = np.zeros([3, 3])
        self.D_vel[0, 0] = b2x
        self.D_vel[1, 1] = b2y
        self.D_vel[2, 2] = b1z


if __name__ == '__main__':
    world_model = WorldModel()
