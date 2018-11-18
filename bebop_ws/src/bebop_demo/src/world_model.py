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
        self.xhat = PointStamped()
        self.xhat.header.frame_id = "world"
        self.xhat_r = PointStamped()
        self.xhat_r.header.frame_id = "world_rot"
        self.xhat_r_t0 = PointStamped()
        self.xhat_r_t0.header.frame_id = "world_rot"
        
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

        a2x = 5.458
        a1x = 2.467
        a0x = -0.03519
        Ax = np.array([[-a2x, 1., 0.],
                       [-a1x, 0., 1.],
                       [-a0x, 0., 0.]])
        a2y = 5.921
        a1y = 2.469
        a0y = 0.02513
        Ay = np.array([[-a2y, 1., 0.],
                       [-a1y, 0., 1.],
                       [-a0y, 0., 0.]])
        a1z = 1.515
        a0z = 0.02072
        Az = np.array([[-a1z, 1.],
                       [-a0z, 0.]])

        self.A = np.zeros([8, 8])  # continuous A matrix
        self.A[0:3, 0:3] = Ax
        self.A[3:6, 3:6] = Ay
        self.A[6:8, 6:8] = Az

        b2x = -0.01447
        b1x = -1.008
        b0x = 18.32
        Bx = np.array([b2x, b1x, b0x])
        b2y = -0.008856
        b1y = -1.063
        b0y = 19.74
        By = np.array([b2y, b1y, b0y])
        b1z = 0.6266
        b0z = 1.597
        Bz = np.array([b1z, b0z])
        self.B = np.zeros([8, 3])  # continuous B matrix
        self.B[0:3, 0] = Bx
        self.B[3:6, 1] = By
        self.B[6:8, 2] = Bz

        self.C = np.zeros([3, 8])
        self.C[0, 0] = 1
        self.C[1, 3] = 1
        self.C[2, 6] = 1


if __name__ == '__main__':
    world_model = WorldModel()
