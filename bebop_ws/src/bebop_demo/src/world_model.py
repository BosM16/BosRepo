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
        Ax = np.array([[0., 1., 0.],
                       [0., 0., 1.],
                       [-a0x, -a1x, -a2x]])
        a2y = 5.921
        a1y = 2.469
        a0y = 0.02513
        Ay = np.array([[0., 1., 0.],
                       [0., 0., 1.],
                       [-a0y, -a1y, -a2y]])
        a1z = 1.515
        a0z = 0.02072
        Az = np.array([[0., 1.],
                       [-a0z, -a1z]])

        self.model.A = np.zeros([8, 8])  # continuous A matrix
        self.model.A[0:3, 0:3] = Ax
        self.model.A[3:6, 3:6] = Ay
        self.model.A[6:8, 6:8] = Az

        self.model.B = np.zeros([8, 3])  # continuous B matrix
        self.model.B[2, 0] = 1
        self.model.B[5, 1] = 1
        self.model.B[7, 2] = 1

        b2x = -0.01447
        b1x = -1.008
        b0x = 18.32
        Bx = np.array([b0x, b1x, b2x])
        b2y = -0.008856
        b1y = -1.063
        b0y = 19.74
        By = np.array([b0y, b1y, b2y])
        b1z = 0.6266
        b0z = 1.597
        Bz = np.array([b0z, b1z])
        self.model.C = np.zeros([3, 8])
        self.model.C[0, 0:3] = Bx
        self.model.C[1, 3:6] = By
        self.model.C[2, 6:8] = Bz

        Bx = np.array([-b2x*a0x, b0x - a1x*b2x, b1x - b2x*a2x])
        By = np.array([-b2y*a0y, b0y - a1y*b2y, b1y - b2y*a2y])
        Bz = np.array([-b1z*a0z, b0z - a1z*b1z])
        self.model.C_vel = np.zeros([3, 8])
        self.model.C_vel[0, 0:3] = Bx
        self.model.C_vel[1, 3:6] = By
        self.model.C_vel[2, 6:8] = Bz

        self.model.D_vel = np.zeros([3, 1])
        self.model.D_vel[0, :] = b2x
        self.model.D_vel[1, :] = b2y
        self.model.D_vel[2, :] = b1z


if __name__ == '__main__':
    world_model = WorldModel()
