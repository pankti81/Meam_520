import math

import numpy as np
from math import pi


class FK():

    def __init__(self):
        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout
        pass

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here

        jointPositions = np.zeros((8, 3))
        T0e = np.identity(4)

        # take in the joint angle between base frame and frame 1
        q = np.concatenate(([0], q))

        # link length
        a = np.array([0, 0, 0, 0.0825, 0.0825, 0, 0.088, 0])

        # link offset
        d = np.array([0.141, 0.192, 0, 0.316, 0, 0.384, 0, 0.21])

        # link twist
        alpha = np.array([0, -pi / 2, pi / 2, pi / 2, pi / 2, -pi / 2, pi / 2, 0])

        # local coordinates
        c = np.array(
            [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0.195, 1], [0, 0, 0, 1],
             [0, 0, 0.125, 1], [0, 0, -0.015, 1],
             [0, 0, 0.051, 1], [0, 0, 0, 1]])

        A0 = self.get_A(a[0], alpha[0], d[0], q[0])
        A1 = self.get_A(a[1], alpha[1], d[1], q[1])
        A2 = self.get_A(a[2], alpha[2], d[2], q[2])
        A3 = self.get_A(a[3], alpha[3], d[3], q[3])
        A4 = self.get_A(a[4], alpha[4], d[4], q[4] + pi)
        A5 = self.get_A(a[5], alpha[5], d[5], q[5])
        A6 = self.get_A(a[6], alpha[6], d[6], q[6] - pi)
        Ae = self.get_A(a[7], alpha[7], d[7], q[7] - pi / 4)

        T0 = A0
        T1 = T0 @ A1
        T2 = T1 @ A2
        T3 = T2 @ A3
        T4 = T3 @ A4
        T5 = T4 @ A5
        T6 = T5 @ A6
        Te = T6 @ Ae

        T0e = Te

        p0 = T0 @ c[0]
        p1 = T1 @ c[1]
        p2 = T2 @ c[2]
        p3 = T3 @ c[3]
        p4 = T4 @ c[4]
        p5 = T5 @ c[5]
        p6 = T6 @ c[6]
        pe = Te @ c[7]

        jointPositions[0, :] = p0[0:3]
        jointPositions[1, :] = p1[0:3]
        jointPositions[2, :] = p2[0:3]
        jointPositions[3, :] = p3[0:3]
        jointPositions[4, :] = p4[0:3]
        jointPositions[5, :] = p5[0:3]
        jointPositions[6, :] = p6[0:3]
        jointPositions[7, :] = pe[0:3]

        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1
    def get_A(self, a, alpha, d, theta):
        A = np.array(
            [[math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha),
              a * math.cos(theta)],
             [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha),
              a * math.sin(theta)],
             [0, math.sin(alpha), math.cos(alpha), d],
             [0, 0, 0, 1]])
        return A

    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return ()

    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return ()


if __name__ == "__main__":
    fk = FK()

    # matches figure in the handout
    q = np.array([0, 0, 0, -pi / 2, 0, pi / 2, pi / 4])

    joint_positions, T0e = fk.forward(q)

    print("Joint Positions:\n", joint_positions)
    print("End Effector Pose:\n", T0e)