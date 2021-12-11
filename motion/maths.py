import numpy as np
from hexapodcore import L1, L2, L3, HEIGHT
# [-(sin(theta1)*sin(theta2))]
#
#
#
#
#
#
#
#


def calcJacobian(theta1, theta2, theta3):
    J = np.zeros((3, 3))

    J[0, 0] = -(-np.sin(theta1)*np.sin(theta2)*np.cos(theta3)-np.sin(theta1)*np.cos(theta2)
                * np.sin(theta3))*L3-np.sin(theta3)*L2*np.cos(theta2)-L1*np.sin(theta1)
    J[0, 1] = -(-np.cos(theta1)*np.sin(theta2)*np.sin(theta3)-np.cos(theta1)
                * np.cos(theta2)*np.cos(theta3))*L3-np.cos(theta1)*L2*np.sin(theta2)
    J[0, 2] = (np.cos(theta1)*np.sin(theta2)*np.sin(theta3) -
               np.cos(theta1)*np.cos(theta2)*np.cos(theta3))*L3
    J[1, 0] = -(np.cos(theta1)*np.cos(theta2)*np.sin(theta3)+np.cos(theta1)*np.sin(theta2)
                * np.cos(theta3))*L3+np.cos(theta1)*L2*np.cos(theta2)+L1*np.cos(theta1)
    J[1, 1] = -(-np.sin(theta1)*np.sin(theta2)*np.sin(theta3)+np.sin(theta1)
                * np.cos(theta2)*np.cos(theta3))*L3-np.sin(theta1)*L2*np.sin(theta2)
    J[1, 2] = -(np.sin(theta1)*np.sin(theta2)*np.sin(theta3) +
                np.sin(theta1)*np.cos(theta2)*np.cos(theta3))*L3
    J[2, 0] = 0
    J[2, 1] = -(-np.cos(theta2)*np.sin(theta3)-np.sin(theta2)
                * np.cos(theta3))*L3-L2*np.cos(theta2)
    J[2, 2] = -(-np.cos(theta2)*np.sin(theta3) -
                np.sin(theta2)*np.cos(theta3))*L3

    # inverse (J) * [xdot; ydot; zdot]
    return J


def calIK(posArr):
    alpha0 = 0  # hip joint starting position
    beta0 = np.arctan(HEIGHT/2)  # knee joint in starting position
    gamma0 = 90 * np.pi / 180  # ankle joint in starting position


def calcFK(angleArr):
    theta1 = angleArr[0]
    theta2 = angleArr[1]
    theta3 = angleArr[2]

    # Fill Transormation Matrices
    T01 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T12 = np.array([
        [np.cos(theta2), 0, 0, L2*np.cos(theta2)],
        [np.sin(theta2), np.cos(theta2), 0, L2*np.sin(theta2)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T23 = np.array([
        [np.cos(theta3), 0, 0, L3*np.cos(theta3)],
        [np.sin(theta3), np.cos(theta3), 0, L3*np.sin(theta3)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T34 = np.array([
        [0, 0, 1, 0],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    return T01*T12*T23*T34
