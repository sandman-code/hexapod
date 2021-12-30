import numpy as np
L1 = 29.49
L2 = 50.12
L3 = 63.94
# [-(sin(theta1)*sin(theta2))]
#
#
#
#
#
#
#
#


def calc_jacobian(theta1, theta2, theta3):
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
    return J


def calIK(posArr):
    alpha0 = 0  # hip joint starting position
    beta0 = np.arctan(HEIGHT/2)  # knee joint in starting position
    gamma0 = 90 * np.pi / 180  # ankle joint in starting position


def calc_fk(angles):

    # Populate Joint Angles
    x1 = angles[0]
    x2 = angles[1]
    x3 = angles[2]

    # Populate DH (known)

    # a distances
    a1 = L1
    a2 = L2
    a3 = L3

    # alpha angles
    alpha1 = np.pi / 2
    alpha2 = 0
    alpha3 = -np.pi / 2

    # End Effector Frame
    x4 = -np.pi / 2
    a4 = 0
    alpha4 = np.pi / 2

    # Fill Transormation Matrices
    T01 = np.matrix([
        [np.cos(x1), -np.sin(x1)*np.cos(alpha1), np.sin(x1)*np.sin(alpha1), a1*np.cos(x1)],
        [np.sin(x1), np.cos(x1)*np.cos(alpha1), -np.cos(x1)*np.sin(alpha1), a1*np.sin(x1)],
        [0,np.sin(alpha1),np.cos(alpha1),0],
        [0,0,0,1],
    ])

    T12 = np.matrix([
        [np.cos(x2), -np.sin(x2)*np.cos(alpha2), np.sin(x2)*np.sin(alpha2), a2*np.cos(x2)],
        [np.sin(x2), np.cos(x2)*np.cos(alpha2), -np.cos(x2)*np.sin(alpha2), a2*np.sin(x2)],
        [0,np.sin(alpha2),np.cos(alpha2),0],
        [0,0,0,1],
    ])

    T23 = np.matrix([
        [np.cos(x3), -np.sin(x3)*np.cos(alpha3), np.sin(x3)*np.sin(alpha3), a3*np.cos(x3)],
        [np.sin(x3), np.cos(x3)*np.cos(alpha3), -np.cos(x3)*np.sin(alpha3), a3*np.sin(x3)],
        [0,np.sin(alpha3),np.cos(alpha3),0],
        [0,0,0,1],
    ])

    T34 = np.matrix([
        [np.cos(x4), (-np.sin(x4) * np.cos(alpha4)), np.sin(x4)*np.sin(alpha4), a4*np.cos(x4)],
        [np.sin(x4), (np.cos(x4) * np.cos(alpha4)), (-np.cos(x4)*np.sin(alpha4)), (a4*np.sin(x4))],
        [0,np.sin(alpha4),np.cos(alpha4),0],
        [0,0,0,1],
    ])

    return T01 @ T12 @ T23 @ T34

