import numpy as np
from math import sqrt, radians, sin, cos, degrees, acos, isnan

L1 = 29.49
L2 = 50.12
L3 = 63.94



class Vector:
    __slots__ = ('x', 'y', 'z', 'id')

    def __init__(self, x, y, z, id=None):
        self.x = x,
        self.y = y,
        self.z = z,
        self.id = id

    @property
    def vec(self):
        return self.x, self.y, self.z
    

    def magnitude(self):
        return sqrt((self.x ** 2) + (self.y ** 2) + (self.z ** 2))

    def vec_arr(self):
        return np.array([self.x, self.y, self.z])
    
    def transposed(self):
        return np.transpose(self.vec_arr())
    


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

'''
Input: Angle (deg)

Output: Tuple of cos and sin value

*Used for matrix multiplication

'''
def return_cos_and_sin(theta):
    d = degrees(theta)
    c = cos(d)
    s = sin(d)
    return c, s

'''
Input: Angle (deg)

Output: Rotational Matrix on th X-axis

*Used for points and vector references
'''
def rotx(theta):
    c, s = return_cos_and_sin(theta)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


'''
Input: Angle (deg)

Output: Rotational Matrix on the Y-axis

*Used for points and vector references
'''
def roty(theta):
    c, s = return_cos_and_sin(theta)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])


'''
Input: Angle (deg)

Output: Rotational Matric on the Z-axis

*Used for points and vector references
'''
def rotz(theta):
    c, s = return_cos_and_sin(theta)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

'''
Input: 2 Vectors

Output: Dot product of those two vectors

*Used with vector class
'''
def dot_product(v1, v2):
    return v1.x * v2.x + v1.y * v2.y + v1.z * v1.z


'''
Input: 2 Vectors

Output: Cross product of the two vectors

*Used with vector class
'''
def cross(v1, v2):
    x = v1.y * v2.z - v1.z * v2.y
    y = v1.z * v2.x - v1.x * v2.z
    z = v1.x * v2.y - v1.y * v2.x
    return Vector(x, y, z)

'''
Input: Vector and a number

Output: A new Vector scaled to the given number

*Used with vector class
'''
def scale(v, d):
    return Vector(v.x / d, v.y / d, v.z / d)

'''
Input: Two numbers (points)

Output: Vector between those points

*Used with vector class
'''
def vector_from(a, b):
    return Vector(b.x - a.x, b.y - a.y, b.z - a.z)

'''
Input: Vector

Output: Numeric length of the vector in space

*Uses vector class
'''
def length(v):
    return sqrt(dot_product(v, v))


'''
Input: Two vectors

Output: Vector

*Uses vector class
'''
def add_vectors(a, b):
    return Vector(a.x + b.x, a.y + b.y, a.z + b.z)


'''
Input: Two vectors

Output: Vector

*Uses vector class
'''

def subtract_vectors(a, b):
    return Vector(a.x - b.x, a.y - b.y, a.z - b.z)


'''
Input: Vector and number

Output: Vector that is multiplied by the given number

*Uses vector class
'''

def scalar_multiply(p, s):
    return Vector(s * p.x, s * p.y, s * p.z)



'''
Input: Vector

Ouptut: Unit vector

*Uses vector class
'''
def get_unit_vector(v):
    return scale(v, length(v))


'''
Input: 3 Number (points)

Output: Normal vector to the given plane

*Uses vector class
'''
def get_normal_given_three_points(a, b, c):
    """
    Get the unit normal vector to the
    plane defined by the points a, b, c.
    """
    ab = vector_from(a, b)
    ac = vector_from(a, c)
    v = cross(ab, ac)
    v = scale(v, length(v))
    return v

'''
Input: Vector

Output: Skewed vector

*Uses vector class
'''
def skew(v):
    return np.array([[0, -v.z, v.y], [v.z, 0, -v.x], [-v.y, v.x, 0]])