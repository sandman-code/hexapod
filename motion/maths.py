import numpy as np
from math import sqrt, radians, sin, cos, degrees, acos, isnan

L1 = 29.49
L2 = 50.12
L3 = 63.94

#beta = 0.75 - wave gait

'''
u(t)_f/b = beta(t)/(1-beta(t) * v(t)
u(t)_f/g = v(t) / 1-beta(t)

x_f/g(t + del_t) = x_f/g(t) + xdot_f/g * del_t
z_f/g(t + del_t) = z_f/g(t) + zdot_f/g * del_t
'''


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

    def get_x_wrld(self):
        return 0

    def get_y_wrld(self):
        return 0
    
    def get_z_wrld(self):
        return 0

    def get_x_plat(self):
        return 0

    def get_y_plat(self):
        return 0

    def get_z_plat(self):
        return 0


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

# orr[alpha, beta, gamma]
def calc_ik_parallel(orr, hexapod):

    O = hexapod.origin_vector
    R = rotz(orr[0]) @ roty(orr[1]) @ rotx(orr[2])
    s1 = hexapod.hip_vector
    u = hexapod.leg_pos

    i = 0
    j = 0

    
    legs = hexapod.legs

    for l in legs:
        l.hip_vector = calc_leg_vector(O, R, s1[i], u[1])
        i += 1

    for l in legs:
        l.alpha = np.arctan(l.hip_vector.get_x(), l.hip_vector.get_y())

    for l in legs:
        s2 = calc_knee_joint_vector(s1[j], j, hexapod.coxa, l.alpha)
        l.knee_vector = calc_leg_vector(O, R, s2, u[j])
        j += 1
    
    for l in legs:
        #returns [phi, rho]
        # need magnitude of vector
        phi_rho = calc_intermediate(l.knee_vector, hexapod.coxa, hexapod.femur, hexapod.tibia)
        
        l.beta = np.arccos((hexapod.femur**2 + l.knee_vector - hexapod.tibia**2)/(2 * hexapod.femur * l.knee_vector.magnitude())) - (phi_rho[1] + phi_rho[0])

        l.gamma = np.pi - np.arccos((hexapod.femur**2 + hexapod.tibia**2 - l.knee_vector.magnitude()**2)/ (2 * hexapod.femur * hexapod.tibia))


def calc_ik(hexapod):
    legs = hexapod.legs

    for l in legs
        l.alpha = np.arctan(l.y/l.x)
        l.beta = np.arccos((-hexapod.tibia**2 + hexapod.femur**2 + l.x**2 + l.y**2 + l.z**2)/(2 * hexapod.femur * sqrt(l.x**2 + l.y**2 + l.z**2))) + np.arctan(l.z/sqrt(l.x**2 + l.y**2))
        l.gamma = -np.arccos((l.x**2 + l.y**2 + l.z**2 - hexapod.femur**2 - hexapod.tibia**2)/ (2 * hexpod.femur * hexapod.tibia)

# O: position vector of COR wrt ground
# R: desired orientation matrix of the platform body
# s_i: hip joint vector wrt platform body
# u_i: foot point vector wrt to ground

def calc_leg_vector(O, R, s_i, u_i):
    return O + (R @ s_i) - u_i

def calc_knee_joint_vector(s_i, i, l1, alpha)
    return Vector(s_i.get_x_plat() + ((-1)**i) * l1 * np.cos(alpha),
                  s_i.get_y_plat() + ((-1)**i) * l1 * np.sin(alpha),
                  s_i.get_z_plat())

# returns knee and ankle intermediate pair [phi, rho]
# l1 l2 l3 are leg lengths
def calc_intermediate(knee_vector, hip_vector, l1, l2, l3):
    h = hip_vector.get_z_wrld()
    h_prime = knee_vector.get_z_wrld()

    rho = np.arctan(h_prime/sqrt(knee_vector.get_x_wrld()**2 + knee_vector.get_y_wrld()**2))

    phi = np.arcsin((h_prime - h)/l1)

    return phi, rho
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

def pos_vector():
    return 0
    
def trajectory_planning(q_0, q_f, v_0, v_f, t_0, t_f):
    m = np.matrix(
            [1, t_0, t_0**2, t_0**3],
            [0, 1, 2*t_0, 3*(t_0**2)],
            [1, t_f, t_f**2, t_f**3],
            [0, 1, 2*t_f, 3*(t_f**2)])
    given = np.array(q_0, v_0, q_f, v_f)
    return np.matmul(given, np.linalg.inv(m))  

def cubic_traj(a, t):
    return a(0) + a(1)*t + a(2)*(t**2) + a(3)*(t**3)

