# --------- Structure of Hexapod --------------------#
#            ____
#          1      2
#        /          \
#       3            4
#        \          /
#          5 ____ 6
#
#
#


#-------------- CONSTANTS ---------------------------#

# All lengths are in mm
import numpy as np
from motion.maths import *

L1 = 65
L2 = 40
L3 = 140

# Side length in mm
SIDELENGTH = 90

# Max velocity in (m/sec)
VEL = 0.05

# L3 would equal the height of the robot
HEIGHT = 200

# Duty factor of a wave gait
DUTYFACTOR = 0.75

# Vectors from the center of the robot to each joint 

# Coxa
# Given
S1 = 0

# Tibia
'''
Given by this equation: 

S2 = [
    S1_x + (-1)^leg_num * L1 * cos(alpha)
    S1_y + (-1)^leg_num * L1 * sin(alpha)
    S1_z
]
'''
S2 = 0

#Gamma
S3 = 0

# Vector from the ground frame to the body frame
O = 0


R = 0

U = 0

#----------------------------------------------------#
# Utilizing Wave Gait                                #
#----------------------------------------------------#



L_I = O + R * S1 - U

class Hexapod:

    #Each leg has 3 angles (COXIA, FEMUR, TIBIA)
    coxia = 65 / 1000
    femur = 40 / 1000
    tibia = 140 / 1000

    # Max velocity in (m/sec)
    max_vel = 0.05

    # L3 would equal the height of the robot
    height = 200 / 1000

    # Duty factor of a wave gait
    beta = 0.75

    # Vector from the center to the hip
    hip_vector = 185 / 1000

    # Stride length of each leg
    stride_length = 100 / 1000

    # Max height of the leg for the transfer phase
    leg_height_max = 50 / 1000

    def __init__(self):
        self.origin_vector = Vector(0, 0, 200)
        self.orientation = [0,0,0]

    
    def init_legs(self, motors):
        initial = [0,0,0]
        for x in range(5):
            self.legs.append(Leg(initial, motors[x]))

        return legs

    def update_orientation(R):
        self.orientation = R

    def update_origin(O):
        self.origin_vector = O
        
# 6 legs 6 points
