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


L1 = 80
L2 = 80
L3 = 100

# Side length in mm
SIDELENGTH = 90

# Max velocity in (m/sec)
VEL = 0.05

# L3 would equal the height of the robot
HEIGHT = 100

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
    LEG_1 = [0, 0, 0]
    LEG_2 = [0, 0, 0]
    LEG_3 = [0, 0, 0]
    LEG_4 = [0, 0, 0]
    LEG_5 = [0, 0, 0]
    LEG_6 = [0, 0, 0]
    
    #Store each leg in an array
    LEGS = [
        LEG_1,
        LEG_2,
        LEG_3,
        LEG_4,
        LEG_5, 
        LEG_6, 
    ]

    def __init__(self, angles) -> None:
        self.init_legs(angles)
        pass

    def init_legs(self, angles):
        for leg in self.LEGS:
            for i in range(5):
                leg = angles[i]



# 6 legs 6 points
