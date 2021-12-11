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

#----------------------------------------------------#
# Utilizing Wave Gait                                #
#----------------------------------------------------#


class Hexapod:

    def __init__(self) -> None:
        pass


class Joint:

    def __init__(self) -> None:
        pass


class Leg:

    def __init__(self) -> None:
        pass
# 6 legs 6 points
