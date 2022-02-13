import * from comm.lx16a.py
import hexapod from hexapodcore.py

def home(hexapod):

    legs = hexapod.legs

    for l in legs:
        l.move_alpha = 5
        l.move_beta = 5
        l.move_gamma = 5

    print("Robot Calibrated")
