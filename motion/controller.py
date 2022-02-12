import * from comm.lx16a.py
import hexapod from hexapodcore.py

def home(hexapod):

    legs = hexapod.legs

    for l in legs:
        l.move_alpha = 0
        l.move_alpha = 0
        l.move_alpha = 0

    print("Robot Calibrated")
