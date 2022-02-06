from comm.lx16a import *
from hexapod.leg import *

def init_motors():
    LX16A.initialize("COM3")
    starting_angles = [0,0,0]
    try:
        servo_1 = LX16A(1)
        servo_2 = LX16A(2)
        servo_3 = LX16A(3)
        servo_4 = LX16A(4)
        servo_5 = LX16A(5)
        servo_6 = LX16A(6)
        servo_7 = LX16A(7)
        servo_8 = LX16A(8)
        servo_9 = LX16A(9)
        servo_10 = LX16A(10)
        servo_11 = LX16A(11)
        servo_12 = LX16A(12)
        servo_13 = LX16A(13)
        servo_14 = LX16A(14)
        servo_15 = LX16A(15)
        servo_16 = LX16A(16)
        servo_17 = LX16A(17)
        servo_18 = LX16A(18)

        leg_1 = Leg(starting_angles, [servo_1, servo_2, servo_3], 1)
        leg_2 = Leg(starting_angles, [servo_4, servo_5, servo_6], 2)
        leg_3 = Leg(starting_angles, [servo_7, servo_8, servo_9], 3)
        leg_4 = Leg(starting_angles, [servo_10, servo_11, servo_12], 4)
        leg_5 = Leg(starting_angles, [servo_13, servo_14, servo_15], 5)
        leg_6 = Leg(starting_angles, [servo_16, servo_17, servo_18], 6)

        return (leg_1, leg_2, leg_3, leg_4, leg_5, leg_6)

    except ServoTimeout as e:
        print(f"Servo {e.ID} is not responding. Exiting...")
        exit()