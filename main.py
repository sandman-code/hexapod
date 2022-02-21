import numpy as np
from comm.lx16a import *
from motion.controller import *
from hexapod.hexapodcore import Hexapod
from motion.maths import *

if __name__ == '__main__':
    
    #offset is counter, clockwise, clockwise
    # alpha (-90, 90)
    alpha = 0

    # beta (-70, 90)
    beta = 0

    # gamma (-90, 90)
    gamma = 0 

    offset = 5
    #LX16A.initialize("/dev/ttyUSB0")
    #LX16A.initialize("/dev/cu.usbserial-14140")

    '''
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
        
        
        servos = (servo_1,
                servo_2,
                servo_3,
                servo_4,
                servo_5,
                servo_6,
                servo_7,
                servo_8,
                servo_9,
                servo_10,
                servo_11,
                servo_12,
                servo_13,
                servo_14,
                servo_15,
                servo_16,
                servo_17,
                servo_18)
        
    except ServoTimeout as e:
        print(f"Servo {e.ID} is not responding. Exiting...")
        exit()
    
    '''

    '''
    print("\n")
    print("Packets: " + "\n")

    for s in servos:
        s.moveFromCenter(5, 1000)
    
    print("\n") 

    '''

    hexy = Hexapod()

    #walk(hexy, 0.05)
    

    print(calc_fk(hexy.coxia, hexy.tibia, hexy.femur, (30,30,30)))



    u = Vector(100, 0, 0)
    o = Vector(0, 0, 200)

    print(u.get_wrld())
    print(u.get_plat((0,0,0), o))
#b8:27:eb:ae:db:1d
#130.215.210.179
