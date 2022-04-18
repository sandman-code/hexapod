import numpy as np
from comm.lx16a import *
from motion.controller import *
from hexapod.hexapodcore import Hexapod, Leg
from motion.maths import *
from comm.marlinSerialProtocol import MarlinSerialProtocol
from comm.command import *
import time

if __name__ == '__main__':
    
    #offset is counter, clockwise, clockwise
    # alpha (-90, 90)
    alpha = 0

    # beta (-70, 90)
    beta = 0

    # gamma (-90, 90)
    gamma = 0 
    
    offset = 5
    
    
    ard = serial.Serial('COM3', 115200)
    time.sleep(2)
    print(ard.is_open)
    ard.write("G28\r\n".encode())
    time.sleep(1)
    ard.write("M104 S200\r\n".encode())
    time.sleep(1)
    ard.write("G92 E0\r\n".encode())
    time.sleep(1)
    ard.write("G1 F3000\r\n".encode())
    time.sleep(1)
    ard.write("G92\r\n".encode())
    time.sleep(1)
    ard.write("G0 X20 Y20 Z20\r\n".encode())
    time.sleep(1)
    ard.write("G1 E100 F400\r\n".encode())
    time.sleep(2)
    ard.close()

    '''
    G1 F1500 ; set the feedrate to 1500 mm/min
    G92 E0
    G1 X50 Y25.3 E22.4 ; move while extruding
    '''
    #LX16A.initialize("/dev/ttyUSB0")
    #LX16A.initialize("/dev/cu.usbserial-14140")
    #LX16A.initialize("COM3")
    
    
    '''
    
    
    try:
        
        servos = (LX16A(1),
        LX16A(2),
        LX16A(3),
        LX16A(4),
        LX16A(5),
        LX16A(6),
        LX16A(7),
        LX16A(8),
        LX16A(9),
        LX16A(10),
        LX16A(11),
        LX16A(12),
        LX16A(13),
        LX16A(14),
        LX16A(15),
        LX16A(16),
        LX16A(17),
        LX16A(18))

        initial = [0,0,0]
        LEG_1 = Leg(initial, [servos[0], servos[1], servos[2]], 1)
        LEG_2 = Leg(initial, [servos[3], servos[4], servos[5]], 2)
        LEG_3 = Leg(initial, [servos[6], servos[7], servos[8]], 3)
        LEG_4 = Leg(initial, [servos[9], servos[10], servos[11]], 4)
        LEG_5 = Leg(initial, [servos[12], servos[13], servos[14]], 5)
        LEG_6 = Leg(initial, [servos[15], servos[16], servos[17]], 6)

        LEGS = (LEG_1,
        LEG_2,
        LEG_3,
        LEG_4,
        LEG_5,
        LEG_6)
    
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

    #hexy = Hexapod()
    #hexy = Hexapod(LEGS)

    #walk(hexy, 0.05)
    #generate_traj(hexy, 0.05)
    #move_leg_4(hexy, 0.05)
    
    #home(hexy)
    #print(calc_ik(hexy, [0.105, 0, -0.140]))
    #print(calc_fk(hexy.coxia, hexy.femur, hexy.tibia, (0,0,0)))
    #gen_traj_example()



#b8:27:eb:ae:db:1d
#130.215.210.179
