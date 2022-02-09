import numpy as np
from motion.maths import calc_jacobian, calc_fk
from comm.lx16a import *

if __name__ == '__main__':
    JT = calc_jacobian(0, 0, (np.pi / 2))
    torque = JT @ np.array([0,0,10])

    # alpha (-90, 90)
    alpha = 0

    # beta (-70, 90)
    beta = 0

    # gamma (-90, 90)
    gamma = 0 

    offset = 5

    LX16A.initialize("/dev/cu.usbserial-14340")
    try:
        servo_1 = LX16A(16)
        servo_2 = LX16A(17)
        servo_3 = LX16A(18)
    
    except ServoTimeout as e:
        print(f"Servo {e.ID} is not responding. Exiting...")
        exit()
    
    print("\n")
    print("Packets: " + "\n")
    servo_1.moveFromCenter(alpha, 1000)
    servo_2.moveFromCenter(beta, 1000)
    servo_3.moveFromCenter(gamma, 1000)
    print("\n")
    


    '''
        
    servo_1.moveTimeWrite(65, 1000)
    servo_2.moveTimeWrite(65, 1000)
    servo_3.moveTimeWrite(180, 1000)
    '''

    




#b8:27:eb:ae:db:1d
#130.215.210.179
