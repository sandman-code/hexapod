import numpy as np
from motion.maths import calc_jacobian, calc_fk
from comm.lx16a import *

if __name__ == '__main__':
    JT = calc_jacobian(0, 0, (np.pi / 2))
    torque = JT @ np.array([0,0,10])
    alpha = 240
    beta = 5
    gamma = 5

    
    LX16A.initialize("/dev/tty.usbserial-14340")
    try:
        servo_1 = LX16A(16)
    
    except ServoTimeout as e:
        print(f"Servo {e.ID} is not responding. Exiting...")
        exit()
    
    print("\n")
    print("Packets: " + "\n")
    servo_1.moveTimeWrite(alpha, 1000)
    print("\n")
    


    '''
        
    servo_1.moveTimeWrite(65, 1000)
    servo_2.moveTimeWrite(65, 1000)
    servo_3.moveTimeWrite(180, 1000)

    '''

    
    print("EE Position: " + "\n" )
    print(np.around(calc_fk([0,0,np.pi/2])))





# 5 deg offset 
# 500 == zero
# 100 == 90
# 850 