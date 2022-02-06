import numpy as np
from motion.maths import calc_jacobian, calc_fk
from comm.lx16a import *

if __name__ == '__main__':
    JT = calc_jacobian(0, 0, (np.pi / 2))
    torque = JT @ np.array([0,0,10])
    alpha = 0
    beta = 5
    gamma = 95 

    
    LX16A.initialize("COM3")
    try:
 
        servo_16 = LX16A(16)
        #servo_17 = LX16A(17)
        #servo_18 = LX16A(18)

    except ServoTimeout as e:
        print(f"Servo {e.ID} is not responding. Exiting...")
        exit()
    

    

 
        
    servo_16.moveTimeWrite(alpha, 1000)
    #servo_17.moveTimeWrite(beta, 1000)
    #servo_18.moveTimeWrite(gamma, 1000)


    
    print("EE Position: " + "\n" )
    print(np.around(calc_fk([0,0,np.pi/2])))





# 5 deg offset 
# 500 == zero
# 100 == 90
# 850 