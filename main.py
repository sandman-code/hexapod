import numpy as np
from motion.maths import calc_jacobian, calc_fk
from comm.lx16a import *

if __name__ == '__main__':
    JT = calc_jacobian(0, 0, (np.pi / 2))
    torque = JT @ np.array([0,0,10])
    alpha = 95
    beta = 5
    gamma = 95 

    


    
    print("EE Position: " + "\n" )
    print(np.around(calc_fk([0,0,np.pi/2])))





# 5 deg offset 
# 500 == zero
# 100 == 90
# 850 