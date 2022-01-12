import numpy as np
from motion.maths import calc_jacobian, calc_fk
if __name__ == '__main__':
    JT = calc_jacobian(0, 0, (np.pi / 2))
    torque = JT @ np.array([0,0,10])
    print(np.around(calc_fk([0,0,np.pi/2])))




