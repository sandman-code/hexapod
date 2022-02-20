import numpy as np
import matplotlib.pyplot as plt

def home(hexapod):

    legs = hexapod.legs

    for l in legs:
        l.move_alpha = 5
        l.move_beta = 5
        l.move_gamma = 5

    print("Robot Calibrated")

def walk(hexapod, v):

    crab_angle = 60
    
    T = hexapod.stride_length / v

    u = (hexapod.beta / (1 - hexapod.beta)) * v

    T_t = (1 - hexapod.beta) * T

    T_s = hexapod.beta * T
    
    gait_phase = np.array([0, 0.5, 0.75, 1.25, 0.5, 0])  

    t0 = 0
    t1 = T_t / 5
    t2 = 2*t1
    t3 = T_t - t2
    t4 = T_t - t1
    t5 = T_t

    transfer_time = np.array([t0, t1, t2, t3, t4, t5])

    x_max_vel = (2 * (t5 - t0) * u) / ((t4 - t1) * (t3 - t2))

    z_max_vel = x_max_vel

    x_vel = [0, 0, x_max_vel, x_max_vel, 0, 0]
    x_pos = np.zeros((6, 6))

    for i in range(5):
        x_pos[i, 0] = -(hexapod.stride_length / 2)
        x_pos[i, 1] = x_pos[i, 0] + 0
        x_pos[i, 2] = x_pos[i, 1] + (t2 - t1) * (x_max_vel / 2)
        x_pos[i, 3] = x_pos[i, 2] + (t3 - t2) * (x_max_vel)
        x_pos[i, 4] = x_pos[i, 3] + (t4 - t3) * (x_max_vel / 2)
        x_pos[i, 5] = x_pos[i, 4] + 0

    z_vel = [0, z_max_vel, 0, 0, -z_max_vel, 0]
    z_pos = np.zeros((6,6))

    for j in range(5):
        z_pos[j, 0] = 0
        z_pos[j, 1] = z_pos[i, 0] + (t1 - t0) * (z_max_vel / 2)
        z_pos[j, 2] = z_pos[i, 1] + (t3 - t2) * (z_max_vel / 2)
        z_pos[j, 3] = z_pos[i, 2] + 0
        z_pos[j, 4] = z_pos[i, 3] - (t4 - t3) * (z_max_vel / 2)
        z_pos[j, 5] = z_pos[i, 4] - (t5 - t4) * (z_max_vel / 2)

    z_pos_ground = np.zeros((6,6))
    
    for z in range(5):
        z_pos_ground[0, z] = hexapod.height

    D = hexapod.coxia + hexapod.femur
    alphaH = np.array([-30, 30, -90, 90, -150, 150])

    x_pos_ground = np.zeros((6,6))
        
    x_pos_ground[0, 0] = -((1-hexapod.beta)/2) * hexapod.stride_length - D * np.cos(alphaH[0])
    x_pos_ground[0, 1] = -((1-hexapod.beta)/2) * hexapod.stride_length - D * np.cos(alphaH[1])
    x_pos_ground[0, 2] = -((1-hexapod.beta)/2) * hexapod.stride_length
    x_pos_ground[0, 3] = -((1-hexapod.beta)/2) * hexapod.stride_length
    x_pos_ground[0, 4] = -((1-hexapod.beta)/2) * hexapod.stride_length - D * np.cos(alphaH[4])
    x_pos_ground[0, 5] = -((1-hexapod.beta)/2) * hexapod.stride_length - D * np.cos(alphaH[5])

    d_t = T_t / 5

    x_dot = v
    z_dot = 0 
    

    for l in range(5):
        for t in range(4):
            x_pos_ground[l, t+1] = x_pos_ground[l, t] + (x_dot * d_t)
            z_pos_ground[l, t+1] = z_pos_ground[l, t] + (z_dot * d_t)
        for n in range(5):
            x_pos[l, n] = x_pos_ground[l, n] - x_pos[l, n]
            z_pos[l, n] = z_pos_ground[l, n] - z_pos[l, n]
    
    print("Time Stamps")
    print(transfer_time)
    print("X Vel")
    print(x_vel)
    print("Z Vel")
    print(z_vel)
    print("X Pos")
    print(x_pos)

    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4)
    fig.suptitle("Motions of Robot")
    ax1.set_title("Time v. X-Axis Velocity")
    ax1.plot(transfer_time, x_vel)
    ax2.set_title("Time v. Z-Axis Velocity")
    ax2.plot(transfer_time, z_vel)
    ax3.set_title("Time v. X Position WRT Ground")
    ax3.plot(transfer_time, x_pos_ground[2, :])
    ax4.set_title("Time v. Z Position WRT Ground")
    ax4.plot(transfer_time, z_pos_ground[2, :])
    fig.tight_layout()
    plt.show()

