from ast import walk
from motion.maths import *
from math import cos, gamma, sin
import numpy as np
import matplotlib.pyplot as plt

def home(hexapod):

    legs = hexapod.legs

    for l in legs:
        l.move_alpha = 5
        l.move_beta = 5
        l.move_gamma = 5

    print("Robot Calibrated")

def generate_traj(hexapod, v):

    crab_angle = 60
    
    T = hexapod.stride_length / v

    
    u_fb = v / (1 - hexapod.beta)
    u_fg = u_fb - v

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

    x_max_vel = (2 * (t5 - t0) * u_fg) / ((t4 - t1) * (t3 - t2))

    z_max_vel = x_max_vel

    x_dotfg = [0, 0, x_max_vel, x_max_vel, 0, 0]
    x_foot_ground = np.zeros((6, 6))

    for i in range(6):
        x_foot_ground[i, 0] = -(hexapod.stride_length / 2)
        x_foot_ground[i, 1] = x_foot_ground[i, 0] + 0
        x_foot_ground[i, 2] = x_foot_ground[i, 1] + (t2 - t1) * (x_max_vel / 2)
        x_foot_ground[i, 3] = x_foot_ground[i, 2] + (t3 - t2) * (x_max_vel)
        x_foot_ground[i, 4] = x_foot_ground[i, 3] + (t4 - t3) * (x_max_vel / 2)
        x_foot_ground[i, 5] = x_foot_ground[i, 4] + 0

    z_dotfg = [0, z_max_vel, 0, 0, -z_max_vel, 0]
    z_foot_ground = np.zeros((6,6))

    for j in range(6):
        z_foot_ground[j, 0] = 0
        z_foot_ground[j, 1] = z_foot_ground[i, 0] + (t1 - t0) * (z_max_vel / 2)
        z_foot_ground[j, 2] = z_foot_ground[i, 1] + (t3 - t2) * (z_max_vel / 2)
        z_foot_ground[j, 3] = z_foot_ground[i, 2] + 0
        z_foot_ground[j, 4] = z_foot_ground[i, 3] - (t4 - t3) * (z_max_vel / 2)
        z_foot_ground[j, 5] = z_foot_ground[i, 4] - (t5 - t4) * (z_max_vel / 2)

    z_body_ground = np.zeros((6,6))
    
    for z in range(6):
        z_body_ground[z, 0] = hexapod.height

    D = hexapod.coxia + hexapod.femur
    alphaH = np.array([-30, 30, -90, 90, -150, 150])

    

    x_body_ground = np.zeros((6,6))

    x_body_ground[0, 0] = -((1-hexapod.beta)/2) * hexapod.stride_length - D * cos(alphaH[0])
    x_body_ground[0, 1] = -((1-hexapod.beta)/2) * hexapod.stride_length - D * cos(alphaH[1])
    x_body_ground[0, 2] = -((1-hexapod.beta)/2) * hexapod.stride_length
    x_body_ground[0, 3] = -((1-hexapod.beta)/2) * hexapod.stride_length
    x_body_ground[0, 4] = -((1-hexapod.beta)/2) * hexapod.stride_length - D * cos(alphaH[4])
    x_body_ground[0, 5] = -((1-hexapod.beta)/2) * hexapod.stride_length - D * cos(alphaH[5])

    d_t = T_t / 5

    x_dotbg = v
    z_dotbg = 0
    y_dotbg = 0 
    

    x_foot_body = np.zeros((6,6))
    z_foot_body = np.zeros((6,6))
    y_foot_body = np.array([D*sin(30), -D*sin(30), D, -D, D*sin(30), -D*sin(30)])

    x_dotfb = np.zeros((6,6))
    z_dotfb = np.zeros((6,6))

    x_foot_hip = np.zeros((6,6))
    y_foot_hip = np.zeros((6,6))
    z_foot_hip = np.zeros((6,6))

    x_dotfh = np.zeros((6,6))
    y_dotfh = np.zeros((6,6))
    z_dotfh = np.zeros((6,6))    


    y_dotfb = 0
    for l in range(6):
        
        for dot in range(6):
            x_dotfb[l, dot] = x_dotfg[dot] - x_dotbg
            z_dotfb[l, dot] = z_dotfg[dot] - z_dotbg
            
        for t in range(5):
            x_body_ground[l, t+1] = x_body_ground[l, t] + (x_dotbg * d_t)
            z_body_ground[l, t+1] = z_body_ground[l, t] + (z_dotbg * d_t)

        for n in range(6):
            x_foot_body[l, n] = z_foot_ground[l, n] - x_body_ground[l, n]
            z_foot_body[l, n] = z_foot_ground[l, n] - z_body_ground[l, n]

            x_foot_hip[l, n] = np.dot(np.array([cos(alphaH[n]), -sin(alphaH[n]),0]), np.array([[x_foot_body[l,n]], [y_foot_body[l]], [z_foot_body[l,n]]]))
            y_foot_hip[l, n] = np.dot(np.array([sin(alphaH[n]), cos(alphaH[n]),0]), np.array([[x_foot_body[l,n]], [y_foot_body[l]], [z_foot_body[l,n]]]))
            z_foot_hip[l, n] = z_foot_body[l, n]

            x_dotfh[l, n] = np.dot(np.array([cos(alphaH[n]), -sin(alphaH[n]),0]), np.array([[x_dotfb[l,n]], [y_dotfb], [z_dotfb[l,n]]]))
            y_dotfh[l, n] = np.dot(np.array([sin(alphaH[n]), cos(alphaH[n]),0]), np.array([[x_dotfb[l,n]], [y_dotfb], [z_dotfb[l,n]]]))
            z_dotfh[l, n] = z_dotfb[l,n]
    
    print("X Pos")
    print(x_foot_body)
    print("Z Pos")
    print(z_foot_body)


    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4)
    fig.suptitle("Motions of Robot")
    ax1.set_title("Time v. X-Axis Velocity")
    ax1.plot(transfer_time, x_dotfg)
    ax2.set_title("Time v. Z-Axis Velocity")
    ax2.plot(transfer_time, z_dotfg)
    ax3.set_title("Time v. X Position WRT Ground")
    ax3.plot(transfer_time, x_foot_ground[2, :])
    ax4.set_title("Time v. Z Position WRT Ground")
    ax4.plot(transfer_time, z_foot_ground[2, :])
    fig.tight_layout()
    plt.show()


    return (x_foot_hip, y_foot_hip, z_foot_hip, x_dotfh, y_dotfh, z_dotfh)




def walk(hexapod, v):

    leg1 = hexapod.legs[0]
    leg2 = hexapod.legs[1]
    leg3 = hexapod.legs[2]
    leg4 = hexapod.legs[3]
    leg5 = hexapod.legs[4]
    leg6 = hexapod.legs[5]


    points = generate_traj(hexapod, v)

    x_pos = points[0]
    y_pos = points[1]
    z_pos = points[2]

    x_vel = points[0]
    y_vel = points[1]
    z_vel = points[2]
    

    for t in range(5):

        x_coeff_1 = trajectory_planning(x_pos[0,t], x_pos[0,t+1], x_vel[0,t], x_vel[0,t+1], t, t+1)
        y_coeff_1 = trajectory_planning(y_pos[0,t], y_pos[0,t+1], y_vel[0,t], y_vel[0,t+1], t, t+1)
        z_coeff_1 = trajectory_planning(z_pos[0,t], z_pos[0,t+1], z_vel[0,t], z_vel[0,t+1], t, t+1)

        x_coeff_2 = trajectory_planning(x_pos[1,t], x_pos[1,t+1], x_vel[1,t], x_vel[1,t+1], t, t+1)
        y_coeff_2 = trajectory_planning(y_pos[1,t], y_pos[1,t+1], y_vel[1,t], y_vel[1,t+1], t, t+1)
        z_coeff_2 = trajectory_planning(z_pos[1,t], z_pos[1,t+1], z_vel[1,t], z_vel[1,t+1], t, t+1)        

        x_coeff_3 = trajectory_planning(x_pos[2,t], x_pos[2,t+1], x_vel[2,t], x_vel[2,t+1], t, t+1)
        y_coeff_3 = trajectory_planning(y_pos[2,t], y_pos[2,t+1], y_vel[2,t], y_vel[2,t+1], t, t+1)
        z_coeff_3 = trajectory_planning(z_pos[2,t], z_pos[2,t+1], z_vel[2,t], z_vel[2,t+1], t, t+1)

        x_coeff_4 = trajectory_planning(x_pos[3,t], x_pos[3,t+1], x_vel[3,t], x_vel[3,t+1], t, t+1)
        y_coeff_4 = trajectory_planning(y_pos[3,t], y_pos[3,t+1], y_vel[3,t], y_vel[3,t+1], t, t+1)
        z_coeff_4 = trajectory_planning(z_pos[3,t], z_pos[3,t+1], z_vel[3,t], z_vel[3,t+1], t, t+1)

        x_coeff_5 = trajectory_planning(x_pos[4,t], x_pos[4,t+1], x_vel[4,t], x_vel[4,t+1], t, t+1)
        y_coeff_5 = trajectory_planning(y_pos[4,t], y_pos[4,t+1], y_vel[4,t], y_vel[4,t+1], t, t+1)
        z_coeff_5 = trajectory_planning(z_pos[4,t], z_pos[4,t+1], z_vel[4,t], z_vel[4,t+1], t, t+1)

        x_coeff_6 = trajectory_planning(x_pos[5,t], x_pos[5,t+1], x_vel[5,t], x_vel[5,t+1], t, t+1)
        y_coeff_6 = trajectory_planning(y_pos[5,t], y_pos[5,t+1], y_vel[5,t], y_vel[5,t+1], t, t+1)
        z_coeff_6 = trajectory_planning(z_pos[5,t], z_pos[5,t+1], z_vel[5,t], z_vel[5,t+1], t, t+1)

        i = 0
        dt = 0.5

        while i < 1:

            alpha1 = cubic_traj(x_coeff_1, i)
            beta1 = cubic_traj(y_coeff_1, i)
            gamma1 = cubic_traj(z_coeff_1, i)

            alpha2 = cubic_traj(x_coeff_2, i)
            beta2 = cubic_traj(y_coeff_2, i)
            gamma2 = cubic_traj(z_coeff_2, i)

            alpha3 = cubic_traj(x_coeff_3, i)
            beta3 = cubic_traj(y_coeff_3, i)
            gamma3 = cubic_traj(z_coeff_3, i)

            alpha4 = cubic_traj(x_coeff_4, i)
            beta4 = cubic_traj(y_coeff_4, i)
            gamma4 = cubic_traj(z_coeff_4, i)

            alpha5 = cubic_traj(x_coeff_5, i)
            beta5 = cubic_traj(y_coeff_5, i)
            gamma5 = cubic_traj(z_coeff_5, i)

            alpha6 = cubic_traj(x_coeff_6, i)
            beta6 = cubic_traj(y_coeff_6, i)
            gamma6 = cubic_traj(z_coeff_6, i)

            
            leg1.move_alpha(alpha1, i)
            leg1.move_beta(beta1, i)
            leg1.move_gamma(gamma1, i)

            leg2.move_alpha(alpha2, i)
            leg2.move_beta(beta2, i)
            leg2.move_gamma(gamma2, i)

            leg3.move_alpha(alpha3, i)
            leg3.move_beta(beta3, i)
            leg3.move_gamma(gamma3, i)
            
            leg4.move_alpha(alpha4, i)
            leg4.move_beta(beta4, i)
            leg4.move_gamma(gamma4, i)

            leg5.move_alpha(alpha5, i)
            leg5.move_beta(beta5, i)
            leg5.move_gamma(gamma5, i)

            leg6.move_alpha(alpha6, i)
            leg6.move_beta(beta6, i)
            leg6.move_gamma(gamma6, i)

            i += dt
            

    