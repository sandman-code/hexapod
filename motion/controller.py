from ast import walk
from time import time
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

    dt = 1
    points = generate_traj(hexapod, v)

    x_pos = points[0]
    y_pos = points[1]
    z_pos = points[2]

    x_vel = points[0]
    y_vel = points[1]
    z_vel = points[2]
    
    q_t = np.empty((6,6), dtype=object)
    qd_t = np.empty((6,6), dtype=object)

    for l in range(6):
        for t in range(6):
            q_t[l, t] = calc_ik(hexapod, [x_pos[l, t], y_pos[l, t], z_pos[l, t]]) 

    for l2 in range(6):
        for t in range(6):
            angles = q_t[l2, t]
            J = calc_jacobian([angles[0], angles[1], angles[2]])
            qd_t[l2, t] = np.matmul(np.inverse(J), np.transpose(angles))
    
    for t in range(5):
        for a in range(6):
            curr_leg = hexapod.legs[a]
            coeffs = trajectory_planning(q_t[a,t], q_t[a, t+1], qd_t[a,t], qd_t[a,t], 0, dt)
            alpha = cubic_traj(coeffs[0], dt)
            beta = cubic_traj(coeffs[1], dt)
            gamma = cubic_traj(coeffs[2], dt)
            curr_leg.move_alpha(alpha, dt)
            curr_leg.move_beta(beta, dt)
            curr_leg.move_gamma(gamma, dt)
            time.sleep(1)
            

    