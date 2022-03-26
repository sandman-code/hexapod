from time import sleep, process_time_ns
from motion.maths import *
from math import cos, sin 
import numpy as np
import matplotlib.pyplot as plt

def home(hexapod):

    legs = hexapod.legs

    for l in legs:
        l.move_alpha(0, 100)
        l.move_beta(0, 100)
        l.move_gamma(0, 100)

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

    x_max_vel = (2 * (t5 - t0) * u_fg) / ((t4 - t1) + (t3 - t2))

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
        z_foot_ground[j, 1] = z_foot_ground[j, 0] + (t1 - t0) * (z_max_vel / 2)
        z_foot_ground[j, 2] = z_foot_ground[j, 1] + (t2 - t1) * (z_max_vel / 2)
        z_foot_ground[j, 3] = z_foot_ground[j, 2] + 0
        z_foot_ground[j, 4] = z_foot_ground[j, 3] - (t4 - t3) * (z_max_vel / 2)
        z_foot_ground[j, 5] = z_foot_ground[j, 4] - (t5 - t4) * (z_max_vel / 2)

    z_body_ground = np.zeros((6,6))
    
    for z in range(6):
        z_body_ground[z, 0] = hexapod.tibia

    D = hexapod.coxia + hexapod.femur
    alphaH = np.array([radians(-30), radians(30), radians(-90), radians(90), radians(-150), radians(150)])

    

    x_body_ground = np.zeros((6,6))

    x_body_ground[0, 0] = -((1-hexapod.beta)/2) * hexapod.stride_length - D * cos(alphaH[0])
    x_body_ground[1, 0] = -((1-hexapod.beta)/2) * hexapod.stride_length - D * cos(alphaH[1])
    x_body_ground[2, 0] = -((1-hexapod.beta)/2) * hexapod.stride_length
    x_body_ground[3, 0] = -((1-hexapod.beta)/2) * hexapod.stride_length
    x_body_ground[4, 0] = -((1-hexapod.beta)/2) * hexapod.stride_length - D * cos(alphaH[4])
    x_body_ground[5, 0] = -((1-hexapod.beta)/2) * hexapod.stride_length - D * cos(alphaH[5])

    d_t = T_t / 5

    x_dotbg = v
    z_dotbg = 0
    y_dotbg = 0 
    

    x_foot_body = np.zeros((6,6))
    z_foot_body = np.zeros((6,6))
    y_foot_body = np.array([D*sin(radians(30)), -D*sin(radians(30)), D, -D, D*sin(radians(30)), -D*sin(radians(30))])

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
            x_foot_body[l, n] = x_foot_ground[l, n] - x_body_ground[l, n]
            z_foot_body[l, n] = z_foot_ground[l, n] - z_body_ground[l, n]

            x_foot_hip[l, n] = np.dot(np.array([cos(alphaH[n]), -sin(alphaH[n]),0]), np.array([[x_foot_body[l,n]], [y_foot_body[l]], [z_foot_body[l,n]]]))
            y_foot_hip[l, n] = np.dot(np.array([sin(alphaH[n]), cos(alphaH[n]),0]), np.array([[x_foot_body[l,n]], [y_foot_body[l]], [z_foot_body[l,n]]]))
            z_foot_hip[l, n] = z_foot_body[l, n]

            x_dotfh[l, n] = np.dot(np.array([cos(alphaH[n]), -sin(alphaH[n]),0]), np.array([[x_dotfb[l,n]], [y_dotfb], [z_dotfb[l,n]]]))
            y_dotfh[l, n] = np.dot(np.array([sin(alphaH[n]), cos(alphaH[n]),0]), np.array([[x_dotfb[l,n]], [y_dotfb], [z_dotfb[l,n]]]))
            z_dotfh[l, n] = z_dotfb[l,n]
    

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

    fig2, (ax5, ax6, ax7) = plt.subplots(3)
    ax5.set_title("X v. Z Position WRT Ground")
    ax5.plot(x_foot_ground[2,:], z_foot_ground[2,:])
    ax6.set_title("X v. Z Position WRT Body")
    ax6.plot(x_foot_body[2,:], z_foot_body[2,:])
    ax7.set_title("X Position of Body")
    ax7.plot(transfer_time, x_body_ground[1,:])

    fig2.tight_layout()
    fig.tight_layout()
    plt.show()

    print("X Foot Hip")
    print(x_foot_hip)

    print("Y Foot Hip")
    print(y_foot_hip)

    print("Z Foot Hip")
    print(z_foot_hip)
    return (x_foot_hip, y_foot_hip, z_foot_hip, x_dotfh, y_dotfh, z_dotfh)




def walk(hexapod, v):

    points = generate_traj(hexapod, v)

    x_pos = points[0]
    y_pos = points[1]
    z_pos = points[2]
    
    q_t = np.empty((6,6), dtype=object)
    qd_t = np.empty((6,6), dtype=object)

    for l in range(6):
        for t in range(6):
            q_t[l, t] = calc_ik(hexapod, [x_pos[l, t], y_pos[l, t], z_pos[l, t]]) 

    for l2 in range(6):
        for t in range(6):
            angles = q_t[l2, t]
            J = calc_jacobian(hexapod, angles[0], angles[1], angles[2])
            print(J)
            qd_t[l2, t] = np.matmul(np.linalg.inv(J), np.transpose(angles))
    
    print(q_t)
    print(qd_t)
    

    for t in range(5):
        t_0 = process_time_ns()
        for a in range(6):
            while dt < 10000:
                dt = (process_time_ns - t_0) / 1000000
                curr_leg = hexapod.legs[a]
                coeffs = trajectory_planning(q_t[a,t], q_t[a, t+1], qd_t[a,t], qd_t[a,t], 0, dt)
                alpha = cubic_traj(coeffs[0], dt)
                beta = cubic_traj(coeffs[1], dt)
                gamma = cubic_traj(coeffs[2], dt)
                curr_leg.move_alpha(alpha, dt)
                curr_leg.move_beta(beta, dt)
                curr_leg.move_gamma(gamma, dt)


def move_leg_4(hexapod, v):

    points = generate_traj(hexapod, v)

    x_pos = points[0]
    y_pos = points[1]
    z_pos = points[2]
    
    q_t = np.empty(6, dtype=object)
    qd_t = np.empty(6, dtype=object)
    dt = 0

    for t in range(6):
        q_t[t] = calc_ik(hexapod, [x_pos[3, t], y_pos[3, t], z_pos[3, t]])
        print (q_t[t])

    for t in range(6):
        angles = q_t[t]
        J = calc_jacobian(hexapod, angles[0], angles[1], angles[2])
        print(J)
        qd_t[t] = np.matmul(np.linalg.inv(J), np.transpose(angles))
            
    for t in range(5):
        t_0 = process_time_ns()
        while dt < 10000:
            leg4 = hexapod.legs[3]
            dt = (process_time_ns - t_0) / 1000000
            coeffs = trajectory_planning(q_t[t], q_t[t+1], qd_t[t], qd_t[t], 0, dt)
            alpha = cubic_traj(coeffs[0], dt)
            beta = cubic_traj(coeffs[1], dt)
            gamma = cubic_traj(coeffs[2], dt)
            leg4.move_alpha(alpha, dt)
            leg4.move_beta(beta, dt)
            leg4.move_gamma(gamma, dt)        
    
