from time import sleep, process_time_ns
from motion.maths import *
from math import cos, sin, degrees, atan, acos, sqrt 
import numpy as np
import matplotlib.pyplot as plt

def home(hexapod):

    legs = hexapod.legs

    for l in legs:
        l.move_alpha(5, 100)
        l.move_beta(-60, 100)
        l.move_gamma(70, 100)

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
            dt = (process_time_ns() - t_0) / 1000000
            coeffs = trajectory_planning(q_t[t], q_t[t+1], qd_t[t], qd_t[t], 0, dt)
            print(coeffs)
            alpha = degrees(cubic_traj(coeffs[0], dt))
            beta = degrees(cubic_traj(coeffs[1], dt))
            gamma = degrees(cubic_traj(coeffs[2], dt))
            leg4.move_alpha(int(alpha), dt)
            leg4.move_beta(int(beta), dt)
            leg4.move_gamma(int(gamma), dt)        
    
def gen_traj_example():
    l1 = 0.02
    l2 = 0.05
    l3 = 0.1

    b = 0.75
    v = 0.03
    #u = 0.09 avg velocity of the foot tip in swing phase (0.1)

    w = 0 # rotational velocity of COG
    R = 0.22 # distance of foot tip to COG
    u = (b/(1-b))*v
    u = v/(1-b)
    a = np.pi / 6
    L = 0.03 #stride length
    T = L/v # Cycle period
    Tt = (1-b)*T
    Ts = b*T

    #------------- Gait Gen ---------------------
    p = [0, 0, 0, 0, 0, 0]
    p[0] = 0
    p[1] = p[0] + (1/2)
    p[2] = p[0] + b
    p[3] = p[1] + b
    p[4] = p[2] + b
    p[5] = p[3] + b

    for j in range(6):
        for i in range(6):
            if p[i] >= 1:
                p[i] = p[i] - 1
    
    #----------- Trajectory Planning -------------
    t0 = 0
    t1 = Tt/5
    t2 = 2*t1
    t3 = Tt - t2
    t4 = Tt - t1
    t5 = Tt

    Ttt = [t0, t1, t2, t3, t4, t5]

    xdotmaxf_g = 2 * (t5 - t0) * u / ((t4 - t1) + (t3 - t2))
    zdotmaxf_g = xdotmaxf_g

    xdotf_g = [0,0,xdotmaxf_g,xdotmaxf_g,0,0]

    xf_g = np.zeros((6,6))
    for x in range(6):
        xf_g[x,0] = -L/2
        xf_g[x,1] = xf_g[x,0] + 0
        xf_g[x,2] = xf_g[x,1] + (t2-t1) * xdotmaxf_g/2
        xf_g[x,3] = xf_g[x,2] + (t3-t2) * xdotmaxf_g
        xf_g[x,4] = xf_g[x,3] + (t4-t3) * xdotmaxf_g/2
        xf_g[x,5] = xf_g[x,4] + 0

    zdotf_g = [0, zdotmaxf_g, 0, 0, -zdotmaxf_g, 0]

    zf_g = np.zeros((6,6))
    for z in range(6):
        zf_g[z,0] = 0
        zf_g[z,1] = zf_g[z,0] + (t1-t0)*zdotmaxf_g/2
        zf_g[z,2] = zf_g[z,1] + (t2-t1)*zdotmaxf_g/2
        zf_g[z,3] = zf_g[z,2] + 0
        zf_g[z,4] = zf_g[z,3] - (t4-t3)*zdotmaxf_g/2
        zf_g[z,5] = zf_g[z,4] - (t5-t4)*zdotmaxf_g/2

    #---- Updating coordinate system

    h = 0.1
    D = l1+l2

    alphaH = [-30*np.pi/180, 30*np.pi/180, -90*np.pi/180, 90*np.pi/180, -150*np.pi/180, 150*np.pi/180]
    
    xb_g = np.zeros((6,6))
    xb_g[0,0] = -((1-b)/2)*L-D*cos(alphaH[0])
    xb_g[1,0] = -((1-b)/2)*L-D*cos(alphaH[1])
    xb_g[2,0] = -((1-b)/2)*L
    xb_g[3,0] = -((1-b)/2)*L
    xb_g[4,0] = -((1-b)/2)*L-D*cos(alphaH[4])
    xb_g[5,0] = -((1-b)/2)*L-D*cos(alphaH[5])

    zb_g = np.zeros((6,6))
    zb_g[0,0] = h
    zb_g[1,0] = h
    zb_g[2,0] = h
    zb_g[3,0] = h
    zb_g[4,0] = h
    zb_g[5,0] = h

    dt = Tt/5
    xdotb_g = v
    zdotb_g = 0

    xf_b = np.zeros((6,6))
    zf_b = np.zeros((6,6))
    for n in range(6):
        for t in range(5):
            xb_g[n,t+1] = xb_g[n,t] + xdotb_g*dt
            zb_g[n,t+1] = zb_g[n,t] + zdotb_g*dt
        for t2 in range(6):
            xf_b[n,t2] = xf_g[n,t2] - xb_g[n,t2]
            zf_b[n,t2] = zf_g[n,t2] - zb_g[n,t2]

    yf_b=[D*sin(30*np.pi/180), -D*sin(30*np.pi/180), D, -D, D*sin(30*np.pi/180), -D*sin(30*np.pi/180)]
    xf_H = np.zeros((6,6))
    yf_H = np.zeros((6,6))
    zf_H = np.zeros((6,6))

    for i2 in range(6):
        for j2 in range(6):
            xf_H[i2,j2] = np.array([cos(alphaH[i2]), -sin(alphaH[i2]), 0]).dot(np.array([[xf_b[i2,j2]], [yf_b[i2]], [zf_b[i2,j2]]]))
            yf_H[i2,j2] = np.array([sin(alphaH[i2]), cos(alphaH[i2]), 0]).dot(np.array([[xf_b[i2,j2]], [yf_b[i2]], [zf_b[i2,j2]]]))
            zf_H[i2,j2] = zf_b[i2,j2]

    print(xf_H)
    print(yf_H)
    print(zf_H)
    plot_tip_points(xf_H, yf_H, zf_H)
    '''
    plt.subplot(3,2,1)
    plt.plot(Ttt, xdotf_g)
    plt.subplot(3,2,2)
    plt.plot(Ttt, zdotf_g)
    
    plt.subplot(3,2,3)
    plt.plot(Ttt,xf_g[3,:])
    plt.subplot(3,2,4)
    plt.plot(Ttt,zf_g[3,:])

    plt.subplot(3,2,5)
    plt.plot(xf_g[3,:], zf_g[3,:])

    plt.subplot(3,2,6)
    plt.plot(xf_b[3,:], zf_b[3,:])
    plt.show()
    '''
    #----- Inverse Kinematics ---------

    Alpha0 = 0
    Beta0 = np.arctan(h/l2)
    Gamma0 = 90*np.pi/180

    Alpha = np.zeros((6,6))
    Beta = np.zeros((6,6))
    Gamma = np.zeros((6,6))

    l = np.zeros((6,6))
    d = np.zeros((6,6))
    for i3 in range(6):
        for j3 in range(6):
            Alpha[i3,j3] = atan(yf_H[i3,j3]/xf_H[i3,j3])
            l[i3,j3] = sqrt((yf_H[i3,j3])**2 + (xf_H[i3,j3])**2)
            d[i3,j3] = sqrt((zf_H[i3,j3])**2 + (l[i3,j3]-l1)**2)
            Beta[i3,j3] = acos(((l2)**2 + (d[i3,j3])**2 - (l3)**2)/(2*l2*d[i3,j3])) - atan(abs(zf_H[i3,j3])/(l[i3,j3]-l1))
            Gamma[i3,j3] = np.pi - (acos(((l2)**2 + (l3)**2 - (d[i3,j3])**2)/(2*l2*l3)))
            Beta[i3,j3] = Beta[i3,j3] * 180/np.pi
            Gamma[i3,j3] = np.pi - Gamma[i3,j3]
    
    A = Alpha*(180/np.pi)
    B = Beta*(180/np.pi)
    G = Gamma*(180/np.pi)



def plot_tip_points(x_mat, y_mat, z_mat):
    l2 = 0.05
    l3 = 0.1

    x = x_mat[0]
    y = y_mat[0]
    z = z_mat[0]
    
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.scatter3D(np.concatenate((x,[0])), np.concatenate((y,[0])), np.concatenate((z,[0])))
    plt.show()
    

