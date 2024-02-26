import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import control as ct
from numpy import sin, cos, arctan2, sqrt, pi
from numpy.linalg import inv
from matplotlib.patches import Circle, Ellipse
from IPython import embed
import os 

# Define some line styles for later use
ebarstyle = {'elinewidth': 0.5, 'capsize': 2}
xdstyle = {'color': 'k', 'linestyle': '--', 'linewidth': 0.5, 
           'marker': '+', 'markersize': 4}

params_ = {
        'm': 5.0,               # total mass
        'g' : 9.81,             # gravitational acceleration
        'varphi': 0.5              # default tilt angle
    }

def eulzyx2rot(phi,th,psi):
    R = np.zeros((3,3))
    cx,cy,cz = cos(phi),cos(th),cos(psi)
    sx,sy,sz = sin(phi),sin(th),sin(psi)
    R[0,0] = cy*cz
    R[0,1] = cz*sx*sy-cx*sz
    R[0,2] = sx*sz+cx*cz*sy
    R[1,0] = cy*sz
    R[1,1] = cx*cz+sx*sy*sz
    R[1,2] = cx*sy*sz-cz*sx
    R[2,0] = -sy
    R[2,1] = cy*sx
    R[2,2] = cx*cy
    return R

# m4 dynamics
def m4_update(t, x_, u, params):
    # Get the parameter values
    m   = params.get('m', 5.0)            # total mass 
    g   = params.get('g',  9.81)          # gravitational acceleration
    varphi = params.get('varphi',  0.5)      # update the tilt angle

    # get states
    x, y, z, dx, dy, dz   = x_

    # get inputs (roll,pitch,yaw,thrust,horizontal force)
    phi,th,psi,T,F = u

    # dynamics
    R   = eulzyx2rot(phi,th,psi)
    pdd = 1/m * R @ np.array([0,F*sin(varphi),-T*cos(varphi)]) + np.array([0,0,g])

    return np.array([dx,dy,dz,pdd[0],pdd[1],pdd[2]])

# m4 output
def m4_output(t, x_, u, params):
    return x_

# m4 system
m4 = ct.NonlinearIOSystem(
    name='m4',
    updfcn=m4_update, outfcn=m4_output,
    states = ['x','y','z','dx','dy','dz'],
    inputs = ['phi','th','psi','T','F'],
    outputs = ['x','y','z','dx','dy','dz'],
    params = params_
)

############# VISUALIZATION ##################

# Plot the trajectory in xy coordinates
def plot_results(t, x, u, y):
    # Set the size of the figure
    # plt.figure(figsize=(10, 6))

    # T,V,H,x1,y1,x2,y2,th1,th2,th1dot,th2dot = y 

    # c1,s1 = cos(x[0]),sin(x[0])
    # c2,s2 = cos(x[1]),sin(x[1])
    # th1 = np.rad2deg(arctan2(s1,c1))
    # th2 = np.rad2deg(arctan2(s2,c2))

    plt.subplot(2,2,1)
    plt.plot(t, x[0])
    plt.xlabel('Time t [sec]')
    plt.ylabel(r'$x(t)$ [m]')


    plt.subplot(2,2,2)
    plt.plot(t, x[1])
    plt.xlabel('Time t [sec]')
    plt.ylabel(r'$y(t)$ [rad]')

    # plt.xlabel(r'$x_2(t)$ [m]')  
    # plt.ylabel(r'$y_2(t)$ [m]')  
    # plt.axis('equal')

    # # Top plot: x trajectory
    # plt.subplot(3, 1, 1)
    # plt.plot(t, th1)
    # plt.xlabel('Time t [sec]')  
    # plt.ylabel(r'$\theta_1(t)$ [deg]')

    # # Time traces of the two angles and the input
    # plt.subplot(3, 1, 2)
    # plt.plot(t, th2)
    # plt.xlabel('Time t [sec]')
    # plt.ylabel(r'$\theta_2(t)$ [deg]')

    # PLOT INPUTS
    plt.subplot(2, 2, 3)
    plt.plot(t, u[0])
    plt.xlabel('Time t [sec]')
    plt.ylabel(r'$T_1(t)$ [N]')

    plt.subplot(2, 2, 4)
    plt.plot(t, u[1])
    plt.xlabel('Time t [sec]')
    plt.ylabel(r'$T_2(t)$ [N]')

    # plt.subplot(2, 2, 4)
    # plt.plot(t,H,label=r'$H(t)$')
    # plt.plot(t,T,label=r'$T(t)$')
    # plt.plot(t,V,label=r'$V(t)$')
    # plt.xlabel('Time t [sec]')
    # plt.ylabel('Energy [J]')


    plt.suptitle("State response of double pendulum")
    plt.tight_layout()

def make_plot(i,x,y,theta,phi,r,trail_secs,max_trail,ax,params,R2,di,name,make_video):
        # Get the parameter values
        l = params.get('l', 0.3)          # arm length 
        L = params.get('L', 0.15)         # half width of chassis 
        M = params.get('M', 3.0)          # mass
        m = params.get('m', 1.0)          # spring constant
        I = params.get('I', 0.001)        # inertia of base link
        J  = params.get('J', 0.001)       # gravitational acceleration
        g  = params.get('g',  9.81)       # gravitational acceleration

        # Plot and save an image of the double pendulum configuration for time
        # point i.
        # The pendulum rods.

        L = 0.15 # half length of chassis

        R = np.array([
            [cos(theta[i]),-sin(theta[i])],
            [sin(theta[i]),cos(theta[i])]
        ])

        p = np.array([x[i],y[i]])

        # in body frame
        j1 = np.array([-L,0])
        j2 = np.array([L,0])
        p1 = j1 + np.array([-l*cos(phi[i]),-l*sin(phi[i])])
        p2 = j2 + np.array([l*cos(phi[i]),-l*sin(phi[i])])

        j1_w = p + R@j1
        j2_w = p + R@j2
        p1_w = p + R@p1
        p2_w = p + R@p2

        ax.plot([j1_w[0], j2_w[0]], [j1_w[1], j2_w[1]], lw=12, c='0.4')
        ax.plot([j1_w[0], p1_w[0]], [j1_w[1], p1_w[1]], lw=3, c='0.4')
        ax.plot([j2_w[0], p2_w[0]], [j2_w[1], p2_w[1]], lw=3, c='0.4')

        # Circles representing the anchor point of rod 1, and bobs 1 and 2.
        # c0 = Circle((x[i], y[i]), r, fc='k', zorder=10)
        c1 = Ellipse(xy=(p1_w[0], p1_w[1]), width=0.07, height=0.21,
                        angle=-90 + np.rad2deg(phi[i]) + np.rad2deg(theta[i]), 
                        edgecolor='r', fc='k', ec='k', zorder=10)
        
        c2 = Ellipse(xy=(p2_w[0], p2_w[1]), width=0.07, height=0.21,
                     angle=90 + np.rad2deg(-phi[i]) + np.rad2deg(theta[i]), 
                     edgecolor='r', fc='k', ec='k', zorder=10)
        # c1 = Circle((p1_w[0], p1_w[1]), r, fc='k', ec='k', zorder=10)
        # c2 = Circle((p2_w[0], p2_w[1]), r, fc='k', ec='k', zorder=10)
        # ax.add_patch(c0)
        ax.add_patch(c1)
        ax.add_patch(c2)

        if R2 is not None: ax.add_patch(Circle((0, 0), sqrt(R2), fc='None',ec='k', zorder=10))

        # The trail will be divided into ns segments and plotted as a fading line.
        ns = 20
        s = max_trail // ns

        for j in range(ns):
            imin = i - (ns-j)*s
            if imin < 0:
                continue
            imax = imin + s + 1
            # The fading looks better if we square the fractional length along the
            # trail.
            alpha = (j/ns)**2
            ax.plot(x[imin:imax], y[imin:imax], c='r', solid_capstyle='butt',
                    lw=2, alpha=alpha)

        # ax.plot(1.1*np.array([0,-0.69670670934]),1.1*np.array([0,0.7173560909]),'--k')
        # ax.plot(1.1*np.array([0,-cos(2*np.pi/3)]),1.1*np.array([0,sin(2*np.pi/3)]),'--k')

        # Centre the image on the fixed anchor point, and ensure the axes are equal
        ax.set_xlim(2*(-l-l-r), 2*(l+l+r))
        ax.set_ylim(4*(-l-l-r), 4*(l+l+r))
        # ax.set_xlim(x[i]-l-l-r, x[i]+l+l+r)
        # ax.set_ylim(y[i]-l-l-r, y[i]+l+l+r)
        ax.set_aspect('equal', adjustable='box')
        plt.axis('off')
        if make_video:
            plt.savefig('/Users/imandralis/src/data/animations/' + name + '/_img{:04d}.png'.format(i//di), dpi=144)

def animate(outputs,dt,params,fps=20,R2=None,name='frames',make_video=False):
    if make_video:
        os.makedirs('/Users/imandralis/src/data/animations/' + name, mode=0o777, exist_ok=False)
    # get outputs
    x,y,theta,phi =  outputs[:4]

    # Plotted bob circle radius
    r = 0.05
    # Plot a trail of the m2 bob's position for the last trail_secs seconds.
    trail_secs = 1
    # This corresponds to max_trail time points.
    max_trail = int(trail_secs / dt)

    # Make an image every di time points, corresponding to a frame rate of fps
    # frames per second.
    # Frame rate, s-1
    di = int(1/fps/dt)
    fig = plt.figure(figsize=(8.3333, 12), dpi=72)
    # fig = plt.figure(figsize=(8.3333, 6.25), dpi=72)
    ax = fig.add_subplot(111)

    for i in range(0, x.size, di):
        # print(i // di, '/', t.size // di)
        make_plot(i,x,y,theta,phi,r,trail_secs,max_trail,ax,params,R2,di,name,make_video)
        plt.draw()
        plt.pause(1/fps)
        plt.cla()


