import numpy as np 
from IPython import embed
from morphing_lander.parameters import params_

v_max_absolute             = params_.get('v_max_absolute')
m                          = params_.get('m')
g                          = params_.get('g')
T_max                      = params_.get('T_max')
u_max                      = params_.get('u_max')
emergency_descent_velocity = params_.get('emergency_descent_velocity')

def spatial_tracking(X,traj):
    p = X[:3]
    gamma = np.inf
    X_im = traj[:,0]
    for i in range(1,traj.shape[1]):
        X_i = traj[:,i]
        p_i = X_i[:3]
        p_im = X_im[:3]
        d = p_i - p_im
        gamma = np.dot((p-p_im),d/np.linalg.norm(d)**2)
        X_im = X_i
        if gamma < 1.0:
            break
    X_ref = X_im + gamma*(X_i - X_im)
    return X_ref

def traj_jump_time(t):

    done = False

    H = -1.5
    v_up = -0.5
    v_down = 0.30
    v_forward = 0.0

    t1 = H/v_up
    t2 = t1 - H/v_down
    
    phi_max = np.deg2rad(50)

    if (t<t1): 
        x, dx    = v_forward*t, v_forward
        z, dz    = v_up*t, v_up
        phi,dphi = 0.0,0.0
        tilt_vel = 0.0
    elif ((t>t1) and (t<t2)):
        x, dx    = v_forward*t, v_forward
        z,dz = H + v_down * (t-t1), v_down
        phi,dphi = phi_max*(t-t1)/(t2-t1), phi_max/(t2-t1)
        tilt_vel = 1.0
    elif (t>t2):
        x,dx = v_forward*t2,0.0
        z,dz = 0.0,0.0
        phi,dphi = phi_max,phi_max/(t2-t1)
        tilt_vel = 1.0
        done = True

    x_ref = np.zeros(12)
    x_ref[0] = x
    x_ref[2] = z
    x_ref[6] = dx
    x_ref[8] = dz

    u_ref = m*g/T_max*np.ones(4)
    return x_ref,u_ref,tilt_vel, done

def traj_descent_time(t):
    out = np.zeros(12)
    # out[0] = 0.1*np.cos(t)
    # out[1] = 0.1*np.sin(t)
    out[2] = -2.0 + t*0.5
    out[8] = 0.5
    if out[2] >= 0.0 : 
        out[2] = 0.0
        out[0] = 4.0*1.0
    else:
        out[0] = t*1.0
    return out

def traj_circle_time(t):
    done = False 
    if t > 5.0 : done = True

    x_ref = np.zeros(12)
    x_ref[0] = 2.0*np.cos(2*np.pi/5.0*t)
    x_ref[1] = 2.0*np.sin(2*np.pi/5.0*t)
    x_ref[2] = -3.5

    u_ref = m*g/T_max*np.ones(4)
    tilt_vel = 0.0
    return x_ref,u_ref,tilt_vel, done

def emergency_descent_time(t,p_emergency):
    x_ref = np.zeros(12)
    x_ref[0] = p_emergency[0]
    x_ref[1] = p_emergency[1]
    x_ref[2] = p_emergency[2] + t*emergency_descent_velocity
    x_ref[8] = emergency_descent_velocity
    if x_ref[2] >= 0.0 : 
        x_ref[2] = 0.0
    u_ref = m*g/T_max*np.ones(4)
    return x_ref,u_ref

def emergency_descent(x_current,p_emergency):
    N_traj = 100
    traj_emergency = np.zeros((12,N_traj))
    traj_emergency[0,:] = p_emergency[0]*np.ones(N_traj)
    traj_emergency[1,:] = p_emergency[1]*np.ones(N_traj)
    traj_emergency[2,:] = np.linspace(p_emergency[2]-1.0,0.0,N_traj)
    traj_emergency[8,:] = 0.3*np.ones((1,N_traj))
    x_ref = spatial_tracking(x_current,traj_emergency)
    u_ref = m*g/T_max*np.ones(4,dtype='float')
    tilt_vel = -u_max
    return x_ref,u_ref,tilt_vel




# N_traj = 100
# traj_descent = np.zeros((12,N_traj))
# # traj_descent[0,:] = np.linspace(-1.0,1.0,N_traj)
# traj_descent[2,:] = np.linspace(-5.0,0.0,N_traj)
# # traj_descent[6,:] = 1.0*np.ones((1,N_traj))
# traj_descent[8,:] = 0.3*np.ones((1,N_traj))

# def circle(t,z0,R,T):
#     return np.array([0.0,0.0,z0]) + R*np.array([np.cos(2*np.pi/T*t),np.sin(2*np.pi/T*t),0.0])

# N_circle  = 1000
# T_period = 5.0 # seconds
# T_maneuver = 4*T_period
# R_circle  = 1.0  # meters
# z0_circle = -3.5
# traj_circle = np.zeros((12,N_circle))
# t_ = np.linspace(0,T_maneuver,N_circle)
# for i in range(N_circle):
#     traj_circle[:3,i] = circle(t_[i],z0_circle,R_circle,T_period)

# traj_xy = np.zeros((12,N_traj))
# traj_xy[0,:] = np.linspace(-8.0,8.0,N_traj)
# traj_xy[1,:] = np.linspace(-8.0,8.0,N_traj)
# traj_xy[2,:] = -3.5 + 0.5*np.sin(2*np.pi/10.0*np.linspace(0.0,60.0,N_traj))

