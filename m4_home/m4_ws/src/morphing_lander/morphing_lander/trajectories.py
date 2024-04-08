import numpy as np 
from IPython import embed

def time_tracking(t,traj):
    return traj(t)

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
    out = np.zeros(12)
    out[0] = 2.0*np.cos(2*np.pi/5.0*t)
    out[1] = 2.0*np.sin(2*np.pi/5.0*t)
    out[2] = -3.5
    return out

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

N_traj = 100
traj_descent = np.zeros((12,N_traj))
# traj_descent[0,:] = np.linspace(-1.0,1.0,N_traj)
traj_descent[2,:] = np.linspace(-5.0,0.0,N_traj)
# traj_descent[6,:] = 1.0*np.ones((1,N_traj))
traj_descent[8,:] = 0.3*np.ones((1,N_traj))

def circle(t,z0,R,T):
    return np.array([0.0,0.0,z0]) + R*np.array([np.cos(2*np.pi/T*t),np.sin(2*np.pi/T*t),0.0])

N_circle  = 1000
T_period = 5.0 # seconds
T_maneuver = 4*T_period
R_circle  = 1.0  # meters
z0_circle = -3.5
traj_circle = np.zeros((12,N_circle))
t_ = np.linspace(0,T_maneuver,N_circle)
for i in range(N_circle):
    traj_circle[:3,i] = circle(t_[i],z0_circle,R_circle,T_period)

traj_xy = np.zeros((12,N_traj))
traj_xy[0,:] = np.linspace(-8.0,8.0,N_traj)
traj_xy[1,:] = np.linspace(-8.0,8.0,N_traj)
traj_xy[2,:] = -3.5 + 0.5*np.sin(2*np.pi/10.0*np.linspace(0.0,60.0,N_traj))
# embed()

