from __future__ import print_function, division
import numpy as np 
from morphing_lander.mpc.parameters import params_
import morphing_lander.mpc.quadtraj as quadtraj
from morphing_lander.mpc.utils import create_interpolators, interpolate_values

# get parameters
v_max_absolute             = params_.get('v_max_absolute')
m                          = params_.get('m')
g                          = params_.get('g')
T_max                      = params_.get('T_max')
u_max                      = params_.get('u_max')
emergency_descent_velocity = params_.get('emergency_descent_velocity')
land_height                = params_.get('land_height')
z0                         = params_.get('z0')
zf                         = params_.get('zf')

# Define how gravity lies:
gravity = [0,0,9.81]

# Define the trajectory starting state:
pos0 = [0, 0, z0] #position
vel0 = [0, 0, 0] #velocity
acc0 = [0, 0, 0] #acceleration

# Define the second state:
pos1 = [0, 0, -1.5]  # position
vel1 = [0, 0, 0]  # velocity
acc1 = [0, 0, 0]  # acceleration

# Define the third state
pos2 = [3, 0, zf]  # position
vel2 = [2.0, 0, 0.0]  # velocity
acc2 = [0, 0, 0]  # acceleration

# Define the duration:
T1 = 5.0
T2 = 5.0
t_tilt = T1 + 0.6*(T2-T1)
 
# traj0
traj0 = quadtraj.RapidTrajectory(pos0, vel0, acc0, gravity)
traj0.set_goal_position(pos1)
traj0.set_goal_velocity(vel1)
traj0.set_goal_acceleration(acc1)
traj0.generate(T1)

# traj1
traj1 = quadtraj.RapidTrajectory(pos1, vel1, acc1, gravity)
traj1.set_goal_position(pos2)
traj1.set_goal_velocity(vel2)
traj1.set_goal_acceleration(acc2)
traj1.generate(T2)

def traj_minjerk(t):
    tilt_vel = 0.0
    if t < T1:
        pos = traj0.get_position(t)
        vel = traj0.get_velocity(t)
        rates = traj0.get_body_rates(t)
        thrust = traj0.get_thrust(t)
    elif (t>T1) and (t<T1 + T2):
        pos = traj1.get_position(t-T1)
        vel = traj1.get_velocity(t-T1)
        rates = traj1.get_body_rates(t-T1)
        thrust = traj1.get_thrust(t-T1)
        if t > t_tilt:
            tilt_vel = 1.0
    else:
        pos    = traj1.get_position(T2)
        vel    = traj1.get_velocity(T2)
        rates  = traj1.get_body_rates(T2)
        thrust = traj1.get_thrust(T2)

    # write reference 
    x_ref = np.zeros(12)
    x_ref[0] = pos[0]
    x_ref[1] = pos[1]
    x_ref[2] = pos[2]
    x_ref[6] = vel[0]
    x_ref[7] = vel[1]
    x_ref[8] = vel[2]
    x_ref[9] = rates[0]
    x_ref[10] = rates[1]
    x_ref[11] = rates[2]
    u_ref = thrust/T_max*np.ones(4)
    done = False
    return x_ref,u_ref,tilt_vel,done

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

def traj_fly_up(t):
    done = False
    H = -1.5
    v_up = -0.5
    z0 = 0.0

    t1 = H/v_up
    
    if (t<t1): 
        z, dz    = z0 + v_up*t, v_up
    else:
        z,dz = z0 + H, 0.0
        done = True

    x_ref = np.zeros(12)
    x_ref[2] = z
    x_ref[8] = dz
    u_ref = m*g/T_max*np.ones(4)
    tilt_vel = 0.0
    return x_ref,u_ref,tilt_vel, done

def traj_jump_time(t):

    done = False

    H         = -1.5          # 1.5 m 
    H_down    =  H - zf
    v_up      = -0.50
    v_down    =  0.30 
    v_forward =  0.75          # 0.0

    t1 = H/v_up
    t2 = t1 + 5.0
    t3 = t2 + (-H_down/v_down)
    t_tilt = 0.5*t2 + 0.5*t3
    
    if (t<t1): 
        x, dx    = 0.0, 0.0
        z, dz    = z0 + v_up*t, v_up
        tilt_vel = 0.0
    elif ((t>t1) and (t<t2)):
        x, dx    = 0.0, 0.0
        z, dz    = z0 + H, 0.0
        tilt_vel = 0.0
    elif ((t>t2) and (t<t3)):
        x, dx    = v_forward*(t-t2), v_forward
        z,dz = z0 + H + v_down * (t-t2), v_down
        tilt_vel = 0.0
        if t>t_tilt:
            tilt_vel = 1.0
    elif (t>t3):
        x,dx = v_forward*(t3-t2),0.0
        z,dz = zf,0.0
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


# # Create interpolators
# x_opt_vec = np.load('/home/m4pc/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/optimal_trajectory/x_vec.npy')
# u_opt_vec = np.load('/home/m4pc/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/optimal_trajectory/u_vec.npy')
# t_opt_vec = np.load('/home/m4pc/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/optimal_trajectory/t_vec.npy')
# N_opt_horizon = np.load('/home/m4pc/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/optimal_trajectory/N_horizon.npy')
# T_opt_horizon = np.load('/home/m4pc/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/optimal_trajectory/T_horizon.npy')
# x_interpolator, u_interpolator = create_interpolators(t_opt_vec, x_opt_vec, u_opt_vec)



# def traj_jump_time_optimal(t):

#     done = False

#     z0 = 0.0
#     zf = 0.0

#     H = -1.5
#     v_up = -0.5

#     t1 = H/v_up
#     t2 = t1 + T_opt_horizon
    

#     x_ref = np.zeros(12)
#     u_ref = m*g/T_max*np.ones(4)

#     if (t<t1): 
#         # ascend in a linear trajectory
#         z, dz    = z0 + v_up*t, v_up

#         # write references
#         x_ref[2] = z
#         x_ref[8] = dz
#         tilt_vel = 0.0

#     elif ((t>t1) and (t<t2)):
#         # get reference values from optimal trajectory
#         x_ref_temp, u_ref_temp = interpolate_values(x_interpolator, u_interpolator, t-t1)

#         # interpolate reference values
#         x_ref, u_ref = x_ref_temp[:-1], u_ref_temp[:-1]
#         tilt_vel = u_ref_temp[-1]

#     elif (t>t2):
#         z,dz = zf,0.0

#         # write references
#         x_ref[2] = z
#         x_ref[8] = dz
#         tilt_vel = 0.0

#         # trajectory done
#         done = True

#     return x_ref,u_ref,tilt_vel, done


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

