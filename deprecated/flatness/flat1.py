import numpy as np
import control as ct
import control.flatsys as fs
import control.optimal as opt 
import time
import matplotlib.pyplot as plt
from IPython import embed

# Plot the trajectory in z,phi coordinates
def plot_results(t, x, u, phi_c, V_safe, lb=None, ub=None):
    # Set the size of the figure
    plt.figure(figsize=(10, 6))

    # Time traces of the state and input
    plt.subplot(1, 5, 1)
    plt.plot(t, x[0])
    plt.xlabel('Time t [sec]')
    plt.ylabel('z [m]')
    if not ((lb is None) and (ub is None)):
        plt.plot(t,np.ones_like(t)*lb[0],'--k')
        plt.plot(t,np.ones_like(t)*ub[0],'--k')

    plt.subplot(1, 5, 2)
    plt.plot(t, x[1])
    plt.xlabel('Time t [sec]')
    plt.ylabel('zdot [m/s]')
    if not ((lb is None) and (ub is None)):
        plt.plot(t,np.ones_like(t)*lb[1],'--k')
        plt.plot(t,np.ones_like(t)*ub[1],'--k')
        plt.plot(t,np.ones_like(t)*V_safe,'--r')

    plt.subplot(1, 5, 3)
    plt.plot(t, np.rad2deg(x[2]))
    plt.xlabel('Time t [sec]')
    plt.ylabel('phi [deg]')
    if not ((lb is None) and (ub is None)):
        plt.plot(t,np.rad2deg(np.ones_like(t)*lb[2]),'--k')
        plt.plot(t,np.rad2deg(np.ones_like(t)*ub[2]),'--k')
        plt.plot(t,np.rad2deg(np.ones_like(t)*phi_c),'--r')


    plt.subplot(1, 5, 4)
    plt.plot(t, u[0])
    plt.xlabel('Time t [sec]')
    plt.ylabel('T [N]')
    if not ((lb is None) and (ub is None)):
        plt.plot(t,np.ones_like(t)*lb[3],'--k')
        plt.plot(t,np.ones_like(t)*ub[3],'--k')

    plt.subplot(1, 5, 5)
    plt.plot(t, u[1])
    plt.xlabel('Time t [sec]')
    plt.ylabel('v[rad/s]')
    if not ((lb is None) and (ub is None)):
        plt.plot(t,np.ones_like(t)*lb[4],'--k')
        plt.plot(t,np.ones_like(t)*ub[4],'--k')

    plt.tight_layout()

# Function to take states, inputs and return the flat flag
def vehicle_flat_forward(x, u, params={}):
    # Get the parameter values
    m = params.get('m', 5.4)
    g = params.get('g', 9.81)

    # Create a list of arrays to store the flat output and its derivatives
    zflag = [np.zeros(3), np.zeros(3)]

    # Flat outputs are (z,phi)
    zflag[0][0] = x[0]
    zflag[1][0] = x[2]

    # First derivatives of the flat output
    zflag[0][1] = x[1]  # zdot
    zflag[1][1] = u[1]  # v

    # Second derivatives of the flat output
    zflag[0][2] = g - 1.0/m * np.cos(x[2]) * u[0]
    zflag[1][2] =  0

    return zflag

# Function to take the flat flag and return states, inputs
def vehicle_flat_reverse(zflag, params={}):
    # Get the parameter values
    m = params.get('m', 5.4)
    g = params.get('g', 9.81)

    # Create a vector to store the state and inputs
    x = np.zeros(3)
    u = np.zeros(2)

    # Given the flat variables, solve for the state
    x[0] = zflag[0][0]  # z
    x[1] = zflag[0][1]  # zdot
    x[2] = zflag[1][0]  # phi

    # And next solve for the inputs
    u[0] = m * (g - zflag[0][2])/np.cos(zflag[1][0])
    u[1] = zflag[1][1]

    return x, u

# Function to compute the RHS of the system dynamics
def vehicle_update(t, x, u, params):
    # Get the parameter values
    m = params.get('m', 5.4)
    g = params.get('g', 9.81)

    dx = np.array([
        x[1],
        g - 1/m*np.cos(x[2])*u[0],
        u[1]
    ])
    return dx

# output function
def vehicle_output(t, x, u, params):
    # Get the parameter values
    m = params.get('m', 5.4)
    g = params.get('g', 9.81)
    return np.array([x[0],x[1],x[2],u[0],u[1]])

# Create differentially flat input/output system
# output law is optional
# if you dont give it an output the output is all states.
vehicle_flat = fs.FlatSystem(
    vehicle_flat_forward, vehicle_flat_reverse, vehicle_update, vehicle_output,
    inputs=('T', 'v'), outputs=('z', 'zdot', 'phi','T','v'),
    states=('z', 'zdot', 'phi'))

#Parameters
m = 5.4
g = 9.81
Tmax = 2*m*g
vmax  = np.pi/4

phi_c = np.arccos(m*g/Tmax)
V_safe = 0.5

# Time horizon
T = 4.0 # seconds
M = 8   # number of basis functions
N = 10  # number of timepoints

# initial state and input
x0 = [0.0,0.0,0.0]
u0 = [0.0,0.0]

# reference points
x_ref = np.array([0.0,V_safe,np.deg2rad(50)])
u_ref = np.array([0.0,0.0])

# Basis set
basis = fs.BezierFamily(M,T)
# basis = fs.PolyFamily(M)

# Define timepoints for evaluation plus basis function to use
timepts = np.linspace(0, T, N)

# # desired trajectory
# zflag = [np.zeros(3), np.zeros(3)]
# xdes = np.zeros((3,np.size(timepts)))
# udes = np.zeros((2,np.size(timepts)))
# for i in range(np.size(timepts)):
#     t = timepts[i]
#     zflag[0][0],zflag[0][1],zflag[0][2] = t/4.0*1.0,1.0/4.0,0.0
#     zflag[1][0],zflag[1][1],zflag[1][2] = 0.0,0.0,0.0
#     x,u = vehicle_flat_reverse(zflag)
#     xdes[:,i] = x
#     udes[:,i] = u

# # Define the trajectory cost
# traj_cost = opt.quadratic_cost(
#     vehicle_flat, np.diag([1, 1, 1e5]), 1*np.diag([1, 1e-5]), x0=x_ref, u0=u_ref) 

# # Define the terminal cost
# term_cost = opt.quadratic_cost(
#     vehicle_flat, np.diag([1, 1, 1]), np.diag([1, 1]), x0=x_ref, u0=u_ref) 

lb = [-2.0,   -2.0,   0.0,       0.0,   -vmax]
ub = [ 0.0,   +2.0,   np.pi/2,   Tmax,  +vmax]
constraints = [opt.output_range_constraint(vehicle_flat, lb, ub)]

# # Solve for an optimal solution
# start_time = time.process_time()
# traj = fs.solve_flat_ocp(
#     vehicle_flat, timepts, x0, u0, 
#     # trajectory_cost=traj_cost, 
#     terminal_cost=term_cost,
#     constraints=constraints,
#     basis=basis, 
#     minimize_method='SLSQP',
# )
# print("* Total time = %5g seconds\n" % (time.process_time() - start_time))

start_time = time.process_time()
traj = fs.point_to_point(
    vehicle_flat, 
    timepts, 
    x0, 
    u0, 
    x_ref, 
    u_ref, 
    # cost=traj_cost, 
    basis=basis, 
    trajectory_constraints=constraints, 
    params=None)
print("* Total time = %5g seconds\n" % (time.process_time() - start_time))

timepts_eval = np.linspace(0,T,100)
xd, ud = traj.eval(timepts_eval)
plot_results(timepts_eval, xd, ud, phi_c, V_safe, lb=lb, ub=ub)
# plot_results(timepts_eval, xd, ud, phi_c, V_safe)

plt.show()
