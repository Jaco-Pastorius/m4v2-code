import numpy as np
from numpy import cos, sin
import control as ct
import control.flatsys as fs
import control.optimal as opt 
import time
import matplotlib.pyplot as plt
from IPython import embed

# inertia tensor
J =  np.zeros((3,3))
J[0,0],J[1,1],J[2,2] = 0.002, 0.008, 0.007
Jinv =  np.linalg.inv(J)

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

def getE_ZYX(phi,th,psi):
    cz,sz = cos(psi),sin(psi)
    cy,sy = cos(th) ,sin(th)
    out = np.zeros((3,3))
    out[0,0] = 0
    out[0,1] = -sz
    out[0,2] = cy*cz
    out[1,0] = 0
    out[1,1] = cz
    out[1,2] = cy*cz
    out[2,0] = 1
    out[2,1] = 0
    out[2,2] = -sy
    return out

def getE_ZYX_dot(phi,th,psi,dphi,dth,dpsi):
    cz,sz = cos(psi),sin(psi)
    cy,sy = cos(th) ,sin(th)
    out = np.zeros((3,3))
    out[0,0] = 0
    out[0,1] = -cz * dpsi
    out[0,2] = -sy*cz*dth -cy*sz*dpsi
    out[1,0] = 0
    out[1,1] = -sz*dpsi
    out[1,2] = -sy*cz*dth -cy*sz*dpsi
    out[2,0] = 0
    out[2,1] = 0
    out[2,2] = -cy*dth
    return out
    
def getEinvZYX(phi,th,psi):
    cz,sz = cos(psi),sin(psi)
    cy,sy = cos(th) ,sin(th)
    out = np.zeros((3,3))
    out[0,0] = cz*sy/cy
    out[0,1] = sy*sz/cy
    out[0,2] = 1
    out[1,0] = -sz
    out[1,1] = cz
    out[1,2] = 0
    out[2,0] = cz/cy
    out[2,1] = sz/cy
    out[2,2] = 0
    return out
        
def skew(v):
    out = np.zeros((3,3))
    out[0,0] = 0
    out[0,1] = -v[2]
    out[0,2] = v[1]
    out[1,0] = v[2]
    out[1,1] = 0
    out[1,2] = -v[0]
    out[2,0] = -v[1]
    out[2,1] = v[0]
    out[2,2] = 0
    return out
    
# Plot the trajectory in z,phi coordinates
def plot_results(t, x, u):
    # Set the size of the figure
    plt.figure(figsize=(10, 6))

    # Time traces of the state and input
    plt.subplot(1, 5, 1)
    plt.plot(t, x[0])
    plt.xlabel('Time t [sec]')
    plt.ylabel('z [m]')

    plt.subplot(1, 5, 2)
    plt.plot(t, x[1])
    plt.xlabel('Time t [sec]')
    plt.ylabel('zdot [m/s]')

    plt.subplot(1, 5, 3)
    plt.plot(t, np.rad2deg(x[2]))
    plt.xlabel('Time t [sec]')
    plt.ylabel('phi [deg]')

    plt.subplot(1, 5, 4)
    plt.plot(t, np.rad2deg(x[3]))
    plt.xlabel('Time t [sec]')
    plt.ylabel('th [deg]')

    plt.subplot(1, 5, 5)
    plt.plot(t, np.rad2deg(x[4]))
    plt.xlabel('Time t [sec]')
    plt.ylabel('psi [deg]')

    # Set the size of the figure
    plt.figure(figsize=(10, 6))

    plt.subplot(1, 4, 1)
    plt.plot(t, u[0])
    plt.xlabel('Time t [sec]')
    plt.ylabel('taux]')

    plt.subplot(1, 4, 2)
    plt.plot(t, u[1])
    plt.xlabel('Time t [sec]')
    plt.ylabel('tauy')

    plt.subplot(1, 4, 3)
    plt.plot(t, u[2])
    plt.xlabel('Time t [sec]')
    plt.ylabel('tauz')

    plt.subplot(1, 4, 4)
    plt.plot(t, u[3])
    plt.xlabel('Time t [sec]')
    plt.ylabel('T')
    plt.tight_layout()

# Function to take states, inputs and return the flat flag
def vehicle_flat_forward(x, u, params={}):
    
    # get the parameter values
    m = params.get('m', 5.0)
    g = params.get('g', 9.81)
    varphi = params.get('varphi',30.0)
    w = params.get('w',0.0775)
    d = params.get('d',0.1325)
    l = params.get('l',0.16)

    # get wrench from forces
    # utemp = np.array([[1,1,1,1],[-1,1,-1,1],[-1,-1,1,1],[1,-1,-1,1]]) @ u
    # wrench = np.array([[0,w+d*cos(varphi),0,0],[0,0,l*cos(varphi),0],[0,0,0,l*sin(varphi)],[1,0,0,0]]) @ utemp

    wrench = u

    # get state
    z,dz,phi,th,psi,ox,oy,oz = x

    # get necessary quantities
    tau = np.array([wrench[0],wrench[1],wrench[2]])
    o   = np.array([ox,oy,oz])
    oh  = skew(o)
    R = eulzyx2rot(phi,th,psi)
    F = -tau[0]/(w+d*cos(varphi))
    fB = np.array([0,F*sin(varphi),-wrench[3]*cos(varphi)])
    Einv = getEinvZYX(phi,th,psi)

    # Create a list of arrays to store the flat output and its derivatives
    zflag = [np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)]

    # compute dchi
    chid = Einv @ R @ o
    dpsi,dth,dphi = chid
    Edot = getE_ZYX_dot(phi,th,psi,dphi,dth,dpsi)

    # compute ddz
    pdd = 1/m * R@fB + np.array([0,0,g])
    ddz = pdd[2]

    # compute od 
    od = Jinv @ (tau - np.cross(o,J@o))
    
    # compute chidd
    chidd = Einv @(R@oh@o + R@od - Edot@chid)
    ddpsi,ddth,ddphi = chidd
    
    # Flat outputs are (z,phi,th,psi)
    zflag[0][0] = z   
    zflag[1][0] = phi   
    zflag[2][0] = th   
    zflag[3][0] = psi   

    # First derivatives of the flat output
    zflag[0][1] = dz    # dz
    zflag[1][1] = dphi   # dphi
    zflag[2][1] = dth   # dth
    zflag[3][1] = dpsi  # dpsi

    # Second derivatives of the flat output
    zflag[0][2] = ddz     # ddz
    zflag[1][2] = ddphi   # ddphi
    zflag[2][2] = ddth   # ddth
    zflag[3][2] = ddpsi   # ddpsi

    return zflag

# Function to take the flat flag and return states, inputs
def vehicle_flat_reverse(zflag, params={}):
    # Get the parameter values
    m = params.get('m', 5.0)
    g = params.get('g', 9.81)
    varphi = params.get('varphi',30.0)
    w = params.get('w',0.0775)
    d = params.get('d',0.1325)
    l = params.get('l',0.16)

    # Create a vector to store the state and inputs
    x = np.zeros(8)
    u = np.zeros(4)

    z,dz,ddz       = zflag[0][0],zflag[0][1],zflag[0][2]
    phi,dphi,ddphi = zflag[1][0],zflag[1][1],zflag[1][2]
    th,dth,ddth    = zflag[2][0],zflag[2][1],zflag[2][2]
    psi,dpsi,ddpsi = zflag[3][0],zflag[3][1],zflag[3][2]

    chid = np.array([dpsi,dth,dphi])
    chidd = np.array([ddpsi,ddth,ddphi])
    R =  eulzyx2rot(phi,th,psi)
    Rt = R.transpose()
    E = getE_ZYX(phi,th,psi)
    Edot = getE_ZYX_dot(phi,th,psi,dphi,dth,dpsi)

    o = Rt @ E @ chid
    oh = skew(o)
    ox,oy,oz = o

    od = Rt @ (Edot@chid + E@chidd) - oh@o
    tau = J@od + np.cross(o,J@o)

    F = -tau[0]/(w+d*cos(varphi))
    T = (F*sin(varphi)*R[1,2]+m*g-m*ddz)/(cos(varphi)*R[2,2])

    wrench = np.array([tau[0],tau[1],tau[2],T])

    # Given the flat variables, solve for the state
    x[0] = z
    x[1] = dz
    x[2] = phi
    x[3] = th
    x[4] = psi
    x[5] = ox
    x[6] = oy
    x[7] = oz
    
    # And next solve for the inputs
    u[0] = wrench[0]
    u[1] = wrench[1]
    u[2] = wrench[2]
    u[3] = wrench[3]

    return x, u

# Function to compute the RHS of the system dynamics
def vehicle_update(t, x_, u, params):
    # Get the parameter values
    m = params.get('m', 5.0)
    g = params.get('g', 9.81)
    varphi = params.get('varphi',30.0)
    w = params.get('w',0.0775)
    d = params.get('d',0.1325)
    l = params.get('l',0.16)

    # get states
    z,dz,phi,th,psi,ox,oy,oz   = x_

    # get inputs
    taux,tauy,tauz,T = u

    F = -taux/(w+d*cos(varphi))
    o = np.array([ox,oy,oz])
    tau = np.array([taux,tauy,tauz])
    R   = eulzyx2rot(phi,th,psi)

    # dynamics
    pdd = 1/m * R @ np.array([0,F*sin(varphi),-T*cos(varphi)]) + np.array([0,0,g])
    chid = getEinvZYX(phi,th,psi) @ R @ o
    od = Jinv@(tau-np.cross(o,J@o))

    return np.array([dz,pdd[2],chid[2],chid[1],chid[0],od[0],od[1],od[2]])

# output function
def vehicle_output(t, x_, u, params):
    return x_

# Create differentially flat input/output system
vehicle_flat = fs.FlatSystem(
    vehicle_flat_forward, vehicle_flat_reverse, vehicle_update, vehicle_output,
    inputs=('taux','tauy','tauz','T'), outputs=('z','dz','phi','th','psi','ox','oy','oz'),
    states=('z','dz','phi','th','psi','ox','oy','oz'))

#Parameters
m = 5.0
g = 9.81
Tmax = 2*m*g

# Time horizon
T = 4.0 # seconds
M = 10   # number of basis functions
N = 10  # number of timepoints

# initial state and input
x0 = [0,0,0,0,0,0,0,0]
u0 = [0,0,0,m*g]

# reference points
x_ref = np.array([-1,0,0,0,0,0,0,0])
u_ref = np.array([0,0,0,m*g])

# Basis set
basis = fs.BezierFamily(M,T)
# basis = fs.PolyFamily(M)

# Define timepoints for evaluation plus basis function to use
timepts = np.linspace(0, T, N)

# Define the trajectory cost
traj_cost = opt.quadratic_cost(
    vehicle_flat, np.diag([10,1,1,1,0.1,1,1,1]), 1e-5*np.diag([10,10,10,1]), x0=x_ref, u0=u_ref) 

# # Define the terminal cost
# term_cost = opt.quadratic_cost(
#     vehicle_flat, np.diag([1, 1, 100000]), 1e-5*np.diag([1, 1]), x0=x_ref, u0=u_ref) 

# lb = [0.0,-1.0,0.0,0.0, -vmax]
# ub = [x0[0]+0.5,0.5,np.pi/2,Tmax, vmax]
# constraints = [opt.output_range_constraint(vehicle_flat, lb, ub)]

# start_time = time.process_time()
# traj = fs.point_to_point(
#     vehicle_flat, 
#     timepts, 
#     x0, 
#     u0, 
#     x_ref, 
#     u_ref, 
#     cost=traj_cost, 
#     basis=basis, 
#     trajectory_constraints=None, 
#     params=None)
# print("* Total time = %5g seconds\n" % (time.process_time() - start_time))


# Solve for an optimal solution
start_time = time.process_time()
traj = fs.solve_flat_ocp(
    vehicle_flat, timepts, x0, u0, 
    trajectory_cost=traj_cost, 
    # terminal_cost=term_cost,
    # constraints=constraints,
    basis=basis, 
    minimize_method='SLSQP',
)
print("* Total time = %5g seconds\n" % (time.process_time() - start_time))


timepts_eval = np.linspace(0,T,100)
xd, ud = traj.eval(timepts_eval)
plot_results(timepts_eval, xd, ud)
plt.show()
