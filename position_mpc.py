# Python imports
import time
import numpy as np
from numpy import sin, cos,pi
import control as ct
import casadi as ca
import matplotlib.pyplot as plt
from IPython import embed

from m4_position_dynamics import m4_update, m4_output
from m4_position_dynamics import animate, params_
from OptRes import OptRes

class positionMPC():
    def __init__(self, N):

        # plotting fontsize 
        self.fontsize = 10

        # Number of control intervals
        self.N = N

        # Setup problem
        self.opti = ca.Opti()

        # Parameters
        self.m = 5.0
        self.g = 9.81

        self.Tmax = 2*self.m*self.g
        self.Fmax = self.Tmax/2
        self.max_tilt = np.pi/4

        # Create parameter which is current tilt angle
        self.varphi = self.opti.parameter(1,1)

        # current yaw
        self.psi = self.opti.parameter(1,1)

        # Create parameter which is reference position
        self.Pref = self.opti.parameter(3,1)

        # Create parameter which is the initial state
        self.X0 = self.opti.parameter(6,1)

        # Decision variables
        self.X = self.opti.variable(6, self.N)  # state trajectory (excluding self.X0)
        self.x = self.X[0, :]
        self.y = self.X[1, :]
        self.z = self.X[2, :]
        self.dx = self.X[3,:]
        self.dy = self.X[4,:]
        self.dz = self.X[5,:]

        self.U = self.opti.variable(4, self.N)  # control trajectory (self.u1, self.u2) (no final control)
        self.phi = self.U[0, :]
        self.th = self.U[1, :]
        self.T =  self.U[2, :]
        self.F =  self.U[3, :]

        self.tf = self.opti.parameter()          # final time

        self.p = ca.vertcat(self.x,self.y,self.z)

        # Set the objective
        cost_position = ca.sum1(ca.sum2((self.p-self.Pref)*(self.p-self.Pref)))
        cost_inputs   = ca.sum1(ca.sum2(self.U*self.U))
        self.opti.minimize(cost_position + 1e-5*cost_inputs)

        # Create gap closing constraints using RK4
        dt = self.tf / self.N  # length of a control interval
        x_next = self.rk4(self.X0,self.U[:,0],dt)
        self.opti.subject_to(self.X[:, 1] == x_next)
        for k in range(1,self.N):
            x_next = self.rk4(self.X[:, k-1],self.U[:, k],dt)
            self.opti.subject_to(self.X[:, k] == x_next)

        # path constraints
        self.opti.subject_to(self.T <= self.Tmax)
        self.opti.subject_to(0<=self.T)
        self.opti.subject_to(self.F <= self.Fmax)
        self.opti.subject_to(-self.Fmax<=self.F)

        # self.opti.subject_to(-np.pi/2 <= self.psi)
        # self.opti.subject_to(self.psi <= np.pi/2)
        self.opti.subject_to(self.phi**2 + self.th**2 < self.max_tilt**2)

        # Solve NLP using IPOPT
        p_opts = {'expand': True}
        p_opts = {'expand': True,'ipopt.print_level':0, 'print_time':0}
        s_opts = {'max_iter': 1e6}

        self.opti.solver('ipopt', p_opts, s_opts)

    def solve(self):
        self.sol = self.opti.solve()
        return self.sol

    def plot_results(self):
        # Define the time array
        t = np.linspace(0, self.sol.value(self.T), self.N + 1)

        # Create a 2x2 grid of subplots
        plt.figure()

        # Plot z(t) trajectory
        plt.subplot(2, 2, 1)
        plt.plot(t, self.sol.value(self.z), "-k", linewidth=2)
        plt.title(r"$z(t)$ trajectory", fontsize=self.fontsize)
        plt.grid(True)

        # Plot phi(t) trajectory
        plt.subplot(2, 2, 2)
        plt.plot(t, np.rad2deg(self.sol.value(self.phi)), "-k", linewidth=2)
        plt.title(r"$\phi(t)$ trajectory", fontsize=self.fontsize)
        plt.grid(True)

        # Plot zdot(t) trajectory
        plt.subplot(2, 2, 3)
        plt.plot(t, self.sol.value(self.zdot), "-k", linewidth=2)
        plt.title(r"$\dot z(t)$ trajectory", fontsize=self.fontsize)
        plt.grid(True)

        # Plot T(t) trajectory
        plt.subplot(2, 2, 4)
        plt.plot(t[:-1], self.sol.value(self.u1), "-k", linewidth=2)
        plt.title(r"$T(t)$ trajectory", fontsize=self.fontsize)
        plt.grid(True)

        plt.show()

    def eulzyx2rot(self, phi,th,psi):
        R = ca.MX.zeros(3,3)
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

    def f(self,in1, in2):
        m = 5.0
        g = 9.81

        x = in1[0]
        y = in1[1]
        z = in1[2]
        dx = in1[3]
        dy = in1[4]
        dz = in1[5]

        phi = in2[0]
        th = in2[1]
        T = in2[2]
        F = in2[3]
        
        R = self.eulzyx2rot(phi,th,self.psi)

        fB = ca.MX.zeros(3,1)
        fB[1] = F*sin(self.varphi)
        fB[2] = -T*cos(self.varphi)

        pdd = 1/m * R@fB + ca.vertcat(0,0,g)

        f = ca.vertcat(dx,dy,dz,pdd[0],pdd[1],pdd[2])
        return f

    def rk4(self,x,u,dt):
        k1 = self.f(x,u)
        k2 = self.f(x + dt/2 * k1, u)
        k3 = self.f(x + dt/2 * k2, u)
        k4 = self.f(x + dt * k3, u)
        x_next = x + dt/6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return x_next

class positionOCP():
    def __init__(self,timepts,mpc,t_horizon):
        self.mpc = mpc
        self.timepts = timepts
        self.t_horizon = t_horizon
        self.Xsol = None
        self.Usol = None

    def compute_trajectory(self,t,x,varphi,psi,xref,print_summary=False):
        self.mpc.opti.set_value(self.mpc.tf,self.t_horizon)
        self.mpc.opti.set_value(self.mpc.X0,ca.vertcat(x[0],x[1],x[2],x[3],x[4],x[5]))
        self.mpc.opti.set_value(self.mpc.Pref,ca.vertcat(xref[0],xref[1],xref[2]))
        self.mpc.opti.set_value(self.mpc.varphi,varphi)
        self.mpc.opti.set_value(self.mpc.psi,psi)

        # warm start
        if self.Xsol is not None: self.mpc.opti.set_initial(self.mpc.X, self.Xsol)
        if self.Usol is not None: self.mpc.opti.set_initial(self.mpc.U, self.Usol)
        sol = self.mpc.solve()
        self.Xsol =  sol.value(self.mpc.X)
        self.Usol =  sol.value(self.mpc.U)
        states = self.Xsol
        inputs = self.Usol
        time = np.linspace(0, self.t_horizon, self.mpc.N + 1)
        res = OptRes(time,inputs,states,None,None)

        return res

def run_rhc_and_plot(proc, ocp, X0, varphi, psi, xref, print_summary=False, verbose=False, ax=None, plot=True):   
    # Start at the initial point
    x = X0
    
    # Initialize the axes
    if plot and ax is None:
        f,ax = plt.subplots()
        f1,ax1 = plt.subplots()
        f2,ax2 = plt.subplots()

    # Initialize arrays to store the final trajectory
    time_, inputs_, outputs_, states_  = [], [], [], []
    
    # Generate the individual traces for the receding horizon control
    for t in ocp.timepts:
        # Compute the optimal trajectory over the horizon
        start_time = time.process_time()
        res = ocp.compute_trajectory(t, x,varphi, psi, xref,print_summary=print_summary)
        if verbose: print(f"{t=}: comp time = {time.process_time() - start_time:0.3}")

        # Simulate the system for the update time, with higher res for plotting
        tvec = np.linspace(0, res.time[1], 20)
        inputs_temp = np.zeros((5,1))
        inputs_temp[0] = res.inputs[0,0]
        inputs_temp[1] = res.inputs[1,0]
        inputs_temp[2] = psi
        inputs_temp[3] = res.inputs[2,0]
        inputs_temp[4] = res.inputs[3,0]

        inputs = np.outer(inputs_temp,np.ones_like(tvec))
        soln = ct.input_output_response(proc, tvec, inputs, x)
        
        # Save this segment for later use (final point will appear in next segment)
        time_.append(t + soln.time[:-1])
        inputs_.append(soln.inputs[:, :-1])
        outputs_.append(soln.outputs[:, :-1])
        states_.append(soln.states[:, :-1])

        if plot:
            # Plot the results over the full horizon
            h3, = ax.plot(t + res.time[:-1], res.states[0], 'k--', linewidth=0.5)

            # Plot the results for this time segment
            h1, = ax.plot(t + soln.time, soln.states[0], 'r-')
            h1, = ax.plot(t + soln.time, soln.states[1], 'g-')
            h1, = ax.plot(t + soln.time, soln.states[2], 'b-')

            # plot inputs
            h4, = ax2.plot(t + soln.time, soln.inputs[0], 'r--')
            h4, = ax2.plot(t + soln.time, soln.inputs[1], 'g--')
            # h4, = ax2.plot(t + soln.time, soln.inputs[2], 'b--')

            h2, = ax1.plot(t + soln.time, soln.inputs[2], 'r-')
            h2, = ax1.plot(t + soln.time, soln.inputs[3], 'g-')


        # Update the state to use for the next time point
        x = soln.states[:, -1]
        
    # Append the final point to the response
    time_.append(t + soln.time[-1:])
    inputs_.append(soln.inputs[:, -1:])
    outputs_.append(soln.outputs[:, -1:])
    states_.append(soln.states[:, -1:])

    # Append
    return ct.TimeResponseData(
        np.hstack(time_), np.hstack(outputs_), np.hstack(states_), np.hstack(inputs_))

if __name__ == "__main__":
    # stabilizing MPC
    tf = 4.0
    dt = 0.02
    timepts = np.arange(0, tf+dt, dt)
    t_horizon = 4.0

    N = 10

    # define the optimal control problem 
    ocp = positionOCP(timepts,positionMPC(N), t_horizon)

    varphi = np.deg2rad(0)

    params = {
            'm': 5.0,               # total mass
            'g' : 9.81,             # gravitational acceleration
            'varphi': varphi             # default tilt angle
        }

    # m4 system
    m4 = ct.NonlinearIOSystem(
        name='m4',
        updfcn=m4_update, outfcn=m4_output,
        states = ['x','y','z','dx','dy','dz'],
        inputs = ['phi','th','psi','T','F'],
        outputs = ['x','y','z','dx','dy','dz'],
        params = params
    )

    # initial state
    x0 = np.array([0,0,0,0,0,0])

    # reference state
    xref = np.array([1,0.7,-1.3])

    psi = 0.0

    # run mpc
    rhc_resp = run_rhc_and_plot(m4, ocp, x0, varphi, psi, xref, verbose=True, print_summary=False)
    print(f"xf = {rhc_resp.states[:, -1]}")
    plt.show()

    # input output response
    # t,states,inputs,outputs= rhc_resp.time, rhc_resp.states, rhc_resp.inputs, rhc_resp.outputs
    # animate(outputs,np.diff(t)[0],params_,make_video=False)
