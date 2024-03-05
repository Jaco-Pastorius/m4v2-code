# Python imports
import time
import numpy as np
import control as ct
import casadi as ca
from casadi import cos, sin
import matplotlib.pyplot as plt
from IPython import embed

from m4_torque_dynamics import m4_update, m4_output
from m4_torque_dynamics import animate, params_
from OptRes import OptRes

class torqueMPC():
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
        self.J = ca.MX.zeros(3,3)
        self.J[0,0] = 0.002
        self.J[1,1] = 0.008
        self.J[2,2] = 0.007
        self.Jinv =  ca.MX.zeros(3,3)
        self.Jinv[0,0] = 1.0/0.002
        self.Jinv[1,1] = 1.0/0.008
        self.Jinv[2,2] = 1.0/0.007
        self.w = 0.0775
        self.d = 0.1325
        self.l = 0.16

        # Constraint bounds 
        self.Tmax = 2*self.m*self.g
        self.f_max = self.Tmax/4
        self.max_tilt = np.pi/4

        # Final time parameter
        self.tf = self.opti.parameter() 

        # Create parameter which is current tilt angle
        self.varphi = self.opti.parameter(1,1)

        self.psi = self.opti.parameter(1,1)


        # Create parameter which is reference position
        self.X_ref = self.opti.parameter(7,1)

        # Create parameter which is the initial state
        self.X0 = self.opti.parameter(7,1)

        # Decision variables
        self.X = self.opti.variable(7, self.N)  # state trajectory (excluding self.X0)
        self.z = self.X[0, :]
        self.dz = self.X[1,:]
        self.phi = self.X[2, :]
        self.th = self.X[3, :]
        self.ox = self.X[4,:]
        self.oy = self.X[5,:]
        self.oz = self.X[6, :]

        self.U = self.opti.variable(4, self.N)  # control trajectory (self.u1, self.u2) (no final control)
        self.f1 = self.U[0, :]
        self.f2 = self.U[1, :]
        self.f3 =  self.U[2, :]
        self.f4 =  self.U[3, :]

        # Set the objective
        Q = ca.diag([1,1,1,1,1,1,1])
        cost_position = ca.sum1(ca.sum2((self.X-self.X_ref).T@Q@(self.X-self.X_ref)))
        cost_inputs   = ca.sum1(ca.sum2(self.U*self.U))
        alpha         = 1e-6
        self.opti.minimize(cost_position + alpha*cost_inputs)

        # Create gap closing constraints using RK4
        dt = self.tf / self.N  # length of a control interval
        x_next = self.rk4(self.X0,self.U[:,0],dt)
        self.opti.subject_to(self.X[:, 1] == x_next)
        for k in range(1,self.N):
            x_next = self.rk4(self.X[:, k-1],self.U[:, k],dt)
            self.opti.subject_to(self.X[:, k] == x_next)

        # path constraints
        self.opti.subject_to(self.f1 <= self.f_max)
        self.opti.subject_to(0<=self.f1)
        self.opti.subject_to(self.f2 <= self.f_max)
        self.opti.subject_to(0<=self.f2)     
        self.opti.subject_to(self.f3 <= self.f_max)
        self.opti.subject_to(0<=self.f3)
        self.opti.subject_to(self.f4 <= self.f_max)
        self.opti.subject_to(0<=self.f4)  

        # self.opti.subject_to(self.phi**2 + self.th**2 < self.max_tilt**2)

        # Solve NLP using IPOPT
        p_opts = {'expand': True}
        p_opts = {'expand': True,'ipopt.print_level':0, 'print_time':0}
        s_opts = {'max_iter': 1e6}
        self.opti.solver('ipopt', p_opts, s_opts)

        # # Solve NLP using QRQP
        # opts = {
        #         # 'qpsol':'osqp',
        #         'qpsol':'qrqp',
        #         'print_iteration': False, 
        #         'print_header': False, 
        #         'print_status': False, 
        #         'qpsol_options.print_iter': False, 
        #         'qpsol_options.print_header': False, 
        #         'qpsol_options.error_on_fail': False, 
        #         'expand': True}
        
        # self.opti.solver('sqpmethod',opts)

    def compile(self):
        MPC=self.opti.to_function('MPC',[self.X0,self.X_ref,self.varphi],[self.U,self.X])
        MPC.generate('MPC.c')

    def solve(self):
        self.sol = self.opti.solve()
        return self.sol

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

    def getE_ZYX(self,phi,th,psi):
        cz,sz = cos(psi),sin(psi)
        cy,sy = cos(th) ,sin(th)
        out = ca.MX.zeros(3,3)
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
    
    def getE_ZYX_dot(self,phi,th,psi,dphi,dth,dpsi):
        cz,sz = cos(psi),sin(psi)
        cy,sy = cos(th) ,sin(th)
        out = ca.MX.zeros(3,3)
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
        
    def getEinvZYX(self,phi,th,psi):
        cz,sz = cos(psi),sin(psi)
        cy,sy = cos(th) ,sin(th)
        out = ca.MX.zeros(3,3)
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
            
    def skew(self,v):
        out = ca.MX.zeros(3,3)
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
    
    def get_wrench_from_forces(self,forces):
        u = np.array([[1,1,1,1],[-1,1,-1,1],[-1,-1,1,1],[1,-1,-1,1]]) @ forces
        wrench = np.array([[0,self.w+self.d*cos(self.varphi),0,0],[0,0,self.l*cos(self.varphi),0],[0,0,0,self.l*sin(self.varphi)],[1,0,0,0]]) @ u
        return wrench
    
    def f(self,in1, in2):
        z = in1[0]
        dz = in1[1]
        phi = in1[2]
        th = in1[3]
        ox = in1[4]
        oy = in1[5]
        oz = in1[6]

        f1 = in2[0]
        f2 = in2[1]
        f3 = in2[2]
        f4 = in2[3]

        wrench = self.get_wrench_from_forces(np.array([f1,f2,f3,f4]))

        tau = ca.vertcat(wrench[0],wrench[1],wrench[2])
        o   = ca.vertcat(ox,oy,oz)
        R = self.eulzyx2rot(phi,th,self.psi)
        F = -tau[0]/(self.w+self.d*cos(self.varphi))
        
        fB = ca.MX.zeros(3,1)   
        fB[1] = F*sin(self.varphi)
        fB[2] = -wrench[3]*cos(self.varphi)

        # z is pointing down dynamics
        pdd = 1/self.m * R@fB + ca.vertcat(0,0,self.g)
        chid = self.getEinvZYX(phi,th,self.psi) @ R @ o
        od = self.Jinv@(tau - ca.cross(o,self.J@o))

        f = ca.vertcat(dz,pdd[2],chid[2],chid[1],od[0],od[1],od[2])
        return f
    
    def A(f1, f2, f3, f4, ox, oy, oz, phi, th, varphi):
        t2 = np.cos(phi)
        t3 = np.cos(th)
        t4 = np.cos(varphi)
        t5 = np.sin(phi)
        t6 = np.sin(th)
        t7 = np.sin(varphi)
        t12 = -f2
        t13 = -f4
        t16 = f1 + f2 + f3 + f4
        t8 = oy * t2
        t9 = oz * t2
        t10 = oy * t5
        t11 = oz * t5
        t14 = 1.0 / t3
        t20 = f1 + f3 + t12 + t13
        t15 = t14 ** 2
        t17 = -t11
        t18 = t9 + t10
        t19 = t8 + t17
        out1 = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (t3 * t4 * t5 * t16) / 5.0 + (t2 * t3 * t7 * t20) / 5.0, t6 * t14 * t19, -t18],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (t2 * t4 * t6 * t16) / 5.0 - (t5 * t6 * t7 * t20) / 5.0, t15 * t18],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, t6 * t15 * t18],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, oz * (5.0 / 8.0), oy * (-6.0 / 7.0), 0.0, 0.0, t5 * t6 * t14, t2, t5 * t14, oz / 2.0, 0.0, ox * (-6.0 / 7.0), 0.0, 0.0, t2 * t6 * t14, -t5, t2 * t14, oy / 2.0, ox * (5.0 / 8.0), 0.0]])

        return out1

    def B(phi, th, varphi):
        t2 = np.cos(th)
        t3 = np.cos(varphi)
        t4 = np.sin(varphi)
        t5 = phi + varphi
        t6 = np.cos(t5)
        t7 = -varphi
        t8 = t3 * 2.0e+1
        t9 = -t8
        t10 = phi + t7
        t11 = np.cos(t10)
        t12 = t3 * (2.65e+2 / 4.0)
        t13 = t4 * (1.6e+2 / 7.0)
        t14 = -t12
        t15 = -t13
        t16 = (t2 * t6) / 5.0
        t17 = -t16
        t18 = (t2 * t11) / 5.0
        t19 = -t18
        t20 = t12 + 1.55e+2 / 4.0
        t21 = t14 - 1.55e+2 / 4.0
        out1 = np.array([[0.0, t17, 0.0, 0.0],
                        [0.0, t21, t9, t13],
                        [0.0, t19, 0.0, 0.0],
                        [0.0, t20, t9, t15],
                        [0.0, t17, 0.0, 0.0],
                        [0.0, t21, t8, t15],
                        [0.0, t19, 0.0, 0.0],
                        [0.0, t20, t8, t13]])

        return out1

    def rk4(self,x,u,dt):
        k1 = self.f(x,u)
        k2 = self.f(x + dt/2 * k1, u)
        k3 = self.f(x + dt/2 * k2, u)
        k4 = self.f(x + dt * k3, u)
        x_next = x + dt/6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return x_next

class torqueOCP():
    def __init__(self,timepts,mpc,t_horizon):
        self.mpc = mpc
        self.t_horizon = t_horizon
        self.Xsol = None
        self.Usol = None
        self.timepts = timepts

    def compute_trajectory(self,x,varphi,psi,xref,print_summary=False):
        self.mpc.opti.set_value(self.mpc.tf,self.t_horizon)
        self.mpc.opti.set_value(self.mpc.X0,ca.vertcat(x[2],x[5],x[6],x[7],x[9],x[10],x[11]))
        self.mpc.opti.set_value(self.mpc.X_ref,ca.vertcat(xref[0],xref[1],xref[2],xref[3],xref[4],xref[5],xref[6]))
        self.mpc.opti.set_value(self.mpc.varphi,varphi)
        self.mpc.opti.set_value(self.mpc.psi,psi)

        # warm start
        # if self.Xsol is not None: self.mpc.opti.set_initial(self.mpc.X, self.Xsol)
        # if self.Usol is not None: self.mpc.opti.set_initial(self.mpc.U, self.Usol)
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
        f3,ax3 = plt.subplots()

    # Initialize arrays to store the final trajectory
    time_, inputs_, outputs_, states_  = [], [], [], []
    
    # Generate the individual traces for the receding horizon control
    for t in ocp.timepts:
        # Compute the optimal trajectory over the horizon
        start_time = time.process_time()
        res = ocp.compute_trajectory(x,varphi, psi, xref,print_summary=print_summary)
        if verbose: print(f"{t=}: comp time = {time.process_time() - start_time:0.3}")

        # Simulate the system for the update time, with higher res for plotting
        tvec = np.linspace(0, res.time[1], 20)
        # inputs_temp = np.zeros((4,1))
        # inputs_temp[0] = res.inputs[0,0]
        # inputs_temp[1] = res.inputs[1,0]
        # inputs_temp[2] = 0
        # inputs_temp[3] = res.inputs[2,0]

        u = np.array([[1,1,1,1],[-1,1,-1,1],[-1,-1,1,1],[1,-1,-1,1]]) @ res.inputs[:,0]
        wrench = np.array([[0, ocp.mpc.w+ ocp.mpc.d*cos(varphi),0,0],[0,0, ocp.mpc.l*cos(varphi),0],[0,0,0, ocp.mpc.l*sin(varphi)],[1,0,0,0]]) @ u

        inputs = np.outer(wrench,np.ones_like(tvec))
        soln = ct.input_output_response(proc, tvec, inputs, x)
        
        # Save this segment for later use (final point will appear in next segment)
        time_.append(t + soln.time[:-1])
        inputs_.append(soln.inputs[:, :-1])
        outputs_.append(soln.outputs[:, :-1])
        states_.append(soln.states[:, :-1])

        if plot:
            # Plot the results over the full horizon
            # h3, = ax.plot(t + res.time[:-1], res.states[0], 'k--', linewidth=0.5)

            # Plot the results for this time segment
            h1, = ax.plot(t + soln.time, soln.states[0], 'r-')
            h1, = ax.plot(t + soln.time, soln.states[1], 'g-')
            h1, = ax.plot(t + soln.time, soln.states[2], 'b-')

            # Plot the results for this time segment
            h3, = ax3.plot(t + soln.time, soln.states[6], 'r-')
            h3, = ax3.plot(t + soln.time, soln.states[7], 'g-')
            h3, = ax3.plot(t + soln.time, soln.states[8], 'b-')

            # plot inputs
            h4, = ax2.plot(t + soln.time, soln.inputs[3], 'r-')
            # h4, = ax2.plot(t + soln.time, soln.inputs[1], 'g--')
            # h4, = ax2.plot(t + soln.time, soln.inputs[2], 'b--')

            h2, = ax1.plot(t + soln.time, soln.inputs[0], 'r-')
            h2, = ax1.plot(t + soln.time, soln.inputs[1], 'g-')
            h2, = ax1.plot(t + soln.time, soln.inputs[2], 'b-')

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
    tf = 1.0
    dt = 0.001
    timepts = np.arange(0, tf+dt, dt)
    t_horizon = 0.7

    N = 2

    # define the optimal control problem 
    ocp = torqueOCP(timepts,torqueMPC(N), t_horizon)
    # ocp.mpc.compile()
    # embed()

    varphi = np.deg2rad(40)

    params = {
            'm': 5.0,               # total mass
            'g' : 9.81,             # gravitational acceleration
            'varphi': varphi        # default tilt angle
        }

    m4 = ct.NonlinearIOSystem(
        name='m4',
        updfcn=m4_update, outfcn=m4_output,
        states = ['x','y','z','dx','dy','dz','phi','th','psi','ox','oy','oz'],
        inputs = ['taux','tauy','tauz','T'],
        outputs = ['x','y','z','dx','dy','dz','phi','th','psi','ox','oy','oz'],
        params = params
    )
    # initial state
    x0 = np.array([0,0,0,0,0,0,0,0,0,0,0,0])

    # reference state
    xref = np.array([1.5,0.0,0.0,0.0,0,0,0,0,0])

    psi = 0.0

    # run mpc
    rhc_resp = run_rhc_and_plot(m4, ocp, x0, varphi, psi, xref, verbose=True, print_summary=False)
    print(f"xf = {rhc_resp.states[:, -1]}")
    plt.show()