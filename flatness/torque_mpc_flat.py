# Python imports
import time
import numpy as np
from numpy import sin, cos,pi
import control as ct
import casadi as ca
import matplotlib.pyplot as plt
from IPython import embed

from scipy.special import binom, factorial
import itertools
import scipy as sp

from m4_torque_dynamics import m4_update, m4_output
from m4_torque_dynamics import animate, params_
from OptRes import OptRes


varphi = 0.0

# Utility function to compute flag matrix given a basis
def _basis_flag_matrix(ninputs, basis, t):
    """Compute the matrix of basis functions and their derivatives

    This function computes the matrix ``M`` that is used to solve for the
    coefficients of the basis functions given the state and input.  Each
    column of the matrix corresponds to a basis function and each row is a
    derivative, with the derivatives (flag) for each output stacked on top
    of each other.

    """
    flagshape = [3,3,3,3]
    M = ca.MX.zeros(sum(flagshape),
                  sum([basis.var_ncoefs(i) for i in range(ninputs)]))
    flag_off = 0
    coef_off = 0
    for i, flag_len in enumerate(flagshape):
        coef_len = basis.var_ncoefs(i)
        for j, k in itertools.product(range(coef_len), range(flag_len)):
            M[flag_off + k, coef_off + j] = basis.eval_deriv(j, k, t, var=i)
        flag_off += flag_len
        coef_off += coef_len
    return M

class BasisFamily:
    """Base class for implementing basis functions for flat systems.

    A BasisFamily object is used to construct trajectories for a flat system.
    The class must implement a single function that computes the jth
    derivative of the ith basis function at a time t:

      :math:`z_i^{(q)}(t)` = basis.eval_deriv(self, i, j, t)

    A basis set can either consist of a single variable that is used for
    each flat output (nvars = None) or a different variable for different
    flat outputs (nvars > 0).

    Attributes
    ----------
    N : int
        Order of the basis set.

    """
    def __init__(self, N):
        """Create a basis family of order N."""
        self.N = N                    # save number of basis functions
        self.nvars = None             # default number of variables
        self.coef_offset = [0]        # coefficient offset for each variable
        self.coef_length = [N]        # coefficient length for each variable

    def __repr__(self):
        return f'<{self.__class__.__name__}: nvars={self.nvars}, ' + \
            f'N={self.N}>'

    def __call__(self, i, t, var=None):
        """Evaluate the ith basis function at a point in time"""
        return self.eval_deriv(i, 0, t, var=var)

    def var_ncoefs(self, var):
        """Get the number of coefficients for a variable"""
        return self.N if self.nvars is None else self.coef_length[var]

    def eval(self, coeffs, tlist, var=None):
        """Compute function values given the coefficients and time points."""
        if self.nvars is None and var != None:
            raise SystemError("multi-variable call to a scalar basis")

        elif self.nvars is None:
            # Single variable basis
            return [
                sum([coeffs[i] * self(i, t) for i in range(self.N)])
                for t in tlist]

        elif var is None:
            # Multi-variable basis with single list of coefficients
            values = np.empty((self.nvars, tlist.size))
            offset = 0
            for j in range(self.nvars):
                coef_len = self.var_ncoefs(j)
                values[j] = ca.MX(
                    sum([coeffs[offset + i] * self(i, t, var=j)
                         for i in range(coef_len)])
                    for t in tlist)
                offset += coef_len
            return values

        else:
            return ca.MX(
                sum([coeffs[i] * self(i, t, var=var)
                     for i in range(self.var_ncoefs(var))])
                for t in tlist)

    def eval_deriv(self, i, j, t, var=None):
        """Evaluate the kth derivative of the ith basis function at time t."""
        raise NotImplementedError("Internal error; improper basis functions")

class BezierFamily(BasisFamily):
    r"""Bezier curve basis functions.

    This class represents the family of polynomials of the form

    .. math::
         \phi_i(t) = \sum_{i=0}^N {N \choose i}
             \left( \frac{t}{T} - t \right)^{N-i}
             \left( \frac{t}{T} \right)^i

    Parameters
    ----------
    N : int
        Degree of the Bezier curve.

    T : float
        Final time (used for rescaling).

    """
    def __init__(self, N, T=1):
        """Create a polynomial basis of order N."""
        super(BezierFamily, self).__init__(N)
        self.T = T      # save end of time interval

    # Compute the kth derivative of the ith basis function at time t
    def eval_deriv(self, i, k, t, var=None):
        """Evaluate the kth derivative of the ith basis function at time t."""
        if i >= self.N:
            raise ValueError("Basis function index too high")
        elif k >= self.N:
            # Higher order derivatives are zero
            return 0 * t

        # Compute the variables used in Bezier curve formulas
        n = self.N - 1
        u = t/self.T

        if k == 0:
            # No derivative => avoid expansion for speed
            return binom(n, i) * u**i * (1-u)**(n-i)

        # Return the kth derivative of the ith Bezier basis function
        return binom(n, i) * sum([
            (-1)**(j-i) *
            binom(n-i, j-i) * factorial(j)/factorial(j-k) * \
            np.power(u, j-k) / np.power(self.T, k)
            for j in range(max(i, k), n+1)
        ])

class PolyFamily(BasisFamily):
    r"""Polynomial basis functions.

    This class represents the family of polynomials of the form

    .. math::
         \phi_i(t) = \left( \frac{t}{T} \right)^i

    Parameters
    ----------
    N : int
        Degree of the Bezier curve.

    T : float
        Final time (used for rescaling).

    """
    def __init__(self, N, T=1):
        """Create a polynomial basis of order N."""
        super(PolyFamily, self).__init__(N)
        self.T = float(T)       # save end of time interval

    # Compute the kth derivative of the ith basis function at time t
    def eval_deriv(self, i, k, t, var=None):
        """Evaluate the kth derivative of the ith basis function at time t."""
        if (i < k): return 0 * t        # higher derivative than power
        return factorial(i)/factorial(i-k) * \
            np.power(t/self.T, i-k) / np.power(self.T, k)


class FlatTorqueMPC():
    def __init__(self, N):

        # plotting fontsize 
        self.fontsize = 10

        # Number of basis functions
        self.N = N

        # Setup problem
        self.opti = ca.Opti()

        # Parameters
        self.m = 5.0
        self.g = 9.81
        self.J = np.zeros((3,3))
        self.J[0,0] = 0.002
        self.J[1,1] = 0.008
        self.J[2,2] = 0.007
        self.Jinv =  np.zeros((3,3))
        self.Jinv[0,0] = 1.0/0.002
        self.Jinv[1,1] = 1.0/0.008
        self.Jinv[2,2] = 1.0/0.007
        self.w = 0.0775
        self.d = 0.1325
        self.l = 0.16

        # Create parameter which is current tilt angle
        self.varphi = self.opti.parameter(1,1)

        # Create parameter which is reference position
        self.X_ref = self.opti.parameter(8,1)
        self.U_ref = self.opti.parameter(4,1)

        # Create parameter which is the initial state
        self.X0 = self.opti.parameter(8,1)
        self.U0 = self.opti.parameter(4,1)

        # Final time
        self.tf = self.opti.parameter()

        # basis function
        # self.basis = BezierFamily(self.N,self.tf)
        self.basis = PolyFamily(self.N)
        self.ninputs = 4

        # Decision variables
        self.null_coeffs = self.opti.variable(self.ninputs*self.N,1)

        start_time = time.process_time()
        # Get flag
        zflag_T0 = self.vehicle_flat_forward(self.X0,self.U0)
        # zflag_Tf = self.vehicle_flat_forward(self.X_ref,self.U_ref)
        # self.Z = ca.vertcat(zflag_T0[0],zflag_T0[1],zflag_T0[2],zflag_T0[3],zflag_Tf[0],zflag_Tf[1],zflag_Tf[2],zflag_Tf[3])
        self.Z = ca.vertcat(zflag_T0[0],zflag_T0[1],zflag_T0[2],zflag_T0[3])
        print(f"flag computation time = {time.process_time() - start_time:0.3}")

        # Get basis matrix
        start_time = time.process_time()
        M_T0 = _basis_flag_matrix(self.ninputs,self.basis,0.0)
        # M_Tf = _basis_flag_matrix(self.ninputs,self.basis,self.tf)
        # M = ca.vertcat(M_T0,M_Tf)
        M = M_T0
        print(f"basis matrix computation time = {time.process_time() - start_time:0.3}")

        start_time = time.process_time()
        self.Mpinv = ca.pinv(M)
        print(f"pinv computation time = {time.process_time() - start_time:0.3}")

        start_time = time.process_time()
        self.Nullspace = ca.nullspace(M)
        print(f"nullspace computation time = {time.process_time() - start_time:0.3}")
        
        # coeffs
        self.coeffs = self.Mpinv@self.Z + self.Nullspace @ self.null_coeffs

        # Set the objective (minimize null coeffs)
        self.e = M@self.coeffs - self.Z
        self.cost = self.e.T@self.e
        self.opti.minimize(self.cost)

        # Solve NLP using IPOPT
        p_opts = {'expand': True}
        p_opts = {'expand': True,'ipopt.print_level':0, 'print_time':0}
        s_opts = {'max_iter': 1e6}

        self.opti.solver('ipopt', p_opts, s_opts)

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

    def eulzyx2rot_np(self, phi,th,psi):
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

    def getE_ZYX_np(self,phi,th,psi):
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
    
    def getE_ZYX_dot_np(self,phi,th,psi,dphi,dth,dpsi):
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
        
    def getEinvZYX_np(self,phi,th,psi):
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
            
    def skew_np(self,v):
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
 
    def f(self,in1, in2):
        z = in1[0]
        dz = in1[1]
        phi = in1[2]
        th = in1[3]
        psi = in1[4]
        ox = in1[5]
        oy = in1[6]
        oz = in1[7]

        f1 = in2[0]
        f2 = in2[1]
        f3 = in2[2]
        f4 = in2[3]

        wrench = self.get_wrench_from_forces(np.array([f1,f2,f3,f4]))

        tau = ca.vertcat(wrench[0],wrench[1],wrench[2])
        o   = ca.vertcat(ox,oy,oz)
        R = self.eulzyx2rot(phi,th,psi)
        F = -tau[0]/(self.w+self.d*cos(self.varphi))
        
        fB = ca.MX.zeros(3,1)   
        fB[1] = F*sin(self.varphi)
        fB[2] = -wrench[3]*cos(self.varphi)

        # z is pointing down dynamics
        pdd = 1/self.m * R@fB + ca.vertcat(0,0,self.g)
        chid = self.getEinvZYX(phi,th,psi) @ R @ o
        od = self.Jinv@(tau - ca.cross(o,self.J@o))

        f = ca.vertcat(dz,pdd[2],chid[2],chid[1],chid[0],od[0],od[1],od[2])
        return f

    def vehicle_flat_forward_np(self,x,u):

        wrench = u

        # get state
        z = x[0]
        dz = x[1]
        phi = x[2]
        th = x[3]
        psi = x[4]
        ox = x[5]
        oy = x[6]
        oz = x[7]

        # get necessary quantities
        tau = np.array([wrench[0],wrench[1],wrench[2]])
        o   = np.array([ox,oy,oz])
        oh  = self.skew_np(o)
        R = self.eulzyx2rot_np(phi,th,psi)
        F = -tau[0]/(self.w+self.d*cos(varphi))
        fB = np.array([0,F*sin(varphi),-wrench[3]*cos(varphi)])
        Einv = self.getEinvZYX_np(phi,th,psi)

        # Create a list of arrays to store the flat output and its derivatives
        zflag = [np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)]

        # compute dchi
        chid = Einv @ R @ o
        dpsi = chid[0]
        dth  = chid[1]
        dphi = chid[2]
        Edot = self.getE_ZYX_dot_np(phi,th,psi,dphi,dth,dpsi)

        # compute ddz
        pdd = 1/self.m * R@fB + np.array([0,0,self.g])
        ddz = pdd[2]

        # compute od 
        od = self.Jinv @ (tau - np.cross(o,self.J@o))
        
        # compute chidd
        chidd = Einv @(R@oh@o + R@od - Edot@chid)
        ddpsi = chidd[0]
        ddth  = chidd[1]
        ddphi = chidd[2]
        
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

    def vehicle_flat_reverse_np(self, zflag):

        # Create a vector to store the state and inputs
        x = ca.MX.zeros(8,1)
        u = ca.MX.zeros(4,1)

        z,dz,ddz       = zflag[0][0],zflag[0][1],zflag[0][2]
        phi,dphi,ddphi = zflag[1][0],zflag[1][1],zflag[1][2]
        th,dth,ddth    = zflag[2][0],zflag[2][1],zflag[2][2]
        psi,dpsi,ddpsi = zflag[3][0],zflag[3][1],zflag[3][2]

        chid = ca.vertcat(dpsi,dth,dphi)
        chidd = ca.vertcat(ddpsi,ddth,ddphi)
        R =  self.eulzyx2rot(phi,th,psi)
        Rt = R.transpose()
        E = self.getE_ZYX(phi,th,psi)
        Edot = self.getE_ZYX_dot(phi,th,psi,dphi,dth,dpsi)

        o = Rt @ E @ chid
        oh = self.skew(o)
        ox,oy,oz = o

        od = Rt @ (Edot@chid + E@chidd) - oh@o
        tau = self.J@od + np.cross(o,self.J@o)

        F = -tau[0]/(self.w+self.d*cos(varphi))
        T = (F*sin(varphi)*R[1,2]+self.m*self.g-self.m*ddz)/(cos(varphi)*R[2,2])

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

    def vehicle_flat_forward(self,x,u):

        wrench = u

        # get state
        z = x[0]
        dz = x[1]
        phi = x[2]
        th = x[3]
        psi = x[4]
        ox = x[5]
        oy = x[6]
        oz = x[7]

        # get necessary quantities
        tau = ca.vertcat(wrench[0],wrench[1],wrench[2])
        o   = ca.vertcat(ox,oy,oz)
        oh  = self.skew(o)
        R = self.eulzyx2rot(phi,th,psi)
        F = -tau[0]/(self.w+self.d*cos(varphi))
        fB = ca.vertcat(0,F*sin(varphi),-wrench[3]*cos(varphi))
        Einv = self.getEinvZYX(phi,th,psi)

        # Create a list of arrays to store the flat output and its derivatives
        zflag = [ca.MX.zeros(3), ca.MX.zeros(3), ca.MX.zeros(3), ca.MX.zeros(3)]

        # compute dchi
        chid = Einv @ R @ o
        dpsi = chid[0]
        dth  = chid[1]
        dphi = chid[2]
        Edot = self.getE_ZYX_dot(phi,th,psi,dphi,dth,dpsi)

        # compute ddz
        pdd = 1/self.m * R@fB + ca.vertcat(0,0,self.g)
        ddz = pdd[2]

        # compute od 
        od = self.Jinv @ (tau - ca.cross(o,self.J@o))
        
        # compute chidd
        chidd = Einv @(R@oh@o + R@od - Edot@chid)
        ddpsi = chid[0]
        ddth  = chid[1]
        ddphi = chid[2]
        
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

    def vehicle_flat_reverse(self, zflag):

        # Create a vector to store the state and inputs
        x = ca.MX.zeros(8,1)
        u = ca.MX.zeros(4,1)

        z,dz,ddz       = zflag[0][0],zflag[0][1],zflag[0][2]
        phi,dphi,ddphi = zflag[1][0],zflag[1][1],zflag[1][2]
        th,dth,ddth    = zflag[2][0],zflag[2][1],zflag[2][2]
        psi,dpsi,ddpsi = zflag[3][0],zflag[3][1],zflag[3][2]

        chid = ca.vertcat(dpsi,dth,dphi)
        chidd = ca.vertcat(ddpsi,ddth,ddphi)
        R =  self.eulzyx2rot(phi,th,psi)
        Rt = R.transpose()
        E = self.getE_ZYX(phi,th,psi)
        Edot = self.getE_ZYX_dot(phi,th,psi,dphi,dth,dpsi)

        o = Rt @ E @ chid
        oh = self.skew(o)
        ox,oy,oz = o

        od = Rt @ (Edot@chid + E@chidd) - oh@o
        tau = self.J@od + np.cross(o,self.J@o)

        F = -tau[0]/(self.w+self.d*cos(varphi))
        T = (F*sin(varphi)*R[1,2]+self.m*self.g-self.m*ddz)/(cos(varphi)*R[2,2])

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

class FlatTorqueOCP():
    def __init__(self,timepts,mpc,t_horizon):
        self.mpc = mpc
        self.t_horizon = t_horizon
        self.Xsol = None
        self.Usol = None
        self.timepts = timepts

    def compute_trajectory(self,x,varphi,psi,xref,print_summary=False):
        self.mpc.opti.set_value(self.mpc.tf,self.t_horizon)
        self.mpc.opti.set_value(self.mpc.X0,ca.vertcat(x[2],x[5],x[6],x[7],x[8],x[9],x[10],x[11]))
        self.mpc.opti.set_value(self.mpc.X_ref,ca.vertcat(xref[0],xref[1],xref[2],xref[3],xref[4],xref[5],xref[6],xref[7]))
        self.mpc.opti.set_value(self.mpc.varphi,varphi)
        # self.mpc.opti.set_value(self.mpc.psi,psi)

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
    dt = 0.02
    timepts = np.arange(0, tf+dt, dt)
    t_horizon = 4.0

    N = 10

    # define the optimal control problem 
    ocp = FlatTorqueOCP(timepts,FlatTorqueMPC(N), t_horizon)

    varphi = np.deg2rad(0)

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
    ocp.mpc.opti.set_value(ocp.mpc.X0,ca.vertcat(0,0,0,0,0,0,0,0))
    ocp.mpc.opti.set_value(ocp.mpc.U0,ca.vertcat(0,0,0,5.0*9.81))

    ocp.mpc.opti.set_value(ocp.mpc.X_ref,ca.vertcat(-1,0,0,0,0,0,0,0))
    ocp.mpc.opti.set_value(ocp.mpc.U_ref,ca.vertcat(0,0,0,5.0*9.81))

    ocp.mpc.opti.set_value(ocp.mpc.varphi,0.0)
    ocp.mpc.opti.set_value(ocp.mpc.tf,t_horizon)

    start_time = time.process_time()
    sol = ocp.mpc.solve()
    print(f"solve time = {time.process_time() - start_time:0.3}")
    embed()

    # # reference state
    # xref = np.array([-0.5,0.0,0.0,0,0,0,0,0])
    # psi = 0.0

    # # run mpc
    # rhc_resp = run_rhc_and_plot(m4, ocp, x0, varphi, psi, xref, verbose=True, print_summary=False)
    # print(f"xf = {rhc_resp.states[:, -1]}")
    # plt.show()

    # # input output response
    # # t,states,inputs,outputs= rhc_resp.time, rhc_resp.states, rhc_resp.inputs, rhc_resp.outputs
    # # animate(outputs,np.diff(t)[0],params_,make_video=False)
