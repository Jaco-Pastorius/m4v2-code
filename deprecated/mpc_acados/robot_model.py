from acados_template import AcadosModel
from casadi import SX, MX, DM, vertcat, sin, cos, cross
import numpy as np 
from numpy import array
from IPython import embed

def eulzyx2rot(phi,th,psi):
    R = SX.zeros(3,3)
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
    out = SX.zeros(3,3)
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
    out = SX.zeros(3,3)
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
    out = SX.zeros(3,3)
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
    out = SX.zeros(3,3)
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


def export_robot_model() -> AcadosModel:
    model_name = "torque_dynamics"

    # parameters
    varphi = SX.sym("varphi",1)
    w = 0.0775
    d = 0.1325
    l = 0.16
    m = 5.16
    g = 9.81

    J = SX.zeros(3,3)
    J[0,0] = 0.13
    J[1,1] = 0.10
    J[2,2] = 0.21
    Jinv =  SX.zeros(3,3)
    Jinv[0,0] = 1.0/0.13
    Jinv[1,1] = 1.0/0.10
    Jinv[2,2] = 1.0/0.21

    # control allocation matrix
    M = np.array([
        [-6.09,  4.64,  1.45, 29.0],
        [ 6.09, -4.64,  1.45, 29.0],
        [ 6.09,  4.64, -1.45, 29.0],
        [-6.09, -4.64, -1.45, 29.0]
    ])
    M = M.transpose()    

    # states
    x = SX.sym("x",12)
    x_ = x[0]
    y_ = x[1]
    z = x[2]
    dx_ = x[3]
    dy = x[4]
    dz = x[5]
    phi = x[6]
    th = x[7]
    psi = x[8]
    ox = x[9]
    oy = x[10]
    oz = x[11]

    # controls
    u = SX.sym("u",4)

    # xdot
    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    z_dot = SX.sym("z_dot")
    dx_dot = SX.sym("dx_dot")
    dy_dot = SX.sym("dy_dot")
    dz_dot = SX.sym("dz_dot")
    phi_dot = SX.sym("phi_dot")
    th_dot = SX.sym("th_dot")
    psi_dot = SX.sym("psi_dot")
    ox_dot = SX.sym("ox_dot")
    oy_dot = SX.sym("oy_dot")
    oz_dot = SX.sym("oz_dot")

    xdot = vertcat(x_dot,y_dot,z_dot,dx_dot,dy_dot,dz_dot,phi_dot,th_dot,psi_dot,ox_dot,oy_dot,oz_dot)

    # algebraic variables
    # z = None

    # parameters
    p = varphi

    # dynamics
    wrench = M @ u

    tau = vertcat(wrench[0],wrench[1],wrench[2])
    o   = vertcat(ox,oy,oz)
    R = eulzyx2rot(phi,th,psi)
    F = -tau[0]/(w+d*cos(varphi))
    T = wrench[3]

    fB = SX.zeros(3,1)   
    fB[1] = F*sin(varphi)
    fB[2] = -T*cos(varphi)

    # z is pointing down dynamics
    pdd = 1/m * R@fB + vertcat(0,0,g)
    chid = getEinvZYX(phi,th,psi) @ R @ o
    od = Jinv@(tau - cross(o,J@o))

    # finally write dynamics
    f_expl = vertcat(dx_,dy,dz,pdd[0],pdd[1],pdd[2],chid[2],chid[1],chid[0],od[0],od[1],od[2])
    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model
