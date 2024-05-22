from casadi import SX, vertcat, sin, cos, inv, Function, jacobian
import numpy as np
from acados_template import AcadosModel
from morphing_lander.parameters import params_

# get robot model parameters 
# note:(g,m_rotor,m are inactive in robot equations; The rest are active)
g        = params_.get('g')                                 
kT       =  params_.get('kT')
kM       = params_.get('kM')
m_base   = params_.get('m_base')
m_arm    = params_.get('m_arm')
m_rotor  = params_.get('m_rotor')
I_base   = params_.get('I_base')
I_arm_x  = params_.get('I_arm_x')
I_arm_yz = params_.get('I_arm_yz')

def M_(x,phi):
    # get state variables
    x_B      = x[0]
    y_B      = x[1]
    z_B      = x[2]
    theta_Bz = x[3]
    theta_By = x[4]
    theta_Bx = x[5]
    dx_B     = x[6]
    dy_B     = x[7]
    dz_B     = x[8]
    omega_Bx = x[9]
    omega_By = x[10]
    omega_Bz = x[11]

    out = SX(
        np.array([
[                                                                                                                                                                2.0*m_arm + m_base + 0.028,                                                                                                                                                                                     0,                                                                                                                                       0, -0.00004*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By))*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),                                                                                                           -0.00004*cos(theta_By)*cos(theta_Bz)*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),                                                                                                                                                                                                                                                  0],
[                                                                                                                                                                                         0,                                                                                                                                                            2.0*m_arm + m_base + 0.028,                                                                                                                                       0,      0.00004*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz))*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),                                                                                                           -0.00004*cos(theta_By)*sin(theta_Bz)*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),                                                                                                                                                                                                                                                  0],
[                                                                                                                                                                                         0,                                                                                                                                                                                     0,                                                                                                              2.0*m_arm + m_base + 0.028,                                                    0.00004*cos(theta_By)*sin(theta_Bx)*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),                                                                                                                          0.00004*sin(theta_By)*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),                                                                                                                                                                                                                                                  0],
[-0.00004*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By))*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0), 0.00004*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz))*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0), 0.00004*cos(theta_By)*sin(theta_Bx)*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),                      2.0*I_arm_x + I_base + 0.01*m_arm + 0.0005824*cos(phi) + 0.0001568*sin(phi) + 0.03752*m_arm*cos(phi) + 0.00264*m_arm*sin(phi) + 0.00081759999999999999957998007837912,                                                                                                                                                                                                                                                  0,                                                                                                                                                                                                                                                  0],
[                                                  -0.00004*cos(theta_By)*cos(theta_Bz)*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),                                              -0.00004*cos(theta_By)*sin(theta_Bz)*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),               0.00004*sin(theta_By)*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),                                                                                                                                                                                          0, 2.0*I_arm_yz + I_base + 0.0002*m_arm - 0.0002352*sin(2.0*phi) + 0.0000336*cos(phi) - 0.0000784*sin(phi) - 0.00048000000000000000194007136744556*cos(phi)**2 + 0.00112*m_arm*cos(phi) - 0.0052*m_arm*sin(phi) + 0.0014208000000000000015200514458247,                                                                                                                                                                                                                                                  0],
[                                                                                                                                                                                         0,                                                                                                                                                                                     0,                                                                                                                                       0,                                                                                                                                                                                          0,                                                                                                                                                                                                                                                  0, 2.0*I_arm_yz + I_base + 0.0098*m_arm + 0.0002352*sin(2.0*phi) + 0.0005488*cos(phi) + 0.0002352*sin(phi) - 0.00048000000000000000194007136744556*sin(phi)**2 + 0.0364*m_arm*cos(phi) + 0.00784*m_arm*sin(phi) + 0.0015552000000000000015200514458247]
        ])
    )
    return out

def b_(x,phi):
    # get state variables
    x_B      = x[0]
    y_B      = x[1]
    z_B      = x[2]
    theta_Bz = x[3]
    theta_By = x[4]
    theta_Bx = x[5]
    dx_B     = x[6]
    dy_B     = x[7]
    dz_B     = x[8]
    omega_Bx = x[9]
    omega_By = x[10]
    omega_Bz = x[11]

    out = SX(
        np.array([
0.00004*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0)*(omega_Bx**2*sin(theta_Bx)*sin(theta_Bz) + omega_By**2*sin(theta_Bx)*sin(theta_Bz) + omega_Bx**2*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) + omega_By**2*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 1.0*omega_Bx*omega_Bz*cos(theta_By)*cos(theta_Bz) + omega_By*omega_Bz*cos(theta_Bx)*sin(theta_Bz) - 1.0*omega_By*omega_Bz*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)),
-0.00004*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0)*(omega_Bx**2*cos(theta_Bz)*sin(theta_Bx) + omega_By**2*cos(theta_Bz)*sin(theta_Bx) - 1.0*omega_Bx**2*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 1.0*omega_By**2*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) + omega_By*omega_Bz*cos(theta_Bx)*cos(theta_Bz) + omega_Bx*omega_Bz*cos(theta_By)*sin(theta_Bz) + omega_By*omega_Bz*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)),
                                                                                                                                                                                               0.00004*(cos(theta_Bx)*cos(theta_By)*omega_Bx**2 + omega_Bz*sin(theta_By)*omega_Bx + cos(theta_Bx)*cos(theta_By)*omega_By**2 - 1.0*omega_Bz*cos(theta_By)*sin(theta_Bx)*omega_By)*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),
                                                                                 0.000000000000000000000000043368086899420177360298112034798*omega_By*omega_Bz*(221360928884514619392000.0*m_arm + 11879703183468951240704.0*cos(phi) + 7231123676894144233472.0*sin(phi) + 21693371030682432700416.0*cos(phi)*sin(phi) + 22136092888451462028670.0*cos(phi)**2 + 813501413650591226265600.0*m_arm*cos(phi) + 300681928401465691340800.0*m_arm*sin(phi) - 7968993439842526342847.0),
            -0.000000000000000000000000043368086899420177360298112034798*omega_Bx*omega_Bz*(46116860184273879040000000.0*I_arm_yz - 46116860184273879040000000.0*I_arm_x - 4611686018427387904000.0*m_arm + 5423342757670608175104.0*sin(2.0*phi) - 774763251095801167872.0*cos(phi) + 1807780919223536058368.0*sin(phi) + 11068046444225731014335.0*cos(phi)**2 - 25825441703193372262400.0*m_arm*cos(phi) + 119903836479112085504000.0*m_arm*sin(phi) + 5939851591734475620352.0),
      -0.000000000000000000000000043368086899420177360298112034798*omega_Bx*omega_By*(46116860184273879040000000.0*I_arm_x - 46116860184273879040000000.0*I_arm_yz + 225972614902942007296000.0*m_arm + 5423342757670608175104.0*sin(2.0*phi) + 12654466434564752408576.0*cos(phi) + 5423342757670608175104.0*sin(phi) + 11068046444225731014335.0*cos(phi)**2 + 839326855353784598528000.0*m_arm*cos(phi) + 180778091922353605836800.0*m_arm*sin(phi) - 13908845031577001963199.0)        ])
    )
    return out

def g_(x,phi):
    # get state variables
    x_B      = x[0]
    y_B      = x[1]
    z_B      = x[2]
    theta_Bz = x[3]
    theta_By = x[4]
    theta_Bx = x[5]
    dx_B     = x[6]
    dy_B     = x[7]
    dz_B     = x[8]
    omega_Bx = x[9]
    omega_By = x[10]
    omega_Bz = x[11]

    out = SX(
        np.array([
                                                                                                                                         0,
                                                                                                                                         0,
                                                                                                     - 19.62*m_arm - 9.81*m_base - 0.27468,
-0.0003924*cos(theta_By)*sin(theta_Bx)*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),
              -0.0003924*sin(theta_By)*(500.0*m_arm + 42.0*cos(phi) - 98.0*sin(phi) + 1400.0*m_arm*cos(phi) - 6500.0*m_arm*sin(phi) + 7.0),
                                                                                                                                         0        ])
    )
    return out

def S_(x,phi):
    # get state variables
    x_B      = x[0]
    y_B      = x[1]
    z_B      = x[2]
    theta_Bz = x[3]
    theta_By = x[4]
    theta_Bx = x[5]
    dx_B     = x[6]
    dy_B     = x[7]
    dz_B     = x[8]
    omega_Bx = x[9]
    omega_By = x[10]
    omega_Bz = x[11]

    out = SX(
        np.array([
[- 1.0*kT*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)) - 1.0*kT*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)),     kT*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)) + kT*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)),     -1.0*kT*cos(phi + theta_Bx)*cos(theta_By), -0.01*kT*(7.0*cos(phi) - 1.0*sin(phi) + 14.0),  0.01*kT*(17.0*cos(phi) - 100.0*kM*sin(phi)),  0.01*kT*(17.0*sin(phi) + 100.0*kM*cos(phi))],
[      kT*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)) - 1.0*kT*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)), kT*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)) - 1.0*kT*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)), -1.0*kT*cos(phi - 1.0*theta_Bx)*cos(theta_By),  0.01*kT*(7.0*cos(phi) - 1.0*sin(phi) + 14.0), -0.01*kT*(17.0*cos(phi) - 100.0*kM*sin(phi)),  0.01*kT*(17.0*sin(phi) + 100.0*kM*cos(phi))],
[      kT*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)) - 1.0*kT*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)), kT*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)) - 1.0*kT*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)), -1.0*kT*cos(phi - 1.0*theta_Bx)*cos(theta_By),  0.01*kT*(7.0*cos(phi) - 1.0*sin(phi) + 14.0),  0.01*kT*(17.0*cos(phi) - 100.0*kM*sin(phi)), -0.01*kT*(17.0*sin(phi) + 100.0*kM*cos(phi))],
[- 1.0*kT*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)) - 1.0*kT*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)),     kT*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)) + kT*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)),     -1.0*kT*cos(phi + theta_Bx)*cos(theta_By), -0.01*kT*(7.0*cos(phi) - 1.0*sin(phi) + 14.0), -0.01*kT*(17.0*cos(phi) - 100.0*kM*sin(phi)), -0.01*kT*(17.0*sin(phi) + 100.0*kM*cos(phi))]        ])
    )
    return out

def F(theta_Bx,theta_By,theta_Bz):
    # transforms omega to chi
    out = SX(
        np.array([
        [0,                 sin(theta_Bx)/cos(theta_By),                 cos(theta_Bx)/cos(theta_By)],
        [0,                               cos(theta_Bx),                              -sin(theta_Bx)],
        [1, (sin(theta_Bx)*sin(theta_By))/cos(theta_By), (cos(theta_Bx)*sin(theta_By))/cos(theta_By)]
    ])
    )
    return out 

def G(theta_Bx,theta_By,theta_Bz):
    # transforms chi to omega
    out = SX(
        np.array([
            [     -sin(theta_By),              0, 1],
            [cos(theta_By)*sin(theta_Bx),  cos(theta_Bx), 0],
            [cos(theta_Bx)*cos(theta_By), -sin(theta_Bx), 0]
        ])
    )
    return out

def dynamics(X,U,varphi):
    # get state variables
    x_B      = X[0]
    y_B      = X[1]
    z_B      = X[2]
    theta_Bz = X[3]
    theta_By = X[4]
    theta_Bx = X[5]
    dx_B     = X[6]
    dy_B     = X[7]
    dz_B     = X[8]
    omega_Bx = X[9]
    omega_By = X[10]
    omega_Bz = X[11]

    # q, u
    q = vertcat(x_B,y_B,z_B,theta_Bz,theta_By,theta_Bx)
    u = vertcat(dx_B,dy_B,dz_B,omega_Bx,omega_By,omega_Bz)

    # get dynamics matrices
    M_x = M_(X,varphi)
    b_x = b_(X,varphi)
    g_x = g_(X,varphi)  
    S_x = S_(X,varphi)

    # compute inverse mass matrix
    Minv = inv(M_x)
    du = Minv @ (S_x.T @ U - b_x - g_x)

    chid = F(theta_Bx,theta_By,theta_Bz) @ vertcat(omega_Bx,omega_By,omega_Bz)

    # finally write explicit dynamics
    f_expl = vertcat(u[0],u[1],u[2],chid,du)

    return f_expl

def f(x,u,varphi):
    X = SX.sym("X",12)
    U = SX.sym("U",4)    
    Varphi = SX.sym("Varphi",1)
    func = Function('f',[X,U,Varphi],[dynamics(X,U,Varphi)],['X','U','Varphi'],['f'])
    return np.array(func(x,u,varphi))

def A(x,u,varphi):
    X = SX.sym("X",12)
    U = SX.sym("U",4)   
    func = Function('A',[X,U],[jacobian(dynamics(X,U,varphi),X)],['X','U'],['A'])
    return np.array(func(x,u))

def B(x,u,varphi):
    X = SX.sym("X",12)
    U = SX.sym("U",4)    
    func = Function('B',[X,U],[jacobian(dynamics(X,U,varphi),U)],['X','U'],['B'])
    return np.array(func(x,u))

def export_robot_model() -> AcadosModel:
    model_name = "torque_dynamics"

    # tilt angle
    varphi = SX.sym("varphi",1)
    p = varphi

    # states and controls
    X = SX.sym("X",12)
    U = SX.sym("U",4)

    # Explicit dynamics
    f_expl = dynamics(X,U,varphi)

    # xdot
    x_B_dot = SX.sym("x_B_dot")
    y_B_dot = SX.sym("y_B_dot")
    z_B_dot = SX.sym("z_B_dot")
    theta_Bz_dot = SX.sym("theta_Bz_dot")
    theta_By_dot = SX.sym("theta_By_dot")
    theta_Bx_dot = SX.sym("theta_Bx_dot")
    dx_B_dot = SX.sym("dx_B_dot")
    dy_B_dot = SX.sym("dy_B_dot")
    dz_B_dot = SX.sym("dz_B_dot")
    omega_Bx_dot = SX.sym("omega_Bx_dot")
    omega_By_dot = SX.sym("omega_By_dot")
    omega_Bz_dot = SX.sym("omega_Bz_dot")

    Xdot = vertcat(x_B_dot,y_B_dot,z_B_dot,theta_Bz_dot,theta_By_dot,theta_Bx_dot,dx_B_dot,dy_B_dot,dz_B_dot,omega_Bx_dot,omega_By_dot,omega_Bz_dot)

    # write implicit dynamics 
    f_impl = Xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = X
    model.xdot = Xdot
    model.u = U
    # model.z = z
    model.p = p
    model.name = model_name

    return model

def F_numeric(thetax,thetay,thetaz):
    # Create symbolic variables
    theta_Bx = SX.sym('theta_Bx', 1)
    theta_By = SX.sym('theta_By', 1)
    theta_Bz = SX.sym('theta_Bz', 1)

    # Call the original function with symbolic variables
    symbolic_result = F(theta_Bx,theta_By,theta_Bz)

    # Create a function to evaluate the symbolic expression numerically
    numerical_function = Function('F_numeric', [theta_Bx,theta_By,theta_Bz], [symbolic_result])

    # Evaluate the symbolic expression numerically
    numerical_result = numerical_function(thetax,thetay,thetaz)

    return np.array(numerical_result)

def S_numeric(X,phi):
    # Create symbolic variables
    X_sym = SX.sym('X', len(X))
    phi_sym = SX.sym('phi',1)

    # Call the original function with symbolic variables
    symbolic_result = S_(X_sym,phi_sym)

    # Create a function to evaluate the symbolic expression numerically
    numerical_function = Function('S_numeric', [X_sym,phi_sym], [symbolic_result])

    # Evaluate the symbolic expression numerically
    numerical_result = numerical_function(X,phi)

    return np.array(numerical_result)
