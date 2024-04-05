# Acados/Casadi
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, sin, cos, inv, Function, jacobian
import scipy.linalg
import numpy as np
from IPython import embed
from control import lqr, ctrb

# control frequency
Ts = 0.01

# robot model parameters
g = 9.81
kT, kM = 29.0, 0.09
m_base, m_arm, m_rotor = 2.75, 1.31, 0.007
I_base, I_arm_x, I_arm_yz = 0.01, 0.01, 0.04

# total mass
m = m_base + 2*m_arm + 4*m_rotor

# initial state
varphi_0 = 0.0
START_POSITION = [0.0, 0.0, -3.5] 
X0 = np.array([START_POSITION[0],START_POSITION[1],START_POSITION[2],0,0,0,0,0,0,0,0,0]) 

# reference states and inputs
x_ref,y_ref,z_ref = 0,0,-3.5
psi_ref,theta_ref,phi_ref = 0,0,0   
dx_ref,dy_ref,dz_ref = 0.0,0,0.1
ox_ref,oy_ref,oz_ref = 0,0,0
u_ref = 0
              
X_ref = np.array([x_ref,y_ref,z_ref,psi_ref,theta_ref,phi_ref,dx_ref,dy_ref,dz_ref,ox_ref,oy_ref,oz_ref]) 
U_ref = np.array([u_ref,u_ref,u_ref,u_ref])  

# cost function parameters
w_x,w_y,w_z = 0.1,0.1,0.1
w_dx,w_dy,w_dz = 1,1,1
# w_dx,w_dy,w_dz = 1,1,1
w_phi,w_th,w_psi = 0.1,0.1,0.1
w_ox,w_oy,w_oz = 0.5,0.5,0.5
w_u = 1.0

rho = 0.1
beta = 1.0

Q_mat = np.diag([w_x,w_y,w_z,w_psi,w_th,w_phi,w_dx,w_dy,w_dz,w_ox,w_oy,w_oz])
R_mat = rho * np.diag([w_u,w_u,w_u,w_u])
Q_mat_terminal = beta * Q_mat 

# constraint variables
u_max = 1.0 
v_max = 1.0

# maximum tilt velocity
v_max_absolute = (np.pi/2) / 4

# collocation parameters
N_horizon = 10   # Define the number of discretization steps
T_horizon = 1.0 # Define the prediction horizon

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
    # phi   = x[12]

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
    # phi   = x[12]

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
    # phi   = x[12]

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
    # phi   = x[12]

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
    # varphi   = X[12]

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
    # du = Minv @ (S_x.T @ U[:-1] - b_x - g_x)
    du = Minv @ (S_x.T @ U - b_x - g_x)

    chid = F(theta_Bx,theta_By,theta_Bz) @ vertcat(omega_Bx,omega_By,omega_Bz)

    # finally write explicit dynamics
    # f_expl = vertcat(u[0],u[1],u[2],chid,du,v_max_absolute*U[4])
    f_expl = vertcat(u[0],u[1],u[2],chid,du)

    return f_expl

def f(x,u):
    X = SX.sym("X",12)
    U = SX.sym("U",4)    
    func = Function('f',[X,U],[dynamics(X,U)],['X','U'],['f'])
    return np.array(func(x,u))

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

# def A_omega(x_omega,phi):
#     omega_Bx,omega_By,omega_Bz = x_omega
#     out = np.array([
#         [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0, -(1.0*omega_Bz*(1.9322020054045735548027673975285e+53*cos(3.0*phi) - 3.9982089239497186829571462516849e+52*cos(4.0*phi) - 3.0178910067238574347778157168223e+53*sin(2.0*phi) - 4.3428797520008405234598433090795e+52*sin(4.0*phi) - 4.8938433429152475311635552222303e+53*cos(2.0*phi) + 1.8948050416243710436549222602479e+53*sin(3.0*phi) + 3.9554487903063979816877507260111e+54*cos(phi) + 1.0168188323916383573237822197419e+54*sin(phi) + 1.8166415426591058396379575138197e+54))/(1.1589703922631411195659300040326e+54*cos(2.0*phi) + 4.1322891664547783564689543775654e+52*cos(4.0*phi) + 4.3168353467469373378275404788945e+53*sin(2.0*phi) + 4.6817535981853315590671451420278e+52*sin(4.0*phi) + 2.3933023404217881884403186055982e+53*cos(3.0*phi) + 1.2666217940227745215581039760657e+53*sin(3.0*phi) + 4.0636544495937640486325905769342e+54*cos(phi) + 5.2375970506644012143180086920645e+53*sin(phi) + 2.5622299927816198749650484921719e+54), -(1.0*omega_By*(1.9322020054045735548027673975285e+53*cos(3.0*phi) - 3.9982089239497186829571462516849e+52*cos(4.0*phi) - 3.0178910067238574347778157168223e+53*sin(2.0*phi) - 4.3428797520008405234598433090795e+52*sin(4.0*phi) - 4.8938433429152475311635552222303e+53*cos(2.0*phi) + 1.8948050416243710436549222602479e+53*sin(3.0*phi) + 3.9554487903063979816877507260111e+54*cos(phi) + 1.0168188323916383573237822197419e+54*sin(phi) + 1.8166415426591058396379575138197e+54))/(1.1589703922631411195659300040326e+54*cos(2.0*phi) + 4.1322891664547783564689543775654e+52*cos(4.0*phi) + 4.3168353467469373378275404788945e+53*sin(2.0*phi) + 4.6817535981853315590671451420278e+52*sin(4.0*phi) + 2.3933023404217881884403186055982e+53*cos(3.0*phi) + 1.2666217940227745215581039760657e+53*sin(3.0*phi) + 4.0636544495937640486325905769342e+54*cos(phi) + 5.2375970506644012143180086920645e+53*sin(phi) + 2.5622299927816198749650484921719e+54)],
#         [(omega_Bz*(3.7913723981345934070013357173148e+53*cos(2.0*phi) - 4.1322891664547783564689543775654e+52*cos(4.0*phi) + 2.7171809680333015068685519546548e+53*sin(2.0*phi) - 4.6817535981853315590671451420278e+52*sin(4.0*phi) - 2.3933023404217881884403186055982e+53*cos(3.0*phi) - 1.2666217940227745215581039760657e+53*sin(3.0*phi) + 3.1325226166304634803193599529428e+54*cos(phi) + 5.0363027317358175114722163654173e+53*sin(phi) + 2.177493208822260330903117109145e+54))/(1.1589703922631411195659300040326e+54*cos(2.0*phi) + 4.1322891664547783564689543775654e+52*cos(4.0*phi) + 4.3168353467469373378275404788945e+53*sin(2.0*phi) + 4.6817535981853315590671451420278e+52*sin(4.0*phi) + 2.3933023404217881884403186055982e+53*cos(3.0*phi) + 1.2666217940227745215581039760657e+53*sin(3.0*phi) + 4.0636544495937640486325905769342e+54*cos(phi) + 5.2375970506644012143180086920645e+53*sin(phi) + 2.5622299927816198749650484921719e+54),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               0,       (omega_Bx*(3.7913723981345934070013357173148e+53*cos(2.0*phi) - 4.1322891664547783564689543775654e+52*cos(4.0*phi) + 2.7171809680333015068685519546548e+53*sin(2.0*phi) - 4.6817535981853315590671451420278e+52*sin(4.0*phi) - 2.3933023404217881884403186055982e+53*cos(3.0*phi) - 1.2666217940227745215581039760657e+53*sin(3.0*phi) + 3.1325226166304634803193599529428e+54*cos(phi) + 5.0363027317358175114722163654173e+53*sin(phi) + 2.177493208822260330903117109145e+54))/(1.1589703922631411195659300040326e+54*cos(2.0*phi) + 4.1322891664547783564689543775654e+52*cos(4.0*phi) + 4.3168353467469373378275404788945e+53*sin(2.0*phi) + 4.6817535981853315590671451420278e+52*sin(4.0*phi) + 2.3933023404217881884403186055982e+53*cos(3.0*phi) + 1.2666217940227745215581039760657e+53*sin(3.0*phi) + 4.0636544495937640486325905769342e+54*cos(phi) + 5.2375970506644012143180086920645e+53*sin(phi) + 2.5622299927816198749650484921719e+54)],
#         [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  (omega_By*(5423342757670608175104.0*sin(2.0*phi) + 1112172646948022576480256.0*cos(phi) + 11068046444225731014335.0*cos(phi)**2 - 1101390525036939343605439.0) + 242242643175953831821312.0*omega_By*sin(phi))/(5423342757670608175104.0*sin(2.0*phi) + 1112172646948022576480256.0*cos(phi) + 242242643175953831821312.0*sin(phi) + 11068046444225731014335.0*cos(phi)**2 + 2396075257850244223719979.0),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         (omega_Bx*(5423342757670608175104.0*sin(2.0*phi) + 1112172646948022576480256.0*cos(phi) + 11068046444225731014335.0*cos(phi)**2 - 1101390525036939343605439.0) + 242242643175953831821312.0*omega_Bx*sin(phi))/(5423342757670608175104.0*sin(2.0*phi) + 1112172646948022576480256.0*cos(phi) + 242242643175953831821312.0*sin(phi) + 11068046444225731014335.0*cos(phi)**2 + 2396075257850244223719979.0),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               0]    ])
#     return out

# def B_omega(phi):
#     out = np.array([
#         [-(6686944726719712460800000.0*(15412723672238736592937768587589.0*cos(2.0*phi) + 619161492516954992731630072373.44*cos(4.0*phi) + 3872104709049204888359953176921.5*sin(2.0*phi) + 454478168144961836684724491353.75*sin(4.0*phi) + 1655120636954121128422678385264.9*cos(3.0*phi) + 335844155868095948807782945106.69*sin(3.0*phi) + 25258622441450040562892883925344.0*cos(phi) - 2275256293988592914190528555705.8*sin(phi) + 37121356445185360548873326089256.0))/(1.1589703922631411195659300040326e+54*cos(2.0*phi) + 4.1322891664547783564689543775654e+52*cos(4.0*phi) + 4.3168353467469373378275404788945e+53*sin(2.0*phi) + 4.6817535981853315590671451420278e+52*sin(4.0*phi) + 2.3933023404217881884403186055982e+53*cos(3.0*phi) + 1.2666217940227745215581039760657e+53*sin(3.0*phi) + 4.0636544495937640486325905769342e+54*cos(phi) + 5.2375970506644012143180086920645e+53*sin(phi) + 2.5622299927816198749650484921719e+54), (6686944726719712460800000.0*(15412723672238736592937768587589.0*cos(2.0*phi) + 619161492516954992731630072373.44*cos(4.0*phi) + 3872104709049204888359953176921.5*sin(2.0*phi) + 454478168144961836684724491353.75*sin(4.0*phi) + 1655120636954121128422678385264.9*cos(3.0*phi) + 335844155868095948807782945106.69*sin(3.0*phi) + 25258622441450040562892883925344.0*cos(phi) - 2275256293988592914190528555705.8*sin(phi) + 37121356445185360548873326089256.0))/(1.1589703922631411195659300040326e+54*cos(2.0*phi) + 4.1322891664547783564689543775654e+52*cos(4.0*phi) + 4.3168353467469373378275404788945e+53*sin(2.0*phi) + 4.6817535981853315590671451420278e+52*sin(4.0*phi) + 2.3933023404217881884403186055982e+53*cos(3.0*phi) + 1.2666217940227745215581039760657e+53*sin(3.0*phi) + 4.0636544495937640486325905769342e+54*cos(phi) + 5.2375970506644012143180086920645e+53*sin(phi) + 2.5622299927816198749650484921719e+54), (6686944726719712460800000.0*(15412723672238736592937768587589.0*cos(2.0*phi) + 619161492516954992731630072373.44*cos(4.0*phi) + 3872104709049204888359953176921.5*sin(2.0*phi) + 454478168144961836684724491353.75*sin(4.0*phi) + 1655120636954121128422678385264.9*cos(3.0*phi) + 335844155868095948807782945106.69*sin(3.0*phi) + 25258622441450040562892883925344.0*cos(phi) - 2275256293988592914190528555705.8*sin(phi) + 37121356445185360548873326089256.0))/(1.1589703922631411195659300040326e+54*cos(2.0*phi) + 4.1322891664547783564689543775654e+52*cos(4.0*phi) + 4.3168353467469373378275404788945e+53*sin(2.0*phi) + 4.6817535981853315590671451420278e+52*sin(4.0*phi) + 2.3933023404217881884403186055982e+53*cos(3.0*phi) + 1.2666217940227745215581039760657e+53*sin(3.0*phi) + 4.0636544495937640486325905769342e+54*cos(phi) + 5.2375970506644012143180086920645e+53*sin(phi) + 2.5622299927816198749650484921719e+54), -(6686944726719712460800000.0*(15412723672238736592937768587589.0*cos(2.0*phi) + 619161492516954992731630072373.44*cos(4.0*phi) + 3872104709049204888359953176921.5*sin(2.0*phi) + 454478168144961836684724491353.75*sin(4.0*phi) + 1655120636954121128422678385264.9*cos(3.0*phi) + 335844155868095948807782945106.69*sin(3.0*phi) + 25258622441450040562892883925344.0*cos(phi) - 2275256293988592914190528555705.8*sin(phi) + 37121356445185360548873326089256.0))/(1.1589703922631411195659300040326e+54*cos(2.0*phi) + 4.1322891664547783564689543775654e+52*cos(4.0*phi) + 4.3168353467469373378275404788945e+53*sin(2.0*phi) + 4.6817535981853315590671451420278e+52*sin(4.0*phi) + 2.3933023404217881884403186055982e+53*cos(3.0*phi) + 1.2666217940227745215581039760657e+53*sin(3.0*phi) + 4.0636544495937640486325905769342e+54*cos(phi) + 5.2375970506644012143180086920645e+53*sin(phi) + 2.5622299927816198749650484921719e+54)],
#         [                                                                                                                            (18048063817416503931699200000.0*(8849097247246359555584666740.5*cos(phi) - 5010769797247047984742400000.0*sin(2.0*phi) - 6903812536033333460313382672.5*sin(phi) + 27878444322527045876627537920.0*cos(phi)**2 + 13762548193196408602068254720.0*cos(phi)**3 + 798817298082456396166594560.0*sin(phi)**3 - 1959076843694848736961232896.0))/(2.3179407845262822391318600080651e+54*cos(2.0*phi) + 8.2645783329095567129379087551307e+52*cos(4.0*phi) + 8.6336706934938746756550809577889e+53*sin(2.0*phi) + 9.3635071963706631181342902840556e+52*sin(4.0*phi) + 4.7866046808435763768806372111965e+53*cos(3.0*phi) + 2.5332435880455490431162079521315e+53*sin(3.0*phi) + 8.1273088991875280972651811538684e+54*cos(phi) + 1.0475194101328802428636017384129e+54*sin(phi) + 5.1244599855632397499300969843438e+54),                                                                                                                           -(18048063817416503931699200000.0*(8849097247246359555584666740.5*cos(phi) - 5010769797247047984742400000.0*sin(2.0*phi) - 6903812536033333460313382672.5*sin(phi) + 27878444322527045876627537920.0*cos(phi)**2 + 13762548193196408602068254720.0*cos(phi)**3 + 798817298082456396166594560.0*sin(phi)**3 - 1959076843694848736961232896.0))/(2.3179407845262822391318600080651e+54*cos(2.0*phi) + 8.2645783329095567129379087551307e+52*cos(4.0*phi) + 8.6336706934938746756550809577889e+53*sin(2.0*phi) + 9.3635071963706631181342902840556e+52*sin(4.0*phi) + 4.7866046808435763768806372111965e+53*cos(3.0*phi) + 2.5332435880455490431162079521315e+53*sin(3.0*phi) + 8.1273088991875280972651811538684e+54*cos(phi) + 1.0475194101328802428636017384129e+54*sin(phi) + 5.1244599855632397499300969843438e+54),                                                                                                                            (18048063817416503931699200000.0*(8849097247246359555584666740.5*cos(phi) - 5010769797247047984742400000.0*sin(2.0*phi) - 6903812536033333460313382672.5*sin(phi) + 27878444322527045876627537920.0*cos(phi)**2 + 13762548193196408602068254720.0*cos(phi)**3 + 798817298082456396166594560.0*sin(phi)**3 - 1959076843694848736961232896.0))/(2.3179407845262822391318600080651e+54*cos(2.0*phi) + 8.2645783329095567129379087551307e+52*cos(4.0*phi) + 8.6336706934938746756550809577889e+53*sin(2.0*phi) + 9.3635071963706631181342902840556e+52*sin(4.0*phi) + 4.7866046808435763768806372111965e+53*cos(3.0*phi) + 2.5332435880455490431162079521315e+53*sin(3.0*phi) + 8.1273088991875280972651811538684e+54*cos(phi) + 1.0475194101328802428636017384129e+54*sin(phi) + 5.1244599855632397499300969843438e+54),                                                                                                                            -(18048063817416503931699200000.0*(8849097247246359555584666740.5*cos(phi) - 5010769797247047984742400000.0*sin(2.0*phi) - 6903812536033333460313382672.5*sin(phi) + 27878444322527045876627537920.0*cos(phi)**2 + 13762548193196408602068254720.0*cos(phi)**3 + 798817298082456396166594560.0*sin(phi)**3 - 1959076843694848736961232896.0))/(2.3179407845262822391318600080651e+54*cos(2.0*phi) + 8.2645783329095567129379087551307e+52*cos(4.0*phi) + 8.6336706934938746756550809577889e+53*sin(2.0*phi) + 9.3635071963706631181342902840556e+52*sin(4.0*phi) + 4.7866046808435763768806372111965e+53*cos(3.0*phi) + 2.5332435880455490431162079521315e+53*sin(3.0*phi) + 8.1273088991875280972651811538684e+54*cos(phi) + 1.0475194101328802428636017384129e+54*sin(phi) + 5.1244599855632397499300969843438e+54)],
#         [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       (6686944726719712460800000.0*(9.0*cos(phi) + 17.0*sin(phi)))/(5423342757670608175104.0*sin(2.0*phi) + 1112172646948022576480256.0*cos(phi) + 242242643175953831821312.0*sin(phi) + 11068046444225731014335.0*cos(phi)**2 + 2396075257850244223719979.0),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       (6686944726719712460800000.0*(9.0*cos(phi) + 17.0*sin(phi)))/(5423342757670608175104.0*sin(2.0*phi) + 1112172646948022576480256.0*cos(phi) + 242242643175953831821312.0*sin(phi) + 11068046444225731014335.0*cos(phi)**2 + 2396075257850244223719979.0),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      -(6686944726719712460800000.0*(9.0*cos(phi) + 17.0*sin(phi)))/(5423342757670608175104.0*sin(2.0*phi) + 1112172646948022576480256.0*cos(phi) + 242242643175953831821312.0*sin(phi) + 11068046444225731014335.0*cos(phi)**2 + 2396075257850244223719979.0),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       -(6686944726719712460800000.0*(9.0*cos(phi) + 17.0*sin(phi)))/(5423342757670608175104.0*sin(2.0*phi) + 1112172646948022576480256.0*cos(phi) + 242242643175953831821312.0*sin(phi) + 11068046444225731014335.0*cos(phi)**2 + 2396075257850244223719979.0)]    
#     ])
#     return out

def export_robot_model() -> AcadosModel:
    model_name = "torque_dynamics"

    # tilt angle
    varphi = SX.sym("varphi",1)
    p = varphi

    # states and controls
    # X = SX.sym("X",13)
    # U = SX.sym("U",5)
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
    # varphi_dot = SX.sym("varphi_dot")

    # Xdot = vertcat(x_B_dot,y_B_dot,z_B_dot,theta_Bz_dot,theta_By_dot,theta_Bx_dot,dx_B_dot,dy_B_dot,dz_B_dot,omega_Bx_dot,omega_By_dot,omega_Bz_dot,varphi_dot)
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

def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    model = export_robot_model()
    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # set dimensions
    ocp.dims.N = N_horizon

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Q_mat_terminal
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.hstack((X_ref,U_ref))
    ocp.cost.yref_e = X_ref

    # set constraints
    ocp.constraints.lbu = np.array([0,0,0,0])
    ocp.constraints.ubu = np.array([+u_max,+u_max,+u_max,+u_max])
    ocp.constraints.idxbu = np.array([0,1,2,3])

    # ocp.constraints.lbx = np.array([-vz_max])
    # ocp.constraints.ubx = np.array([+vz_max])
    # ocp.constraints.idxbx = np.array([8])
    
    ocp.constraints.x0 = X0

    # parameters
    ocp.parameter_values = np.array([varphi_0])
    
    # set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP
    # ocp.solver_options.nlp_solver_max_iter = 400
    # ocp.solver_options.levenberg_marquardt = 1e-2

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    return ocp

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

# compute max torque and thrust
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

S_temp = S_numeric(np.zeros(12),0.0).T
tau_max_x = np.sum(np.abs(S_temp[3,:]))/2
tau_max_y = np.sum(np.abs(S_temp[4,:]))/2
tau_max_z = np.sum(np.abs(S_temp[5,:]))/2
T_max = np.sum(np.abs(S_temp[2,:]))

# Q,R = np.diag([1,1,1,1,1,1,1,1,1,1,1,1]), np.diag([1,1,1,1])
phi = 0.61
Ac = A(X_ref,U_ref,phi)
Bc = B(X_ref,U_ref,phi)
K,_,_ = lqr(Ac,Bc,Q_mat,R_mat,method='slycot')
embed()
# K,_,_ = lqr(Ac,Bc,Q,R)


## DEPRECATED

# x0_dummy = np.zeros(9)

# x0 = np.hstack((x0_dummy, np.array([0,0,0])))
# u0 = np.array([0,0,0,0])
# phi = 0.0
# Q = np.diag([1,1,1,1,1,1,1,1,1,1,1,1])
# R = np.diag([1,1,1,1])
# rk = np.linalg.matrix_rank(ctrb(A(x0,u0,phi),B(x0,u0,phi)))
# Ac = A(x0,u0,phi)[9:.9:]
# Bc = B(x0,u0,phi)[9:,:]
# K,_,_ = lqr(Ac,Bc,Q,R)
# print(K)
# print(rk)
# embed()

# def body_rate(r,y):
#     # declare global variables
#     global ex_prev,ey_prev,ez_prev,ex_i,ey_i,ez_i

#     # compute errors
#     ex = r[0] - y[5]
#     ey = r[1] - y[4]
#     ez = r[2] - y[3]

#     # error derivatives
#     ex_d = (ex - ex_prev)/Ts
#     ey_d = (ey - ey_prev)/Ts
#     ez_d = (ez - ez_prev)/Ts

#     # error integrals
#     ex_i += ex*Ts
#     ey_i += ey*Ts
#     ez_i += ez*Ts

#     taux = kpid(ex,ex_d,ex_i,gainsx)
#     tauy = kpid(ey,ey_d,ey_i,gainsy)
#     tauz = kpid(ez,ez_d,ex_i,gainsz)

#     # update derivative terms
#     ex_prev = ex
#     ey_prev = ey
#     ez_prev = ez

#     return [taux,tauy,tauz]

# def kpid(e,ed,ei,gains):
#     K,P,I,D = gains
#     u = K * (P*e + I*ei + D*ed)
#     return u 

# # LQR attitude controller
# def body_rate_lqr(x,phi,xd):
#     x = np.array(x)
#     xd = np.array(xd)

#     rho = 1
#     Q = np.diag([1,1,1])
#     R = rho*np.diag([1,1,1,1])

#     # get linearized dynamics
#     A_, B_ = A_omega(xd,phi), B_omega(phi)

#     # get LQR gains
#     K_,_,_ = lqr(A_,B_,Q,R)

#     # get control input
#     u = -K_ @ (x - xd)

#     return u

# # PID attitude controller
# def pid_control(x,u):
#     # get state variables
#     p,theta,v,omega,varphi = x[:3],x[3:6],x[6:9],x[9:12],x[12]

#     # get control input
#     c = 0.1 * (p[2]-X_ref[])

#     return u

# # terminal cost function parameters
# # cost function parameters
# t_x,t_y,t_z = 0,0,1
# t_dx,t_dy,t_dz = 0,0,1
# t_phi,t_th,t_psi = 0,0,0
# t_ox,t_oy,t_oz = 0,0,0
# t_varphi = 1

# Q_mat_terminal = beta * np.diag([t_x,t_y,t_z,t_psi,t_th,t_phi,t_dx,t_dy,t_dz,t_ox,t_oy,t_oz,t_varphi])
