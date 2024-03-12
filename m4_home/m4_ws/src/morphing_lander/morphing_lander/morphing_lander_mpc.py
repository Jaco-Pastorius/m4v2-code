# Acados/Casadi
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, sin, cos, inv, Function
import scipy.linalg
import numpy as np

# robot model parameters
kT, kM = 29.0, 0.09
m_base, m_arm = 2.75, 1.31  # 2.75
I_base, I_arm_x, I_arm_yz = 0.01, 0.01, 0.04

# initial state
START_POSITION = [0.0, 0.0, -2.5] 
X0 = np.array([START_POSITION[0],START_POSITION[1],START_POSITION[2],0,0,0,0,0,0,0,0,0,0]) 

# reference states and inputs
x_ref,y_ref,z_ref = 0,0,0.1
psi_ref,theta_ref,phi_ref = 0,0,0
dx_ref,dy_ref,dz_ref = 0,0,0
ox_ref,oy_ref,oz_ref = 0,0,0
varphi_ref = np.pi/2
u_ref,v_ref = 0,0

X_ref = np.array([x_ref,y_ref,z_ref,psi_ref,theta_ref,phi_ref,dx_ref,dy_ref,dz_ref,ox_ref,oy_ref,oz_ref,varphi_ref]) 
U_ref = np.array([u_ref,u_ref,u_ref,u_ref,v_ref])               

# cost function parameters
w_x,w_y,w_z = 0,0,1
w_dx,w_dy,w_dz = 1,1,1
w_phi,w_th,w_psi = 1,1,0
w_ox,w_oy,w_oz = 7,7,1
w_varphi = 1
w_u, w_v = 1,1

rho = 1e-2

Q_mat = np.diag([w_x,w_y,w_z,w_psi,w_th,w_phi,w_dx,w_dy,w_dz,w_ox,w_oy,w_oz,w_varphi])
R_mat = rho * np.diag([w_u,w_u,w_u,w_u,w_v])

# constraint variables
u_max = 1.0 
v_max = 1.0
varphi_max = np.pi / 3  # limit max tilting angle to 60 degrees (not necessary but safer for first try)

# maximum tilt velocity
v_max_absolute = (np.pi/2) / 4

# collocation parameters
N_horizon = 10   # Define the number of discretization steps
T_horizon = 1.0  # Define the prediction horizon

def M_(q,phi):
    x_B = q[0]
    y_B = q[1]
    z_B = q[2]
    theta_Bz = q[3]
    theta_By = q[4]
    theta_Bx = q[5]

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

def b_(q,u,phi):
    x_B = q[0]
    y_B = q[1]
    z_B = q[2]
    theta_Bz = q[3]
    theta_By = q[4]
    theta_Bx = q[5]

    dx_B = u[0]
    dy_B = u[1]
    dz_B = u[2]
    omega_Bx = u[3]
    omega_By = u[4]
    omega_Bz = u[5]

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

def g_(q,phi):
    x_B = q[0]
    y_B = q[1]
    z_B = q[2]
    theta_Bz = q[3]
    theta_By = q[4]
    theta_Bx = q[5]

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

def S_(q,phi):
    x_B = q[0]
    y_B = q[1]
    z_B = q[2]
    theta_Bz = q[3]
    theta_By = q[4]
    theta_Bx = q[5]

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

def export_robot_model() -> AcadosModel:
    model_name = "torque_dynamics"

    # states
    X = SX.sym("X",13)
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
    varphi   = X[12]

    # controls
    U = SX.sym("U",5)

    # q, u
    q = vertcat(x_B,y_B,z_B,theta_Bz,theta_By,theta_Bx)
    u = vertcat(dx_B,dy_B,dz_B,omega_Bx,omega_By,omega_Bz)

    # get dynamics matrices
    M_x = M_(q,varphi)
    b_x = b_(q,u,varphi)
    g_x = g_(q,varphi)  
    S_x = S_(q,varphi)

    # compute inverse mass matrix
    Minv = inv(M_x)
    du = Minv @ (S_x.T @ U[:-1] - b_x - g_x)

    chid = F(theta_Bx,theta_By,theta_Bz) @ vertcat(omega_Bx,omega_By,omega_Bz)

    # finally write explicit dynamics
    f_expl = vertcat(u[0],u[1],u[2],chid[0],chid[1],chid[2],du,v_max_absolute*U[4])

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
    varphi_dot = SX.sym("varphi_dot")

    Xdot = vertcat(x_B_dot,y_B_dot,z_B_dot,theta_Bz_dot,theta_By_dot,theta_Bx_dot,dx_B_dot,dy_B_dot,dz_B_dot,omega_Bx_dot,omega_By_dot,omega_Bz_dot,varphi_dot)

    # write implicit dynamics 
    f_impl = Xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = X
    model.xdot = Xdot
    model.u = U
    # model.z = z
    # model.p = p
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

    ocp.cost.W_e = Q_mat
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # set constraints
    ocp.constraints.lbu = np.array([0,0,0,0,-v_max])
    ocp.constraints.ubu = np.array([+u_max,+u_max,+u_max,+u_max,+v_max])
    ocp.constraints.idxbu = np.array([0,1,2,3,4])

    ocp.constraints.lbx = np.array([0])
    ocp.constraints.ubx = np.array([varphi_max])
    ocp.constraints.idxbx = np.array([12])
    
    ocp.constraints.x0 = X0

    # set parameters
    # ocp.parameter_values = varphi

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

# compute max torque and thrust
def S_numeric(q, phi):
    # Create symbolic variables
    q_sym = SX.sym('q', len(q))
    phi_sym = SX.sym('phi')

    # Call the original function with symbolic variables
    symbolic_result = S_(q_sym, phi_sym)

    # Create a function to evaluate the symbolic expression numerically
    numerical_function = Function('S_numeric', [q_sym, phi_sym], [symbolic_result])

    # Evaluate the symbolic expression numerically
    numerical_result = numerical_function(q, phi)

    return np.array(numerical_result)

S_temp = S_numeric(np.array([0,0,0,0,0,0]),0.0).T
tau_max_x = np.sum(np.abs(S_temp[3,:]))/2
tau_max_y = np.sum(np.abs(S_temp[4,:]))/2
tau_max_z = np.sum(np.abs(S_temp[5,:]))/2
T_max = np.sum(np.abs(S_temp[2,:]))
