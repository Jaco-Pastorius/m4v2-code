from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosModel
from casadi import SX, vertcat, sin, cos, inv
import numpy as np
import scipy.linalg
import time
import matplotlib.pyplot as plt
from acados_template import latexify_plot
from IPython import embed

# set parameters
X0 = np.array([0,0,0.0,0,0,0,0,0,0,0,0,0,np.deg2rad(0)])  # Initialize the states
X_ref = np.array([0,0,-2.5,0.0,0.0,0.0,0,0,0,0,0,0,np.pi/2])
U_ref = np.array([0.5,0.5,0.5,0.5,0])
N_horizon = 40    # Define the number of discretization steps
T_horizon = 4.0   # Define the prediction horizon
f_max = 1.0       # Define the max force allowed
v_max = np.pi/2 
varphi_max = np.pi/2
varphi_e = np.deg2rad(80)

# set cost
Q_mat = np.diag([0.01,0.01,1,0.01,0.01,1,0.01,0.01,0.01,0.01,0.01,0.01,1])
R_mat = 1e-5 * np.diag([1,1,1,1,1])

def M_(q,phi):
    x_B = q[0]
    y_B = q[1]
    z_B = q[2]
    theta_Bz = q[3]
    theta_By = q[4]
    theta_Bx = q[5]

    out = SX(
        np.array([
[                                                                                                                             4.688,                                                                                                                             0,                                                                               0, -0.00004*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By))*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0),                                                                               -0.00004*cos(theta_By)*cos(theta_Bz)*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0),                                                                                                                                                             0],
[                                                                                                                                 0,                                                                                                                         4.688,                                                                               0,      0.00004*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz))*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0),                                                                               -0.00004*cos(theta_By)*sin(theta_Bz)*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0),                                                                                                                                                             0],
[                                                                                                                                 0,                                                                                                                             0,                                                                           4.688,                                                    0.00004*cos(theta_By)*sin(theta_Bx)*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0),                                                                                              0.00004*sin(theta_By)*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0),                                                                                                                                                             0],
[-0.00004*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By))*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0), 0.00004*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz))*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0), 0.00004*cos(theta_By)*sin(theta_Bx)*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0),                                                      0.0497336*cos(phi) + 0.0036152*sin(phi) + 0.043917599999999999999579980078379,                                                                                                                                                              0,                                                                                                                                                             0],
[                                                  -0.00004*cos(theta_By)*cos(theta_Bz)*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0),                                              -0.00004*cos(theta_By)*sin(theta_Bz)*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0),               0.00004*sin(theta_By)*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0),                                                                                                                                  0, 0.0015008*cos(phi) - 0.0068904*sin(phi) - 0.0004704*cos(phi)*sin(phi) - 0.00048000000000000000194007136744556*cos(phi)**2 + 0.091682800000000000001520051445825,                                                                                                                                                             0],
[                                                                                                                                 0,                                                                                                                             0,                                                                               0,                                                                                                                                  0,                                                                                                                                                              0, 0.0482328*cos(phi) + 0.0105056*sin(phi) + 0.0004704*cos(phi)*sin(phi) - 0.00048000000000000000194007136744556*sin(phi)**2 + 0.10439320000000000000152005144582]
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
 0.00004*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0)*(omega_Bx**2*sin(theta_Bx)*sin(theta_Bz) + omega_By**2*sin(theta_Bx)*sin(theta_Bz) + omega_Bx**2*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) + omega_By**2*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 1.0*omega_Bx*omega_Bz*cos(theta_By)*cos(theta_Bz) + omega_By*omega_Bz*cos(theta_Bx)*sin(theta_Bz) - 1.0*omega_By*omega_Bz*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)),
-0.00004*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0)*(omega_Bx**2*cos(theta_Bz)*sin(theta_Bx) + omega_By**2*cos(theta_Bz)*sin(theta_Bx) - 1.0*omega_Bx**2*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 1.0*omega_By**2*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) + omega_By*omega_Bz*cos(theta_Bx)*cos(theta_Bz) + omega_Bx*omega_Bz*cos(theta_By)*sin(theta_Bz) + omega_By*omega_Bz*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)),
                                                                                                                                                                                               0.00004*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0)*(cos(theta_Bx)*cos(theta_By)*omega_Bx**2 + omega_Bz*sin(theta_By)*omega_Bx + cos(theta_Bx)*cos(theta_By)*omega_By**2 - 1.0*omega_Bz*cos(theta_By)*sin(theta_Bx)*omega_By),
                                                                                                                                              0.000000000000000000000000043368086899420177360298112034798*omega_By*omega_Bz*(1077566555065743457648640.0*cos(phi) + 401124449882814199889920.0*sin(phi) + 21693371030682432700416.0*cos(phi)*sin(phi) + 22136092888451462028670.0*cos(phi)**2 + 282013823398871625060673.0),
                                                                                                                                              -0.000000000000000000000000043368086899420177360298112034798*omega_Bx*omega_Bz*(158881806706860368068608.0*sin(phi) - 34606091882279118831616.0*cos(phi) + 10846685515341216350208.0*cos(phi)*sin(phi) + 11068046444225731014335.0*cos(phi)**2 + 1383404348435810968666112.0),
                                                                                                                                            -0.000000000000000000000000043368086899420177360298112034798*omega_Bx*omega_By*(1112172646948022576480256.0*cos(phi) + 242242643175953831821312.0*sin(phi) + 10846685515341216350208.0*cos(phi)*sin(phi) + 11068046444225731014335.0*cos(phi)**2 - 1101390525036939343605439.0)        ])
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
                                                                         -45.98928,
-0.0003924*cos(theta_By)*sin(theta_Bx)*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0),
              -0.0003924*sin(theta_By)*(1876.0*cos(phi) - 8613.0*sin(phi) + 662.0),
                                                                                 0            
        ])
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
[- 29.0*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)) - 29.0*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)), 29.0*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)) + 29.0*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)),     -29.0*cos(phi + theta_Bx)*cos(theta_By), 0.29*sin(phi) - 2.03*cos(phi) - 4.06, 4.93*cos(phi) - 2.61*sin(phi),   2.61*cos(phi) + 4.93*sin(phi)],
[  29.0*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)) - 29.0*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)), 29.0*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)) - 29.0*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)), -29.0*cos(phi - 1.0*theta_Bx)*cos(theta_By), 2.03*cos(phi) - 0.29*sin(phi) + 4.06, 2.61*sin(phi) - 4.93*cos(phi),   2.61*cos(phi) + 4.93*sin(phi)],
[  29.0*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)) - 29.0*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)), 29.0*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)) - 29.0*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)), -29.0*cos(phi - 1.0*theta_Bx)*cos(theta_By), 2.03*cos(phi) - 0.29*sin(phi) + 4.06, 4.93*cos(phi) - 2.61*sin(phi), - 2.61*cos(phi) - 4.93*sin(phi)],
[- 29.0*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)) - 29.0*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)), 29.0*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)) + 29.0*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)),     -29.0*cos(phi + theta_Bx)*cos(theta_By), 0.29*sin(phi) - 2.03*cos(phi) - 4.06, 2.61*sin(phi) - 4.93*cos(phi), - 2.61*cos(phi) - 4.93*sin(phi)]        ])
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

    # parameters
    # varphi  = SX.sym("varphi",1)
    # theta_Bz  = SX.sym("theta_Bz",1)
    # omega_Bz  = SX.sym("omega_Bz",1)

    # parameters
    # p = vertcat(varphi,theta_Bz,omega_Bz)
    # p = varphi

    # states
    X = SX.sym("X",13)
    x_B      = X[0]
    y_B      = X[1]
    z_B      = X[2]
    dx_B     = X[3]
    dy_B     = X[4]
    dz_B     = X[5]
    theta_Bx = X[6]
    theta_By = X[7]
    theta_Bz = X[8]
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
    f_expl = vertcat(dx_B,dy_B,dz_B,du[0],du[1],du[2],chid[2],chid[1],chid[0],du[3],du[4],du[5],U[4])

    # xdot
    x_B_dot = SX.sym("x_B_dot")
    y_B_dot = SX.sym("y_B_dot")
    z_B_dot = SX.sym("z_B_dot")
    dx_B_dot = SX.sym("dx_B_dot")
    dy_B_dot = SX.sym("dy_B_dot")
    dz_B_dot = SX.sym("dz_B_dot")
    theta_Bx_dot = SX.sym("theta_Bx_dot")
    theta_By_dot = SX.sym("theta_By_dot")
    theta_Bz_dot = SX.sym("theta_Bz_dot")
    omega_Bx_dot = SX.sym("omega_Bx_dot")
    omega_By_dot = SX.sym("omega_By_dot")
    omega_Bz_dot = SX.sym("omega_Bz_dot")
    varphi_dot = SX.sym("varphi_dot")

    Xdot = vertcat(x_B_dot,y_B_dot,z_B_dot,dx_B_dot,dy_B_dot,dz_B_dot,theta_Bx_dot,theta_By_dot,theta_Bz_dot,omega_Bx_dot,omega_By_dot,omega_Bz_dot,varphi_dot)

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

def plot_robot(
    shooting_nodes,
    u_max,
    U,
    X_true,
    X_est=None,
    Y_measured=None,
    latexify=False,
    plt_show=True,
    X_true_label=None,
):
    """
    Params:
        shooting_nodes: time values of the discretization
        u_max: maximum absolute value of u
        U: arrray with shape (N_sim-1, nu) or (N_sim, nu)
        X_true: arrray with shape (N_sim, nx)
        X_est: arrray with shape (N_sim-N_mhe, nx)
        Y_measured: array with shape (N_sim, ny)
        latexify: latex style plots
    """

    if latexify:
        latexify_plot()

    WITH_ESTIMATION = X_est is not None and Y_measured is not None

    N_sim = X_true.shape[0]
    nx = X_true.shape[1]
    nu = U.shape[1]

    Tf = shooting_nodes[N_sim - 1]
    t = shooting_nodes

    Ts = t[1] - t[0]
    if WITH_ESTIMATION:
        N_mhe = N_sim - X_est.shape[0]
        t_mhe = np.linspace(N_mhe * Ts, Tf, N_sim - N_mhe)

    control_lables = [r"$u_1$", r"$u_2$",r"$u_3$", r"$u_4$", r"v"]

    plt.figure()
    for i in range(nu):
        plt.subplot(nu, 1, i+1)
        (line,) = plt.step(t, np.append([U[0, i]], U[:, i]))
        if X_true_label is not None:
            line.set_label(X_true_label)
        else:
            line.set_color("r")
        # plt.title('closed-loop simulation')
        plt.ylabel(control_lables[i])
        plt.xlabel("$t$")
        if u_max[i] is not None:
            plt.hlines(u_max[i], t[0], t[-1], linestyles="dashed", alpha=0.7)
            plt.hlines(-u_max[i], t[0], t[-1], linestyles="dashed", alpha=0.7)
            plt.ylim([-1.2 * u_max[i], 1.2 * u_max[i]])
        plt.grid()

    states_lables = [r"$x$",r"$y$",r"$z$",r"$\dot x$",r"$\dot y$", r"$\dot z$", r"$\phi$", r"$ \theta $", r"$ \psi $",r"$\omega_x$", r"$\omega_y$",r"$\omega_z$",r"φ"]
    plt.figure(figsize=(10, 6))

    X_true[:,12] = np.rad2deg(X_true[:,12])

    # Plotting states in the first column
    for i, state_index in enumerate([0, 1, 2, 5]):
        plt.subplot(4, 2, i*2 + 1)
        (line,) = plt.plot(t, X_true[:, state_index], label="true")
        if WITH_ESTIMATION:
            plt.plot(t_mhe, X_est[:, state_index], "--", label="estimated")
            plt.plot(t, Y_measured[:, state_index], "x", label="measured")
        plt.ylabel(states_lables[state_index])
        plt.xlabel("$t$")
        plt.grid()

        max_value = np.max(np.abs(X_true[:, state_index]))
        if max_value > 1e-2:
            plt.autoscale(axis='y',tight=True)
        else:
            plt.ylim([-0.1, 0.1])

    # Plotting states in the second column
    for i, state_index in enumerate([6, 7, 8, 12]):
        plt.subplot(4, 2, i*2 + 2)
        (line,) = plt.plot(t, X_true[:, state_index], label="true")
        if WITH_ESTIMATION:
            plt.plot(t_mhe, X_est[:, state_index], "--", label="estimated")
            plt.plot(t, Y_measured[:, state_index], "x", label="measured")
        plt.ylabel(states_lables[state_index])
        plt.xlabel("$t$")
        plt.grid(which='both')
        plt.minorticks_on()

        max_value = np.max(np.abs(X_true[:, state_index]))
        if max_value > 1e-2:
            if state_index==12:
                plt.ylim([0,90])
            else:
                plt.autoscale(axis='y',tight=True)
        else:
            plt.ylim([-0.1, 0.1])
    # plt.subplots_adjust(left=None, bottom=None, right=None, top=None, hspace=0.4)

    if plt_show:
        plt.show()

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
    ocp.constraints.ubu = np.array([+f_max,+f_max,+f_max,+f_max,+v_max])
    ocp.constraints.idxbu = np.array([0,1,2,3,4])

    ocp.constraints.lbx = np.array([0])
    ocp.constraints.ubx = np.array([varphi_max])
    ocp.constraints.idxbx = np.array([12])

    # ocp.constraints.lbx_e = np.array([varphi_e])
    # ocp.constraints.ubx_e = np.array([varphi_e])
    # ocp.constraints.idxbx_e = np.array([12])

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

def closed_loop_simulation():

    # create solvers
    ocp = create_ocp_solver_description()
    acados_ocp_solver = AcadosOcpSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )
    acados_integrator = AcadosSimSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )

    # prepare simulation
    Nsim = 100
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    simX = np.ndarray((Nsim + 1, nx))
    simU = np.ndarray((Nsim, nu))

    xcurrent = X0
    simX[0, :] = xcurrent

    # initialize solver
    for stage in range(N_horizon + 1):
        acados_ocp_solver.set(stage, "x", X_ref)
    for stage in range(N_horizon):
        acados_ocp_solver.set(stage, "u", U_ref)

    # closed loop
    for i in range(Nsim):

        start_time = time.process_time()
        # set initial state constraint
        acados_ocp_solver.set(0, "lbx", xcurrent)
        acados_ocp_solver.set(0, "ubx", xcurrent)

        # update yref
        for j in range(N_horizon):
            yref = np.hstack((X_ref,U_ref))
            acados_ocp_solver.set(j, "yref", yref)
            # acados_ocp_solver.set(j, "p", varphi)   # set parameter
        yref_N = X_ref
        acados_ocp_solver.set(N_horizon, "yref", yref_N)

        # solve ocp
        status = acados_ocp_solver.solve()
        print("* comp time = %5g seconds\n" % (time.process_time() - start_time))


        if status not in [0, 2]:
            acados_ocp_solver.print_statistics()
            plot_robot(
                np.linspace(0, T_horizon / N_horizon * i, i + 1),
                [None,None,None,None,None],
                simU[:i, :],
                simX[: i + 1, :],
            )
            raise Exception(
                f"acados acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            )

        if status == 2:
            print(
                f"acados acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            )
        simU[i, :] = acados_ocp_solver.get(0, "u")

        # simulate system
        acados_integrator.set("x", xcurrent)
        acados_integrator.set("u", simU[i, :])

        status = acados_integrator.solve()
        if status != 0:
            raise Exception(
                f"acados integrator returned status {status} in closed loop instance {i}"
            )

        # update state
        xcurrent = acados_integrator.get("x")
        simX[i + 1, :] = xcurrent

    # plot results
    plot_robot(
        np.linspace(0, T_horizon / N_horizon * Nsim, Nsim + 1), [None, None, None, None, None], simU, simX,latexify=True
    )

if __name__ == "__main__":
    closed_loop_simulation()
