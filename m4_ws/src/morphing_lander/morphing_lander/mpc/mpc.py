# Acados/Casadi
from acados_template import AcadosOcp
import scipy.linalg
import numpy as np
from morphing_lander.mpc.parameters import params_
from morphing_lander.mpc.dynamics import export_robot_model
from morphing_lander.mpc.dynamics import export_discrete_robot_model
from morphing_lander.mpc.dynamics import export_discrete_robot_model_with_int_action
from morphing_lander.mpc.dynamics import export_robot_model_driving
from morphing_lander.mpc.dynamics import export_hybrid_robot_model

# constraint variables
u_max                       = params_.get('u_max')
v_max_absolute              = params_.get('v_max_absolute')

# collocation parameters in flight
N_horizon                   = params_.get('N_horizon')
T_horizon                   = params_.get('T_horizon')

# cost function in flight
Q_mat                       = params_.get('Q_mat')
R_mat                       = params_.get('R_mat')
Q_mat_terminal              = params_.get('Q_mat_terminal')

# collocatoin parameters near ground
N_horizon_near_ground       = params_.get('N_horizon_near_ground')
T_horizon_near_ground       = params_.get('T_horizon_near_ground')

# cost function near ground
Q_mat_near_ground           = params_.get('Q_mat_near_ground')
R_mat_near_ground           = params_.get('R_mat_near_ground')
Q_mat_terminal_near_ground  = params_.get('Q_mat_terminal_near_ground')

# integral term cost
w_int                       = params_.get('w_int')
gamma                       = params_.get('gamma')

# residual learning parameters
use_residual_model          = params_.get('use_residual_model')
l4c_residual_model          = params_.get('l4c_residual_model')
model_ninputs               = params_.get('model_ninputs')

# temporary references (these get overwritten the first time mpc is called)
x_ref                       = np.zeros(12,dtype='float')
u_ref                       = np.zeros(4,dtype='float')
x0                          = np.zeros(12,dtype='float')
varphi0                     = 0.0
grounded0                   = True

# export directory
acados_ocp_path             = params_.get('acados_ocp_path')
acados_ocp_path_near_ground = params_.get('acados_ocp_path_near_ground')
acados_ocp_path_driving     = params_.get('acados_ocp_path_driving')
acados_ocp_path_hybrid      = params_.get('acados_ocp_path_hybrid')

# driving parameters
max_drive_speed             = params_.get('max_drive_speed')
max_turn_speed              = params_.get('max_turn_speed')

# driving cost function
Q_mat_driving               = params_.get('Q_mat_driving')
R_mat_driving               = params_.get('R_mat_driving')
Q_mat_terminal_driving      = params_.get('Q_mat_terminal_driving')

# driving collocation parameters
N_horizon_driving           = params_.get('N_horizon_driving')
T_horizon_driving           = params_.get('T_horizon_driving')

# driving initial references
x_ref_driving               = np.zeros(3,dtype='float')
u_ref_driving               = np.zeros(2,dtype='float')
x0_driving                  = np.zeros(3,dtype='float')

def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # model = export_robot_model()
    model = export_discrete_robot_model(T_horizon/N_horizon)

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

    ocp.cost.yref = np.hstack((x_ref,u_ref))
    ocp.cost.yref_e = x_ref

    # set constraints
    ocp.constraints.lbu = np.array([0,0,0,0])
    ocp.constraints.ubu = np.array([+u_max,+u_max,+u_max,+u_max])
    ocp.constraints.idxbu = np.array([0,1,2,3])
    
    ocp.constraints.x0 = x0

    # parameters
    if use_residual_model:
        l4c_params = l4c_residual_model.get_params(np.zeros((1, model_ninputs)))
        ocp.parameter_values = np.hstack((np.array(varphi0),l4c_params.squeeze()))
    else:
        ocp.parameter_values = np.array([varphi0])
    
    # set options
    ocp.solver_options.qp_solver       = "FULL_CONDENSING_HPIPM"       # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx  = "GAUSS_NEWTON"                # GAUSS_NEWTON, EXACT
    ocp.solver_options.integrator_type = "DISCRETE"                    # ERK      
    ocp.solver_options.nlp_solver_type = "SQP_RTI"                     # SQP_RTI, SQP

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    # export directory
    ocp.code_export_directory = acados_ocp_path

    return ocp

def create_ocp_solver_description_near_ground() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # model = export_robot_model()
    model = export_discrete_robot_model(T_horizon_near_ground/N_horizon_near_ground)

    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # set dimensions
    ocp.dims.N = N_horizon_near_ground

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Q_mat_terminal_near_ground
    ocp.cost.W = scipy.linalg.block_diag(Q_mat_near_ground, R_mat_near_ground)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.hstack((x_ref,u_ref))
    ocp.cost.yref_e = x_ref

    # set constraints
    ocp.constraints.lbu = np.array([0,0,0,0])
    ocp.constraints.ubu = np.array([+u_max,+u_max,+u_max,+u_max])
    ocp.constraints.idxbu = np.array([0,1,2,3])
    
    ocp.constraints.x0 = x0

    # parameters
    if use_residual_model:
        l4c_params = l4c_residual_model.get_params(np.zeros((1, model_ninputs)))
        ocp.parameter_values = np.hstack((np.array(varphi0),l4c_params.squeeze()))
    else:
        ocp.parameter_values = np.array([varphi0])
    
    # set options
    ocp.solver_options.qp_solver       = "FULL_CONDENSING_HPIPM"       # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx  = "GAUSS_NEWTON"                # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "DISCRETE"                    # ERK
    ocp.solver_options.nlp_solver_type = "SQP_RTI"                     # SQP_RTI, SQP

    # set prediction horizon
    ocp.solver_options.tf = T_horizon_near_ground

    # export directory
    ocp.code_export_directory = acados_ocp_path_near_ground

    return ocp

def create_ocp_solver_description_with_int() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # get robot model
    model = export_discrete_robot_model_with_int_action(T_horizon/N_horizon)

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

    # expand the cost matrices to add integral cost
    Q_mat_pad = np.pad(Q_mat,((0,1),(0,1)))
    Q_mat_terminal_pad = np.pad(Q_mat_terminal,((0,1),(0,1)))
    Q_mat_pad[-1,-1] = w_int
    Q_mat_terminal_pad[-1,-1] = gamma * w_int

    ocp.cost.W_e = Q_mat_terminal_pad
    ocp.cost.W = scipy.linalg.block_diag(Q_mat_pad, R_mat)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    # pad the state reference
    x_ref_pad = np.zeros(13)
    x_ref_pad[:-1] = x_ref

    ocp.cost.yref = np.hstack((x_ref_pad,u_ref))
    ocp.cost.yref_e = x_ref_pad

    # set constraints
    ocp.constraints.lbu = np.array([0,0,0,0])
    ocp.constraints.ubu = np.array([+u_max,+u_max,+u_max,+u_max])
    ocp.constraints.idxbu = np.array([0,1,2,3])
    
    # pad initial state
    x0_pad      = np.zeros(13)
    x0_pad[:-1] = x0

    ocp.constraints.x0 = x0_pad

    # parameters
    z_ref0 = 0.0
    ocp.parameter_values = np.array([varphi0,z_ref0])
    
    # set options
    ocp.solver_options.qp_solver       = "FULL_CONDENSING_HPIPM"       # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx  = "GAUSS_NEWTON"                # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "DISCRETE"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"                     # SQP_RTI, SQP

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    # export directory
    ocp.code_export_directory = acados_ocp_path

    return ocp

def create_ocp_solver_description_driving() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # model = export_robot_model()
    model = export_robot_model_driving()

    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # set dimensions
    ocp.dims.N = N_horizon_driving

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Q_mat_terminal_driving
    ocp.cost.W = scipy.linalg.block_diag(Q_mat_driving, R_mat_driving)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.hstack((x_ref_driving,u_ref_driving))
    ocp.cost.yref_e = x_ref_driving

    # set constraints
    ocp.constraints.lbu = np.array([-max_drive_speed,-max_turn_speed])
    ocp.constraints.ubu = np.array([+max_drive_speed,+max_turn_speed])
    ocp.constraints.idxbu = np.array([0,1])
    
    ocp.constraints.x0 = x0_driving
    
    # set options
    ocp.solver_options.qp_solver       = "FULL_CONDENSING_HPIPM"       # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx  = "GAUSS_NEWTON"                # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "ERK"                         # IRK
    ocp.solver_options.nlp_solver_type = "SQP_RTI"                     # SQP_RTI, SQP

    # set prediction horizon
    ocp.solver_options.tf = T_horizon_driving

    # export directory
    ocp.code_export_directory = acados_ocp_path_driving

    return ocp

def create_ocp_solver_description_hybrid() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    model = export_hybrid_robot_model()

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

    # pad cost matrices
    R_mat_pad = np.pad(R_mat,((0,2),(0,2)))
    R_mat_pad[-2:,-2:] = R_mat_driving

    ocp.cost.W_e = Q_mat_terminal
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat_pad)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    # pad the input reference
    u_ref_pad = np.zeros(6)

    ocp.cost.yref = np.hstack((x_ref,u_ref_pad))
    ocp.cost.yref_e = x_ref

    # set constraints
    ocp.constraints.lbu = np.array([0,0,0,0,-max_drive_speed,-max_turn_speed])
    ocp.constraints.ubu = np.array([+u_max,+u_max,+u_max,+u_max,+max_drive_speed,+max_turn_speed])
    ocp.constraints.idxbu = np.array([0,1,2,3,4,5])
    
    ocp.constraints.x0 = x0

    # parameters
    ocp.parameter_values = np.array([varphi0,grounded0])
    
    # set options
    ocp.solver_options.qp_solver       = "FULL_CONDENSING_HPIPM"       # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx  = "GAUSS_NEWTON"                # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "ERK"                         # IRK
    ocp.solver_options.nlp_solver_type = "SQP_RTI"                     # SQP_RTI, SQP

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    # export directory
    ocp.code_export_directory = acados_ocp_path_hybrid

    return ocp
