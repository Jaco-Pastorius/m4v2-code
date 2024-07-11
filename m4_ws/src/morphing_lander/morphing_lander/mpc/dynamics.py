from casadi import SX, vertcat, sin, cos, inv, Function, jacobian, MX, pi, fabs, if_else
import numpy as np
from acados_template import AcadosModel
from morphing_lander.mpc.utils import theta_fit
from morphing_lander.mpc.parameters import params_

# check whether to use residual model
use_residual_model  = params_['use_residual_model']
l4c_residual_model  = params_['l4c_residual_model']

# get learning parameters
model_states_in_idx  = params_.get('model_states_in_idx')
model_inputs_in_idx  = params_.get('model_inputs_in_idx')
model_phi_in         = params_.get('model_phi_in')
model_dt_in          = params_.get('model_dt_in')
model_states_out_idx = params_.get('model_states_out_idx')
model_ninputs        = params_.get('model_ninputs')
model_noutputs       = params_.get('model_noutputs')

# integrator
integral_gain        = params_.get('integral_gain')

# get robot model parameters 
gravity              = params_.get('g')                                 
kT                   = params_.get('kT')
kM                   = params_.get('kM')
m_base               = params_.get('m_base')
m_arm                = params_.get('m_arm')
m_rotor              = params_.get('m_rotor')
m                    = params_.get('m')
I_base_xx            = params_.get('I_base_xx')
I_base_yy            = params_.get('I_base_yy')
I_base_zz            = params_.get('I_base_zz')
I_base_xy            = params_.get('I_base_xy')
I_base_xz            = params_.get('I_base_xz')
I_base_yz            = params_.get('I_base_yz')
I_arm_xx             = params_.get('I_arm_xx')
I_arm_yy             = params_.get('I_arm_yy')
I_arm_zz             = params_.get('I_arm_zz')
I_arm_xy             = params_.get('I_arm_xy')
I_arm_xz             = params_.get('I_arm_xz')
I_arm_yz             = params_.get('I_arm_yz')
I_rotor_xx           = params_.get('I_rotor_xx')
I_rotor_yy           = params_.get('I_rotor_yy')
I_rotor_zz           = params_.get('I_rotor_zz')
r_BA_right_x         = params_.get('r_BA_right_x')
r_BA_right_y         = params_.get('r_BA_right_y')
r_BA_right_z         = params_.get('r_BA_right_z')
r_AG_right_x         = params_.get('r_AG_right_x')
r_AG_right_y         = params_.get('r_AG_right_y')
r_AG_right_z         = params_.get('r_AG_right_z')
r_AR1_x              = params_.get('r_AR1_x')
r_AR1_y              = params_.get('r_AR1_y')
r_AR1_z              = params_.get('r_AR1_z')

# individual terms of robot dynamics equation
def M_func():
    x   = SX.sym('x',12)
    phi = SX.sym('phi',1)

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
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   2*m_arm + m_base + 4*m_rotor,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               0,                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                                                       2*(cos(theta_Bx)*sin(theta_Bz) - cos(theta_Bz)*sin(theta_Bx)*sin(theta_By))*(m_arm*r_BA_right_z + 2*m_rotor*r_BA_right_z + m_arm*r_AG_right_z*cos(phi) + 2*m_rotor*r_AR1_z*cos(phi) + m_arm*r_AG_right_y*sin(phi) + 2*m_rotor*r_AR1_y*sin(phi)), 2*m_arm*r_BA_right_z*cos(theta_By)*cos(theta_Bz) - 2*m_arm*r_BA_right_x*sin(theta_Bx)*sin(theta_Bz) - 4*m_rotor*r_BA_right_x*sin(theta_Bx)*sin(theta_Bz) - 2*m_arm*r_AG_right_x*sin(theta_Bx)*sin(theta_Bz) + 4*m_rotor*r_BA_right_z*cos(theta_By)*cos(theta_Bz) + 2*m_arm*r_AG_right_z*cos(phi)*cos(theta_By)*cos(theta_Bz) + 4*m_rotor*r_AR1_z*cos(phi)*cos(theta_By)*cos(theta_Bz) + 2*m_arm*r_AG_right_y*cos(theta_By)*cos(theta_Bz)*sin(phi) + 4*m_rotor*r_AR1_y*cos(theta_By)*cos(theta_Bz)*sin(phi) - 2*m_arm*r_AG_right_x*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 2*m_arm*r_BA_right_x*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 4*m_rotor*r_BA_right_x*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  -2*(cos(theta_Bx)*sin(theta_Bz) - cos(theta_Bz)*sin(theta_Bx)*sin(theta_By))*(m_arm*r_AG_right_x + m_arm*r_BA_right_x + 2*m_rotor*r_BA_right_x)],
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    2*m_arm + m_base + 4*m_rotor,                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                                                      -2*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz))*(m_arm*r_BA_right_z + 2*m_rotor*r_BA_right_z + m_arm*r_AG_right_z*cos(phi) + 2*m_rotor*r_AR1_z*cos(phi) + m_arm*r_AG_right_y*sin(phi) + 2*m_rotor*r_AR1_y*sin(phi)), 2*m_arm*r_AG_right_x*cos(theta_Bz)*sin(theta_Bx) + 2*m_arm*r_BA_right_x*cos(theta_Bz)*sin(theta_Bx) + 2*m_arm*r_BA_right_z*cos(theta_By)*sin(theta_Bz) + 4*m_rotor*r_BA_right_x*cos(theta_Bz)*sin(theta_Bx) + 4*m_rotor*r_BA_right_z*cos(theta_By)*sin(theta_Bz) + 2*m_arm*r_AG_right_z*cos(phi)*cos(theta_By)*sin(theta_Bz) + 4*m_rotor*r_AR1_z*cos(phi)*cos(theta_By)*sin(theta_Bz) + 2*m_arm*r_AG_right_y*cos(theta_By)*sin(phi)*sin(theta_Bz) + 4*m_rotor*r_AR1_y*cos(theta_By)*sin(phi)*sin(theta_Bz) - 2*m_arm*r_AG_right_x*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 2*m_arm*r_BA_right_x*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 4*m_rotor*r_BA_right_x*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   2*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz))*(m_arm*r_AG_right_x + m_arm*r_BA_right_x + 2*m_rotor*r_BA_right_x)],
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               0,                                                                                                                                                                                                                                                                                                                                                                                             2*m_arm + m_base + 4*m_rotor,                                                                                                                                                                                                                                                                                                                                                    -2*cos(theta_By)*sin(theta_Bx)*(m_arm*r_BA_right_z + 2*m_rotor*r_BA_right_z + m_arm*r_AG_right_z*cos(phi) + 2*m_rotor*r_AR1_z*cos(phi) + m_arm*r_AG_right_y*sin(phi) + 2*m_rotor*r_AR1_y*sin(phi)),                                                                                                                                                                                                                                                                                        - 2*m_arm*r_BA_right_z*sin(theta_By) - 4*m_rotor*r_BA_right_z*sin(theta_By) - 2*m_arm*r_AG_right_y*sin(phi)*sin(theta_By) - 4*m_rotor*r_AR1_y*sin(phi)*sin(theta_By) - 2*m_arm*r_AG_right_x*cos(theta_Bx)*cos(theta_By) - 2*m_arm*r_BA_right_x*cos(theta_Bx)*cos(theta_By) - 4*m_rotor*r_BA_right_x*cos(theta_Bx)*cos(theta_By) - 2*m_arm*r_AG_right_z*cos(phi)*sin(theta_By) - 4*m_rotor*r_AR1_z*cos(phi)*sin(theta_By),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 2*cos(theta_By)*sin(theta_Bx)*(m_arm*r_AG_right_x + m_arm*r_BA_right_x + 2*m_rotor*r_BA_right_x)],
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                2*(cos(theta_Bx)*sin(theta_Bz) - cos(theta_Bz)*sin(theta_Bx)*sin(theta_By))*(m_arm*r_BA_right_z + 2*m_rotor*r_BA_right_z + m_arm*r_AG_right_z*cos(phi) + 2*m_rotor*r_AR1_z*cos(phi) + m_arm*r_AG_right_y*sin(phi) + 2*m_rotor*r_AR1_y*sin(phi)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                -2*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz))*(m_arm*r_BA_right_z + 2*m_rotor*r_BA_right_z + m_arm*r_AG_right_z*cos(phi) + 2*m_rotor*r_AR1_z*cos(phi) + m_arm*r_AG_right_y*sin(phi) + 2*m_rotor*r_AR1_y*sin(phi)),                                                                                                                                                                                                                       -2*cos(theta_By)*sin(theta_Bx)*(m_arm*r_BA_right_z + 2*m_rotor*r_BA_right_z + m_arm*r_AG_right_z*cos(phi) + 2*m_rotor*r_AR1_z*cos(phi) + m_arm*r_AG_right_y*sin(phi) + 2*m_rotor*r_AR1_y*sin(phi)), 2*I_arm_xx + I_base_xx + 4*I_rotor_xx + 2*m_arm*r_BA_right_y**2 + 2*m_arm*r_BA_right_z**2 + 4*m_rotor*r_AR1_y**2 + 4*m_rotor*r_AR1_z**2 + 4*m_rotor*r_BA_right_y**2 + 4*m_rotor*r_BA_right_z**2 + 4*m_arm*r_AG_right_y*r_BA_right_y*cos(phi) + 4*m_arm*r_AG_right_z*r_BA_right_z*cos(phi) + 8*m_rotor*r_AR1_y*r_BA_right_y*cos(phi) + 8*m_rotor*r_AR1_z*r_BA_right_z*cos(phi) + 4*m_arm*r_AG_right_y*r_BA_right_z*sin(phi) - 4*m_arm*r_AG_right_z*r_BA_right_y*sin(phi) + 8*m_rotor*r_AR1_y*r_BA_right_z*sin(phi) - 8*m_rotor*r_AR1_z*r_BA_right_y*sin(phi),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           I_base_xy + 2*I_arm_xy*cos(phi) + 2*I_arm_xz*sin(phi),                                                                                                                                                                                                                                                                                                                                            I_base_xz - 2*m_arm*r_AG_right_x*r_BA_right_z - 2*m_arm*r_BA_right_x*r_BA_right_z - 4*m_rotor*r_BA_right_x*r_BA_right_z - 2*m_arm*r_AG_right_z*r_BA_right_x*cos(phi) - 4*m_rotor*r_AR1_z*r_BA_right_x*cos(phi) - 2*m_arm*r_AG_right_y*r_BA_right_x*sin(phi) - 4*m_rotor*r_AR1_y*r_BA_right_x*sin(phi)],
[2*m_arm*r_BA_right_z*cos(theta_By)*cos(theta_Bz) - 2*m_arm*r_BA_right_x*sin(theta_Bx)*sin(theta_Bz) - 4*m_rotor*r_BA_right_x*sin(theta_Bx)*sin(theta_Bz) - 2*m_arm*r_AG_right_x*sin(theta_Bx)*sin(theta_Bz) + 4*m_rotor*r_BA_right_z*cos(theta_By)*cos(theta_Bz) + 2*m_arm*r_AG_right_z*cos(phi)*cos(theta_By)*cos(theta_Bz) + 4*m_rotor*r_AR1_z*cos(phi)*cos(theta_By)*cos(theta_Bz) + 2*m_arm*r_AG_right_y*cos(theta_By)*cos(theta_Bz)*sin(phi) + 4*m_rotor*r_AR1_y*cos(theta_By)*cos(theta_Bz)*sin(phi) - 2*m_arm*r_AG_right_x*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 2*m_arm*r_BA_right_x*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 4*m_rotor*r_BA_right_x*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By), 2*m_arm*r_AG_right_x*cos(theta_Bz)*sin(theta_Bx) + 2*m_arm*r_BA_right_x*cos(theta_Bz)*sin(theta_Bx) + 2*m_arm*r_BA_right_z*cos(theta_By)*sin(theta_Bz) + 4*m_rotor*r_BA_right_x*cos(theta_Bz)*sin(theta_Bx) + 4*m_rotor*r_BA_right_z*cos(theta_By)*sin(theta_Bz) + 2*m_arm*r_AG_right_z*cos(phi)*cos(theta_By)*sin(theta_Bz) + 4*m_rotor*r_AR1_z*cos(phi)*cos(theta_By)*sin(theta_Bz) + 2*m_arm*r_AG_right_y*cos(theta_By)*sin(phi)*sin(theta_Bz) + 4*m_rotor*r_AR1_y*cos(theta_By)*sin(phi)*sin(theta_Bz) - 2*m_arm*r_AG_right_x*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 2*m_arm*r_BA_right_x*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 4*m_rotor*r_BA_right_x*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz), - 2*m_arm*r_BA_right_z*sin(theta_By) - 4*m_rotor*r_BA_right_z*sin(theta_By) - 2*m_arm*r_AG_right_y*sin(phi)*sin(theta_By) - 4*m_rotor*r_AR1_y*sin(phi)*sin(theta_By) - 2*m_arm*r_AG_right_x*cos(theta_Bx)*cos(theta_By) - 2*m_arm*r_BA_right_x*cos(theta_Bx)*cos(theta_By) - 4*m_rotor*r_BA_right_x*cos(theta_Bx)*cos(theta_By) - 2*m_arm*r_AG_right_z*cos(phi)*sin(theta_By) - 4*m_rotor*r_AR1_z*cos(phi)*sin(theta_By),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 I_base_xy + 2*I_arm_xy*cos(phi) + 2*I_arm_xz*sin(phi),                                                                2*I_arm_zz + I_base_yy + 4*I_rotor_zz + 2*m_arm*r_BA_right_x**2 + 2*m_arm*r_BA_right_z**2 + 4*m_rotor*r_AR1_x**2 + 4*m_rotor*r_AR1_y**2 + 4*m_rotor*r_BA_right_x**2 + 4*m_rotor*r_BA_right_z**2 + 2*I_arm_yy*cos(phi)**2 - 2*I_arm_zz*cos(phi)**2 + 4*I_rotor_yy*cos(phi)**2 - 4*I_rotor_zz*cos(phi)**2 + 2*I_arm_yz*sin(2*phi) + 4*m_arm*r_AG_right_x*r_BA_right_x - 4*m_rotor*r_AR1_y**2*cos(phi)**2 + 4*m_rotor*r_AR1_z**2*cos(phi)**2 + 4*m_arm*r_AG_right_z*r_BA_right_z*cos(phi) + 8*m_rotor*r_AR1_z*r_BA_right_z*cos(phi) + 4*m_arm*r_AG_right_y*r_BA_right_z*sin(phi) + 8*m_rotor*r_AR1_y*r_BA_right_z*sin(phi) + 4*m_rotor*r_AR1_y*r_AR1_z*sin(2*phi),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        I_base_yz],
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                -2*(cos(theta_Bx)*sin(theta_Bz) - cos(theta_Bz)*sin(theta_Bx)*sin(theta_By))*(m_arm*r_AG_right_x + m_arm*r_BA_right_x + 2*m_rotor*r_BA_right_x),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  2*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz))*(m_arm*r_AG_right_x + m_arm*r_BA_right_x + 2*m_rotor*r_BA_right_x),                                                                                                                                                                                                                                                                                                                         2*cos(theta_By)*sin(theta_Bx)*(m_arm*r_AG_right_x + m_arm*r_BA_right_x + 2*m_rotor*r_BA_right_x),                                                                                                                                                                                                                                                 I_base_xz - 2*m_arm*r_AG_right_x*r_BA_right_z - 2*m_arm*r_BA_right_x*r_BA_right_z - 4*m_rotor*r_BA_right_x*r_BA_right_z - 2*m_arm*r_AG_right_z*r_BA_right_x*cos(phi) - 4*m_rotor*r_AR1_z*r_BA_right_x*cos(phi) - 2*m_arm*r_AG_right_y*r_BA_right_x*sin(phi) - 4*m_rotor*r_AR1_y*r_BA_right_x*sin(phi),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       I_base_yz, 2*I_arm_yy + I_base_zz + 4*I_rotor_yy + 2*m_arm*r_BA_right_x**2 + 2*m_arm*r_BA_right_y**2 + 4*m_rotor*r_AR1_x**2 + 4*m_rotor*r_AR1_z**2 + 4*m_rotor*r_BA_right_x**2 + 4*m_rotor*r_BA_right_y**2 - 2*I_arm_yy*cos(phi)**2 + 2*I_arm_zz*cos(phi)**2 - 4*I_rotor_yy*cos(phi)**2 + 4*I_rotor_zz*cos(phi)**2 - 2*I_arm_yz*sin(2*phi) + 4*m_arm*r_AG_right_x*r_BA_right_x + 4*m_rotor*r_AR1_y**2*cos(phi)**2 - 4*m_rotor*r_AR1_z**2*cos(phi)**2 + 4*m_arm*r_AG_right_y*r_BA_right_y*cos(phi) + 8*m_rotor*r_AR1_y*r_BA_right_y*cos(phi) - 4*m_arm*r_AG_right_z*r_BA_right_y*sin(phi) - 8*m_rotor*r_AR1_z*r_BA_right_y*sin(phi) - 4*m_rotor*r_AR1_y*r_AR1_z*sin(2*phi)]        
        ])
    )
    return Function("M_func",[x,phi],[out])

def b_func():
    x   = SX.sym('x',12)
    phi = SX.sym('phi',1)
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
2*m_arm*omega_Bx*omega_Bz*r_BA_right_z*cos(theta_By)*cos(theta_Bz) - 2*m_arm*omega_Bz**2*r_AG_right_x*cos(theta_By)*cos(theta_Bz) - 2*m_arm*omega_By**2*r_BA_right_x*cos(theta_By)*cos(theta_Bz) - 2*m_arm*omega_Bz**2*r_BA_right_x*cos(theta_By)*cos(theta_Bz) - 4*m_rotor*omega_By**2*r_BA_right_x*cos(theta_By)*cos(theta_Bz) - 4*m_rotor*omega_Bz**2*r_BA_right_x*cos(theta_By)*cos(theta_Bz) - 2*m_arm*omega_Bx**2*r_BA_right_z*sin(theta_Bx)*sin(theta_Bz) - 2*m_arm*omega_By**2*r_BA_right_z*sin(theta_Bx)*sin(theta_Bz) - 4*m_rotor*omega_Bx**2*r_BA_right_z*sin(theta_Bx)*sin(theta_Bz) - 4*m_rotor*omega_By**2*r_BA_right_z*sin(theta_Bx)*sin(theta_Bz) - 2*m_arm*omega_Bx**2*r_BA_right_z*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 2*m_arm*omega_By**2*r_BA_right_z*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 4*m_rotor*omega_Bx**2*r_BA_right_z*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 4*m_rotor*omega_By**2*r_BA_right_z*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 2*m_arm*omega_Bx**2*r_AG_right_z*cos(phi)*sin(theta_Bx)*sin(theta_Bz) - 2*m_arm*omega_By**2*r_AG_right_z*cos(phi)*sin(theta_Bx)*sin(theta_Bz) - 4*m_rotor*omega_Bx**2*r_AR1_z*cos(phi)*sin(theta_Bx)*sin(theta_Bz) - 4*m_rotor*omega_By**2*r_AR1_z*cos(phi)*sin(theta_Bx)*sin(theta_Bz) - 2*m_arm*omega_Bx**2*r_AG_right_y*sin(phi)*sin(theta_Bx)*sin(theta_Bz) - 2*m_arm*omega_By**2*r_AG_right_y*sin(phi)*sin(theta_Bx)*sin(theta_Bz) - 4*m_rotor*omega_Bx**2*r_AR1_y*sin(phi)*sin(theta_Bx)*sin(theta_Bz) - 4*m_rotor*omega_By**2*r_AR1_y*sin(phi)*sin(theta_Bx)*sin(theta_Bz) - 2*m_arm*omega_By**2*r_AG_right_x*cos(theta_By)*cos(theta_Bz) + 4*m_rotor*omega_Bx*omega_Bz*r_BA_right_z*cos(theta_By)*cos(theta_Bz) - 2*m_arm*omega_Bx*omega_By*r_AG_right_x*cos(theta_Bx)*sin(theta_Bz) - 2*m_arm*omega_Bx*omega_By*r_BA_right_x*cos(theta_Bx)*sin(theta_Bz) - 2*m_arm*omega_By*omega_Bz*r_BA_right_z*cos(theta_Bx)*sin(theta_Bz) - 4*m_rotor*omega_Bx*omega_By*r_BA_right_x*cos(theta_Bx)*sin(theta_Bz) - 4*m_rotor*omega_By*omega_Bz*r_BA_right_z*cos(theta_Bx)*sin(theta_Bz) + 2*m_arm*omega_Bx*omega_Bz*r_AG_right_x*sin(theta_Bx)*sin(theta_Bz) + 2*m_arm*omega_Bx*omega_Bz*r_BA_right_x*sin(theta_Bx)*sin(theta_Bz) + 4*m_rotor*omega_Bx*omega_Bz*r_BA_right_x*sin(theta_Bx)*sin(theta_Bz) - 2*m_arm*omega_Bx**2*r_AG_right_z*cos(phi)*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 2*m_arm*omega_By**2*r_AG_right_z*cos(phi)*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 4*m_rotor*omega_Bx**2*r_AR1_z*cos(phi)*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 4*m_rotor*omega_By**2*r_AR1_z*cos(phi)*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 2*m_arm*omega_Bx**2*r_AG_right_y*cos(theta_Bx)*cos(theta_Bz)*sin(phi)*sin(theta_By) - 2*m_arm*omega_By**2*r_AG_right_y*cos(theta_Bx)*cos(theta_Bz)*sin(phi)*sin(theta_By) - 4*m_rotor*omega_Bx**2*r_AR1_y*cos(theta_Bx)*cos(theta_Bz)*sin(phi)*sin(theta_By) - 4*m_rotor*omega_By**2*r_AR1_y*cos(theta_Bx)*cos(theta_Bz)*sin(phi)*sin(theta_By) + 2*m_arm*omega_Bx*omega_Bz*r_AG_right_z*cos(phi)*cos(theta_By)*cos(theta_Bz) + 4*m_rotor*omega_Bx*omega_Bz*r_AR1_z*cos(phi)*cos(theta_By)*cos(theta_Bz) + 2*m_arm*omega_Bx*omega_Bz*r_AG_right_y*cos(theta_By)*cos(theta_Bz)*sin(phi) - 2*m_arm*omega_By*omega_Bz*r_AG_right_z*cos(phi)*cos(theta_Bx)*sin(theta_Bz) + 4*m_rotor*omega_Bx*omega_Bz*r_AR1_y*cos(theta_By)*cos(theta_Bz)*sin(phi) - 4*m_rotor*omega_By*omega_Bz*r_AR1_z*cos(phi)*cos(theta_Bx)*sin(theta_Bz) + 2*m_arm*omega_Bx*omega_Bz*r_AG_right_x*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) + 2*m_arm*omega_Bx*omega_Bz*r_BA_right_x*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) + 4*m_rotor*omega_Bx*omega_Bz*r_BA_right_x*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 2*m_arm*omega_By*omega_Bz*r_AG_right_y*cos(theta_Bx)*sin(phi)*sin(theta_Bz) - 4*m_rotor*omega_By*omega_Bz*r_AR1_y*cos(theta_Bx)*sin(phi)*sin(theta_Bz) + 2*m_arm*omega_Bx*omega_By*r_AG_right_x*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 2*m_arm*omega_Bx*omega_By*r_BA_right_x*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 2*m_arm*omega_By*omega_Bz*r_BA_right_z*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 4*m_rotor*omega_Bx*omega_By*r_BA_right_x*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 4*m_rotor*omega_By*omega_Bz*r_BA_right_z*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 2*m_arm*omega_By*omega_Bz*r_AG_right_z*cos(phi)*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 4*m_rotor*omega_By*omega_Bz*r_AR1_z*cos(phi)*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 2*m_arm*omega_By*omega_Bz*r_AG_right_y*cos(theta_Bz)*sin(phi)*sin(theta_Bx)*sin(theta_By) + 4*m_rotor*omega_By*omega_Bz*r_AR1_y*cos(theta_Bz)*sin(phi)*sin(theta_Bx)*sin(theta_By),
2*m_arm*omega_Bx**2*r_BA_right_z*cos(theta_Bz)*sin(theta_Bx) - 2*m_arm*omega_Bz**2*r_AG_right_x*cos(theta_By)*sin(theta_Bz) - 2*m_arm*omega_By**2*r_AG_right_x*cos(theta_By)*sin(theta_Bz) - 2*m_arm*omega_By**2*r_BA_right_x*cos(theta_By)*sin(theta_Bz) + 2*m_arm*omega_By**2*r_BA_right_z*cos(theta_Bz)*sin(theta_Bx) - 2*m_arm*omega_Bz**2*r_BA_right_x*cos(theta_By)*sin(theta_Bz) + 4*m_rotor*omega_Bx**2*r_BA_right_z*cos(theta_Bz)*sin(theta_Bx) - 4*m_rotor*omega_By**2*r_BA_right_x*cos(theta_By)*sin(theta_Bz) + 4*m_rotor*omega_By**2*r_BA_right_z*cos(theta_Bz)*sin(theta_Bx) - 4*m_rotor*omega_Bz**2*r_BA_right_x*cos(theta_By)*sin(theta_Bz) + 2*m_arm*omega_Bx**2*r_AG_right_z*cos(phi)*cos(theta_Bz)*sin(theta_Bx) + 2*m_arm*omega_By**2*r_AG_right_z*cos(phi)*cos(theta_Bz)*sin(theta_Bx) + 4*m_rotor*omega_Bx**2*r_AR1_z*cos(phi)*cos(theta_Bz)*sin(theta_Bx) + 4*m_rotor*omega_By**2*r_AR1_z*cos(phi)*cos(theta_Bz)*sin(theta_Bx) + 2*m_arm*omega_Bx**2*r_AG_right_y*cos(theta_Bz)*sin(phi)*sin(theta_Bx) + 2*m_arm*omega_By**2*r_AG_right_y*cos(theta_Bz)*sin(phi)*sin(theta_Bx) + 4*m_rotor*omega_Bx**2*r_AR1_y*cos(theta_Bz)*sin(phi)*sin(theta_Bx) + 4*m_rotor*omega_By**2*r_AR1_y*cos(theta_Bz)*sin(phi)*sin(theta_Bx) - 2*m_arm*omega_Bx**2*r_BA_right_z*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 2*m_arm*omega_By**2*r_BA_right_z*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 4*m_rotor*omega_Bx**2*r_BA_right_z*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 4*m_rotor*omega_By**2*r_BA_right_z*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 2*m_arm*omega_Bx*omega_By*r_AG_right_x*cos(theta_Bx)*cos(theta_Bz) + 2*m_arm*omega_Bx*omega_By*r_BA_right_x*cos(theta_Bx)*cos(theta_Bz) + 2*m_arm*omega_By*omega_Bz*r_BA_right_z*cos(theta_Bx)*cos(theta_Bz) + 4*m_rotor*omega_Bx*omega_By*r_BA_right_x*cos(theta_Bx)*cos(theta_Bz) + 4*m_rotor*omega_By*omega_Bz*r_BA_right_z*cos(theta_Bx)*cos(theta_Bz) - 2*m_arm*omega_Bx*omega_Bz*r_AG_right_x*cos(theta_Bz)*sin(theta_Bx) - 2*m_arm*omega_Bx*omega_Bz*r_BA_right_x*cos(theta_Bz)*sin(theta_Bx) + 2*m_arm*omega_Bx*omega_Bz*r_BA_right_z*cos(theta_By)*sin(theta_Bz) - 4*m_rotor*omega_Bx*omega_Bz*r_BA_right_x*cos(theta_Bz)*sin(theta_Bx) + 4*m_rotor*omega_Bx*omega_Bz*r_BA_right_z*cos(theta_By)*sin(theta_Bz) - 2*m_arm*omega_Bx**2*r_AG_right_z*cos(phi)*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 2*m_arm*omega_By**2*r_AG_right_z*cos(phi)*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 4*m_rotor*omega_Bx**2*r_AR1_z*cos(phi)*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 4*m_rotor*omega_By**2*r_AR1_z*cos(phi)*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 2*m_arm*omega_Bx**2*r_AG_right_y*cos(theta_Bx)*sin(phi)*sin(theta_By)*sin(theta_Bz) - 2*m_arm*omega_By**2*r_AG_right_y*cos(theta_Bx)*sin(phi)*sin(theta_By)*sin(theta_Bz) - 4*m_rotor*omega_Bx**2*r_AR1_y*cos(theta_Bx)*sin(phi)*sin(theta_By)*sin(theta_Bz) - 4*m_rotor*omega_By**2*r_AR1_y*cos(theta_Bx)*sin(phi)*sin(theta_By)*sin(theta_Bz) + 2*m_arm*omega_By*omega_Bz*r_AG_right_z*cos(phi)*cos(theta_Bx)*cos(theta_Bz) + 4*m_rotor*omega_By*omega_Bz*r_AR1_z*cos(phi)*cos(theta_Bx)*cos(theta_Bz) + 2*m_arm*omega_By*omega_Bz*r_AG_right_y*cos(theta_Bx)*cos(theta_Bz)*sin(phi) + 2*m_arm*omega_Bx*omega_Bz*r_AG_right_z*cos(phi)*cos(theta_By)*sin(theta_Bz) + 4*m_rotor*omega_By*omega_Bz*r_AR1_y*cos(theta_Bx)*cos(theta_Bz)*sin(phi) + 4*m_rotor*omega_Bx*omega_Bz*r_AR1_z*cos(phi)*cos(theta_By)*sin(theta_Bz) + 2*m_arm*omega_Bx*omega_Bz*r_AG_right_y*cos(theta_By)*sin(phi)*sin(theta_Bz) + 4*m_rotor*omega_Bx*omega_Bz*r_AR1_y*cos(theta_By)*sin(phi)*sin(theta_Bz) + 2*m_arm*omega_Bx*omega_Bz*r_AG_right_x*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 2*m_arm*omega_Bx*omega_Bz*r_BA_right_x*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 4*m_rotor*omega_Bx*omega_Bz*r_BA_right_x*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 2*m_arm*omega_Bx*omega_By*r_AG_right_x*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 2*m_arm*omega_Bx*omega_By*r_BA_right_x*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 2*m_arm*omega_By*omega_Bz*r_BA_right_z*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 4*m_rotor*omega_Bx*omega_By*r_BA_right_x*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 4*m_rotor*omega_By*omega_Bz*r_BA_right_z*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 2*m_arm*omega_By*omega_Bz*r_AG_right_z*cos(phi)*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 4*m_rotor*omega_By*omega_Bz*r_AR1_z*cos(phi)*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 2*m_arm*omega_By*omega_Bz*r_AG_right_y*sin(phi)*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 4*m_rotor*omega_By*omega_Bz*r_AR1_y*sin(phi)*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                2*m_arm*omega_By**2*r_AG_right_x*sin(theta_By) + 2*m_arm*omega_Bz**2*r_AG_right_x*sin(theta_By) + 2*m_arm*omega_By**2*r_BA_right_x*sin(theta_By) + 2*m_arm*omega_Bz**2*r_BA_right_x*sin(theta_By) + 4*m_rotor*omega_By**2*r_BA_right_x*sin(theta_By) + 4*m_rotor*omega_Bz**2*r_BA_right_x*sin(theta_By) - 2*m_arm*omega_Bx*omega_Bz*r_BA_right_z*sin(theta_By) - 4*m_rotor*omega_Bx*omega_Bz*r_BA_right_z*sin(theta_By) - 2*m_arm*omega_Bx**2*r_BA_right_z*cos(theta_Bx)*cos(theta_By) - 2*m_arm*omega_By**2*r_BA_right_z*cos(theta_Bx)*cos(theta_By) - 4*m_rotor*omega_Bx**2*r_BA_right_z*cos(theta_Bx)*cos(theta_By) - 4*m_rotor*omega_By**2*r_BA_right_z*cos(theta_Bx)*cos(theta_By) - 2*m_arm*omega_Bx**2*r_AG_right_z*cos(phi)*cos(theta_Bx)*cos(theta_By) - 2*m_arm*omega_By**2*r_AG_right_z*cos(phi)*cos(theta_Bx)*cos(theta_By) - 4*m_rotor*omega_Bx**2*r_AR1_z*cos(phi)*cos(theta_Bx)*cos(theta_By) - 4*m_rotor*omega_By**2*r_AR1_z*cos(phi)*cos(theta_Bx)*cos(theta_By) - 2*m_arm*omega_Bx**2*r_AG_right_y*cos(theta_Bx)*cos(theta_By)*sin(phi) - 2*m_arm*omega_By**2*r_AG_right_y*cos(theta_Bx)*cos(theta_By)*sin(phi) - 4*m_rotor*omega_Bx**2*r_AR1_y*cos(theta_Bx)*cos(theta_By)*sin(phi) - 4*m_rotor*omega_By**2*r_AR1_y*cos(theta_Bx)*cos(theta_By)*sin(phi) + 2*m_arm*omega_Bx*omega_Bz*r_AG_right_x*cos(theta_Bx)*cos(theta_By) + 2*m_arm*omega_Bx*omega_Bz*r_BA_right_x*cos(theta_Bx)*cos(theta_By) + 4*m_rotor*omega_Bx*omega_Bz*r_BA_right_x*cos(theta_Bx)*cos(theta_By) - 2*m_arm*omega_Bx*omega_Bz*r_AG_right_z*cos(phi)*sin(theta_By) - 4*m_rotor*omega_Bx*omega_Bz*r_AR1_z*cos(phi)*sin(theta_By) + 2*m_arm*omega_Bx*omega_By*r_AG_right_x*cos(theta_By)*sin(theta_Bx) + 2*m_arm*omega_Bx*omega_By*r_BA_right_x*cos(theta_By)*sin(theta_Bx) + 2*m_arm*omega_By*omega_Bz*r_BA_right_z*cos(theta_By)*sin(theta_Bx) + 4*m_rotor*omega_Bx*omega_By*r_BA_right_x*cos(theta_By)*sin(theta_Bx) + 4*m_rotor*omega_By*omega_Bz*r_BA_right_z*cos(theta_By)*sin(theta_Bx) - 2*m_arm*omega_Bx*omega_Bz*r_AG_right_y*sin(phi)*sin(theta_By) - 4*m_rotor*omega_Bx*omega_Bz*r_AR1_y*sin(phi)*sin(theta_By) + 2*m_arm*omega_By*omega_Bz*r_AG_right_z*cos(phi)*cos(theta_By)*sin(theta_Bx) + 4*m_rotor*omega_By*omega_Bz*r_AR1_z*cos(phi)*cos(theta_By)*sin(theta_Bx) + 2*m_arm*omega_By*omega_Bz*r_AG_right_y*cos(theta_By)*sin(phi)*sin(theta_Bx) + 4*m_rotor*omega_By*omega_Bz*r_AR1_y*cos(theta_By)*sin(phi)*sin(theta_Bx),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     I_base_yz*omega_By**2 - I_base_yz*omega_Bz**2 - I_base_xy*omega_Bx*omega_Bz + I_base_xz*omega_Bx*omega_By - I_base_yy*omega_By*omega_Bz + I_base_zz*omega_By*omega_Bz - 2*I_arm_xy*omega_Bx*omega_Bz*cos(phi) - 2*I_arm_xz*omega_Bx*omega_Bz*sin(phi) + 2*m_arm*omega_By*omega_Bz*r_BA_right_y**2 - 2*m_arm*omega_By*omega_Bz*r_BA_right_z**2 + 4*m_rotor*omega_By*omega_Bz*r_BA_right_y**2 - 4*m_rotor*omega_By*omega_Bz*r_BA_right_z**2 - 2*I_arm_yy*omega_By*omega_Bz*cos(2*phi) + 2*I_arm_zz*omega_By*omega_Bz*cos(2*phi) - 4*I_rotor_yy*omega_By*omega_Bz*cos(2*phi) + 4*I_rotor_zz*omega_By*omega_Bz*cos(2*phi) - 4*I_arm_yz*omega_By*omega_Bz*sin(2*phi) - 2*m_arm*omega_Bx*omega_By*r_AG_right_x*r_BA_right_z - 2*m_arm*omega_Bx*omega_By*r_BA_right_x*r_BA_right_z - 4*m_rotor*omega_Bx*omega_By*r_BA_right_x*r_BA_right_z + 4*m_rotor*omega_By*omega_Bz*r_AR1_y**2*cos(2*phi) - 4*m_rotor*omega_By*omega_Bz*r_AR1_z**2*cos(2*phi) - 2*m_arm*omega_Bx*omega_By*r_AG_right_z*r_BA_right_x*cos(phi) + 4*m_arm*omega_By*omega_Bz*r_AG_right_y*r_BA_right_y*cos(phi) - 4*m_arm*omega_By*omega_Bz*r_AG_right_z*r_BA_right_z*cos(phi) - 4*m_rotor*omega_Bx*omega_By*r_AR1_z*r_BA_right_x*cos(phi) + 8*m_rotor*omega_By*omega_Bz*r_AR1_y*r_BA_right_y*cos(phi) - 8*m_rotor*omega_By*omega_Bz*r_AR1_z*r_BA_right_z*cos(phi) - 2*m_arm*omega_Bx*omega_By*r_AG_right_y*r_BA_right_x*sin(phi) - 4*m_arm*omega_By*omega_Bz*r_AG_right_y*r_BA_right_z*sin(phi) - 4*m_arm*omega_By*omega_Bz*r_AG_right_z*r_BA_right_y*sin(phi) - 4*m_rotor*omega_Bx*omega_By*r_AR1_y*r_BA_right_x*sin(phi) - 8*m_rotor*omega_By*omega_Bz*r_AR1_y*r_BA_right_z*sin(phi) - 8*m_rotor*omega_By*omega_Bz*r_AR1_z*r_BA_right_y*sin(phi) - 8*m_rotor*omega_By*omega_Bz*r_AR1_y*r_AR1_z*sin(2*phi),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             I_base_xz*omega_Bz**2 - I_base_xz*omega_Bx**2 + 2*I_arm_xx*omega_Bx*omega_Bz - 2*I_arm_yy*omega_Bx*omega_Bz + I_base_xx*omega_Bx*omega_Bz + I_base_xy*omega_By*omega_Bz - I_base_yz*omega_Bx*omega_By - I_base_zz*omega_Bx*omega_Bz + 4*I_rotor_xx*omega_Bx*omega_Bz - 4*I_rotor_yy*omega_Bx*omega_Bz + 2*I_arm_xy*omega_By*omega_Bz*cos(phi) + 2*I_arm_xz*omega_By*omega_Bz*sin(phi) - 2*m_arm*omega_Bx*omega_Bz*r_BA_right_x**2 + 2*m_arm*omega_Bx*omega_Bz*r_BA_right_z**2 - 4*m_rotor*omega_Bx*omega_Bz*r_AR1_x**2 + 4*m_rotor*omega_Bx*omega_Bz*r_AR1_y**2 - 4*m_rotor*omega_Bx*omega_Bz*r_BA_right_x**2 + 4*m_rotor*omega_Bx*omega_Bz*r_BA_right_z**2 + 2*m_arm*omega_Bx**2*r_AG_right_x*r_BA_right_z - 2*m_arm*omega_Bz**2*r_AG_right_x*r_BA_right_z + 2*m_arm*omega_Bx**2*r_BA_right_x*r_BA_right_z - 2*m_arm*omega_Bz**2*r_BA_right_x*r_BA_right_z + 4*m_rotor*omega_Bx**2*r_BA_right_x*r_BA_right_z - 4*m_rotor*omega_Bz**2*r_BA_right_x*r_BA_right_z + 2*I_arm_yy*omega_Bx*omega_Bz*cos(phi)**2 - 2*I_arm_zz*omega_Bx*omega_Bz*cos(phi)**2 + 4*I_rotor_yy*omega_Bx*omega_Bz*cos(phi)**2 - 4*I_rotor_zz*omega_Bx*omega_Bz*cos(phi)**2 + 2*I_arm_yz*omega_Bx*omega_Bz*sin(2*phi) + 2*m_arm*omega_Bx**2*r_AG_right_z*r_BA_right_x*cos(phi) - 2*m_arm*omega_Bz**2*r_AG_right_z*r_BA_right_x*cos(phi) + 4*m_rotor*omega_Bx**2*r_AR1_z*r_BA_right_x*cos(phi) - 4*m_rotor*omega_Bz**2*r_AR1_z*r_BA_right_x*cos(phi) + 2*m_arm*omega_Bx**2*r_AG_right_y*r_BA_right_x*sin(phi) - 2*m_arm*omega_Bz**2*r_AG_right_y*r_BA_right_x*sin(phi) + 4*m_rotor*omega_Bx**2*r_AR1_y*r_BA_right_x*sin(phi) - 4*m_rotor*omega_Bz**2*r_AR1_y*r_BA_right_x*sin(phi) - 4*m_arm*omega_Bx*omega_Bz*r_AG_right_x*r_BA_right_x - 4*m_rotor*omega_Bx*omega_Bz*r_AR1_y**2*cos(phi)**2 + 4*m_rotor*omega_Bx*omega_Bz*r_AR1_z**2*cos(phi)**2 + 4*m_arm*omega_Bx*omega_Bz*r_AG_right_z*r_BA_right_z*cos(phi) + 8*m_rotor*omega_Bx*omega_Bz*r_AR1_z*r_BA_right_z*cos(phi) + 4*m_arm*omega_Bx*omega_Bz*r_AG_right_y*r_BA_right_z*sin(phi) + 8*m_rotor*omega_Bx*omega_Bz*r_AR1_y*r_BA_right_z*sin(phi) + 4*m_rotor*omega_Bx*omega_Bz*r_AR1_y*r_AR1_z*sin(2*phi),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 I_base_xy*omega_Bx**2 - I_base_xy*omega_By**2 + 2*I_arm_xy*omega_Bx**2*cos(phi) - 2*I_arm_xy*omega_By**2*cos(phi) + 2*I_arm_xz*omega_Bx**2*sin(phi) - 2*I_arm_xz*omega_By**2*sin(phi) - 2*I_arm_xx*omega_Bx*omega_By + 2*I_arm_zz*omega_Bx*omega_By - I_base_xx*omega_Bx*omega_By + I_base_yy*omega_Bx*omega_By - I_base_xz*omega_By*omega_Bz + I_base_yz*omega_Bx*omega_Bz - 4*I_rotor_xx*omega_Bx*omega_By + 4*I_rotor_zz*omega_Bx*omega_By + 2*m_arm*omega_Bx*omega_By*r_BA_right_x**2 - 2*m_arm*omega_Bx*omega_By*r_BA_right_y**2 + 4*m_rotor*omega_Bx*omega_By*r_AR1_x**2 - 4*m_rotor*omega_Bx*omega_By*r_AR1_z**2 + 4*m_rotor*omega_Bx*omega_By*r_BA_right_x**2 - 4*m_rotor*omega_Bx*omega_By*r_BA_right_y**2 + 2*I_arm_yy*omega_Bx*omega_By*cos(phi)**2 - 2*I_arm_zz*omega_Bx*omega_By*cos(phi)**2 + 4*I_rotor_yy*omega_Bx*omega_By*cos(phi)**2 - 4*I_rotor_zz*omega_Bx*omega_By*cos(phi)**2 + 2*I_arm_yz*omega_Bx*omega_By*sin(2*phi) + 4*m_arm*omega_Bx*omega_By*r_AG_right_x*r_BA_right_x + 2*m_arm*omega_By*omega_Bz*r_AG_right_x*r_BA_right_z + 2*m_arm*omega_By*omega_Bz*r_BA_right_x*r_BA_right_z + 4*m_rotor*omega_By*omega_Bz*r_BA_right_x*r_BA_right_z - 4*m_rotor*omega_Bx*omega_By*r_AR1_y**2*cos(phi)**2 + 4*m_rotor*omega_Bx*omega_By*r_AR1_z**2*cos(phi)**2 - 4*m_arm*omega_Bx*omega_By*r_AG_right_y*r_BA_right_y*cos(phi) + 2*m_arm*omega_By*omega_Bz*r_AG_right_z*r_BA_right_x*cos(phi) - 8*m_rotor*omega_Bx*omega_By*r_AR1_y*r_BA_right_y*cos(phi) + 4*m_rotor*omega_By*omega_Bz*r_AR1_z*r_BA_right_x*cos(phi) + 4*m_arm*omega_Bx*omega_By*r_AG_right_z*r_BA_right_y*sin(phi) + 2*m_arm*omega_By*omega_Bz*r_AG_right_y*r_BA_right_x*sin(phi) + 8*m_rotor*omega_Bx*omega_By*r_AR1_z*r_BA_right_y*sin(phi) + 4*m_rotor*omega_By*omega_Bz*r_AR1_y*r_BA_right_x*sin(phi) + 4*m_rotor*omega_Bx*omega_By*r_AR1_y*r_AR1_z*sin(2*phi)        
        ])
    )
    return Function("b_func",[x,phi],[out])

def g_func():
    x   = SX.sym('x',12)
    phi = SX.sym('phi',1)
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
                                                                                                                                                                                                                                                                                                                                                                                 -gravity*(2*m_arm + m_base + 4*m_rotor),
                                                                                                                                                                                                               2*gravity*cos(theta_By)*sin(theta_Bx)*(m_arm*r_BA_right_z + 2*m_rotor*r_BA_right_z + m_arm*r_AG_right_z*cos(phi) + 2*m_rotor*r_AR1_z*cos(phi) + m_arm*r_AG_right_y*sin(phi) + 2*m_rotor*r_AR1_y*sin(phi)),
2*gravity*(m_arm*r_BA_right_z*sin(theta_By) + 2*m_rotor*r_BA_right_z*sin(theta_By) + m_arm*r_AG_right_y*sin(phi)*sin(theta_By) + 2*m_rotor*r_AR1_y*sin(phi)*sin(theta_By) + m_arm*r_AG_right_x*cos(theta_Bx)*cos(theta_By) + m_arm*r_BA_right_x*cos(theta_Bx)*cos(theta_By) + 2*m_rotor*r_BA_right_x*cos(theta_Bx)*cos(theta_By) + m_arm*r_AG_right_z*cos(phi)*sin(theta_By) + 2*m_rotor*r_AR1_z*cos(phi)*sin(theta_By)),
                                                                                                                                                                                                                                                                                                               -2*gravity*cos(theta_By)*sin(theta_Bx)*(m_arm*r_AG_right_x + m_arm*r_BA_right_x + 2*m_rotor*r_BA_right_x)      
        ])
    )
    return Function("g",[x,phi],[out])

def S_func():
    x   = SX.sym('x',12)
    phi = SX.sym('phi',1)
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
[- kT*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)) - kT*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)), kT*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)) + kT*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)), -kT*cos(phi + theta_Bx)*cos(theta_By), -kT*(r_AR1_y + r_BA_right_y*cos(phi) + r_BA_right_z*sin(phi)), kT*(r_AR1_x*cos(phi) + r_BA_right_x*cos(phi) - kM*sin(phi)),  kT*(kM*cos(phi) + r_AR1_x*sin(phi) + r_BA_right_x*sin(phi))],
[  kT*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)) - kT*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)), kT*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)) - kT*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)), -kT*cos(phi - theta_Bx)*cos(theta_By),  kT*(r_AR1_y + r_BA_right_y*cos(phi) + r_BA_right_z*sin(phi)), kT*(r_BA_right_x*cos(phi) - r_AR1_x*cos(phi) + kM*sin(phi)),  kT*(kM*cos(phi) + r_AR1_x*sin(phi) - r_BA_right_x*sin(phi))],
[  kT*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)) - kT*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)), kT*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)) - kT*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)), -kT*cos(phi - theta_Bx)*cos(theta_By),  kT*(r_AR1_y + r_BA_right_y*cos(phi) + r_BA_right_z*sin(phi)), kT*(r_AR1_x*cos(phi) + r_BA_right_x*cos(phi) - kM*sin(phi)), -kT*(kM*cos(phi) + r_AR1_x*sin(phi) + r_BA_right_x*sin(phi))],
[- kT*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)) - kT*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)), kT*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)) + kT*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)), -kT*cos(phi + theta_Bx)*cos(theta_By), -kT*(r_AR1_y + r_BA_right_y*cos(phi) + r_BA_right_z*sin(phi)), kT*(r_BA_right_x*cos(phi) - r_AR1_x*cos(phi) + kM*sin(phi)), -kT*(kM*cos(phi) + r_AR1_x*sin(phi) - r_BA_right_x*sin(phi))]        
        ])
    )
    return Function("S_func",[x,phi],[out])

def F_func():
    theta_Bx = SX.sym('theta_Bx',1)
    theta_By = SX.sym('theta_By',1)
    theta_Bz = SX.sym('theta_Bz',1)
    # transforms omega to chi
    out = SX(
        np.array([
        [0,                 sin(theta_Bx)/cos(theta_By),                 cos(theta_Bx)/cos(theta_By)],
        [0,                               cos(theta_Bx),                              -sin(theta_Bx)],
        [1, (sin(theta_Bx)*sin(theta_By))/cos(theta_By), (cos(theta_Bx)*sin(theta_By))/cos(theta_By)]
    ])
    )
    return Function("F_func",[theta_Bx,theta_By,theta_Bz],[out])

def G_func():
    theta_Bx = SX.sym('theta_Bx',1)
    theta_By = SX.sym('theta_By',1)
    theta_Bz = SX.sym('theta_Bz',1)
    # transforms chi to omega
    out = SX(
        np.array([
            [     -sin(theta_By),              0, 1],
            [cos(theta_By)*sin(theta_Bx),  cos(theta_Bx), 0],
            [cos(theta_Bx)*cos(theta_By), -sin(theta_Bx), 0]
        ])
    )
    return Function("G_func",[theta_Bx,theta_By,theta_Bz],[out])

# full dynamics functions
def dynamics_func():
    # get symbolic variables
    X      = MX.sym('X',12,1)
    U      = MX.sym('U',4,1)
    varphi = MX.sym('varphi',1,1)

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
    M_x = M_func()(X,varphi)
    b_x = b_func()(X,varphi)
    g_x = g_func()(X,varphi)  
    S_x = S_func()(X,varphi)

    # compute inverse mass matrix
    Minv = inv(M_x)
    du = Minv @ (S_x.T @ U - b_x - g_x)

    chid = F_func()(theta_Bx,theta_By,theta_Bz) @ vertcat(omega_Bx,omega_By,omega_Bz)

    # finally write explicit dynamics
    f_expl = vertcat(u[0],u[1],u[2],chid,du)
    
    return Function('dynamics_func',[X,U,varphi],[f_expl])

def dynamics_with_res_func():
    # get symbolic variables
    X      = MX.sym('X',12,1)
    U      = MX.sym('U',4,1)
    varphi = MX.sym('varphi',1,1)

    # get nominal dynamics
    f_expl = dynamics_func()(X,U,varphi)

    # get residual function
    residual_func = f_res()

    # get symbolic parameters
    params_sym = l4c_residual_model.get_sym_params()

    # get conditioning variables
    cond = MX([])
    for state_idx in model_states_in_idx:
        cond = vertcat(cond,X[state_idx])
    for input_idx in model_inputs_in_idx:
        cond = vertcat(cond,U[input_idx])
    if model_phi_in:
        cond = vertcat(cond,varphi)  

    # add residual dynamics to nominal dynamics
    f_residual = residual_func(cond.T,params_sym)
    count = 0

    for state_idx in model_states_out_idx:
        f_expl[state_idx] += f_residual[count]
        count += 1

    return Function('dynamics_with_res_func',[X,U,varphi,params_sym],[f_expl])

def dynamics_ground_effect_func():
    # get symbolic variables
    X      = MX.sym('X',12,1)
    U      = MX.sym('U',4,1)
    varphi = MX.sym('varphi',1,1)

    # get nominal dynamics without ground effect
    f_expl = dynamics_func()(X,U,varphi)

    # add ground effect dynamics to z acceleration
    z = X[2]
    R = 0.1143
    ground_effect = if_else(fabs(z)/R < 5,1 + (-1.0/5.0*fabs(z/R)+1.0),1.0)
    f_expl[8] += -(ground_effect-1) * kT * (U[0] + U[1] + U[2] + U[3]) * cos(varphi) / m

    return Function('dynamics_ground_effect_func',[X,U,varphi],[f_expl])

def dynamics_phi_func():
    # get symbolic variables
    X_no_varphi      = MX.sym('X',12,1)
    U_no_v           = MX.sym('U',4,1)
    varphi           = MX.sym('varphi',1,1)

    # get dynamics without phi dynamics
    f_expl = dynamics_func()(X_no_varphi,U_no_v,varphi)
 
    # get phi dynamics
    coeff = 0.3905
    theta = theta_fit(varphi)
    v_max = 2*pi/60*(220 - 0.14*coeff*(U[0]+U[1]+U[2]+U[3])*kT/gravity*sin(theta))

    # finally write explicit dynamics including phi
    v = MX.sym('v',1,1)
    f_expl = vertcat(f_expl,v_max*v)

    # get augmented state and input variables
    X = vertcat(X_no_varphi,varphi)
    U = vertcat(U,v)

    return Function('dynamics_phi_func',[X,U],[f_expl])

def dynamics_with_int_action_func():
    # get symbolic variables
    X      = MX.sym('X',13,1)
    U      = MX.sym('U',4,1)
    varphi = MX.sym('varphi',1,1)
    z_ref  = MX.sym('z_ref',1,1)

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
    xi       = X[12]

    # q, u
    q = vertcat(x_B,y_B,z_B,theta_Bz,theta_By,theta_Bx)
    u = vertcat(dx_B,dy_B,dz_B,omega_Bx,omega_By,omega_Bz)

    # get dynamics matrices
    M_x = M_func()(X[:-1],varphi)
    b_x = b_func()(X[:-1],varphi)
    g_x = g_func()(X[:-1],varphi)  
    S_x = S_func()(X[:-1],varphi)

    # compute inverse mass matrix
    Minv = inv(M_x)
    du = Minv @ (S_x.T @ U - b_x - g_x)

    chid = F_func()(theta_Bx,theta_By,theta_Bz) @ vertcat(omega_Bx,omega_By,omega_Bz)

    # finally write explicit dynamics
    f_expl = vertcat(u[0],u[1],u[2],chid,du,integral_gain*(z_B-z_ref))
    
    return Function('dynamics_with_int_action_func',[X,U,varphi,z_ref],[f_expl])

# driving dynamics
def driving_dynamics_func():
    # get symbolic variables
    X      = MX.sym('X',3,1)
    U      = MX.sym('U',2,1)

    # get state variables
    x      = X[0]
    y      = X[1]
    theta  = X[2]

    # get input variables
    v      = U[0]
    omega  = U[1]

    # finally write explicit dynamics
    f_expl = vertcat(v*cos(theta),v*sin(theta),omega)
    
    return Function('driving_dynamics_func',[X,U],[f_expl])    

# hybrid dynamics
def hybrid_dynamics_func():
    # get symbolic variables
    X        = MX.sym('X',12,1)
    U        = MX.sym('U',6,1)
    varphi   = MX.sym('varphi',1,1)
    grounded = MX.sym('grounded',1,1)

    # get grounded flag
    lam     = if_else(grounded,1.0,0.0)
    not_lam = if_else(grounded,0.0,1.0)

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
    M_x = M_func()(X,varphi)
    b_x = b_func()(X,varphi)
    g_x = g_func()(X,varphi)  
    S_x = S_func()(X,varphi)

    # compute inverse mass matrix
    Minv = inv(M_x)
    du = not_lam * Minv @ (S_x.T @ U[:-2] - b_x - g_x)

    chid = F_func()(theta_Bx,theta_By,theta_Bz) @ vertcat(omega_Bx,omega_By,omega_Bz)

    # get driving dynamics
    ug = driving_dynamics_func()(vertcat(X[0],X[1],X[3]),U[4:6])

    # apply driving dynamics only if grounded
    ug  = lam * ug

    chid[0] = chid[0] + ug[2]

    # finally write explicit dynamics
    f_expl = vertcat(u[0]+ug[0],u[1]+ug[1],u[2],chid,du)
    
    return Function('hybrid_dynamics_func',[X,U,varphi,grounded],[f_expl])

# special functions
def f():
    X = MX.sym("X",12)
    U = MX.sym("U",4)    
    Varphi = MX.sym("Varphi",1)
    return Function('f',[X,U,Varphi],[dynamics_func()(X,U,Varphi)])

def f_res():
    in_sym      = MX.sym('in_sym',params_.get('model_ninputs'),1)
    y_sym       = l4c_residual_model(in_sym)
    params_sym  = l4c_residual_model.get_sym_params()
    return Function('f_res',[in_sym, params_sym],[y_sym])

def f_ground_effect():
    X = MX.sym("X",12)
    U = MX.sym("U",4)    
    Varphi = MX.sym("Varphi",1)
    return Function('f',[X,U,Varphi],[dynamics_ground_effect_func()(X,U,Varphi)])

def rk4():
    DT = MX.sym("DT", 1)
    X = MX.sym("X",12)
    U = MX.sym("U",4)    
    Varphi = MX.sym("Varphi",1)

    # Fixed step Runge-Kutta 4 integrator
    k1 = dynamics_func()(X, U, Varphi)
    k2 = dynamics_func()(X + DT / 2 * k1, U, Varphi)
    k3 = dynamics_func()(X + DT / 2 * k2, U, Varphi)
    k4 = dynamics_func()(X + DT * k3, U, Varphi)
    x_next = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    func = Function('rk4', [DT, X, U, Varphi], [x_next])
    return func

def rk4_with_res():
    DT = MX.sym("DT", 1)
    X = MX.sym("X",12)
    U = MX.sym("U",4)    
    Varphi = MX.sym("Varphi",1)
    params_sym  = l4c_residual_model.get_sym_params()

    # Fixed step Runge-Kutta 4 integrator
    k1 = dynamics_with_res_func()(X, U, Varphi,params_sym)
    k2 = dynamics_with_res_func()(X + DT / 2 * k1, U, Varphi,params_sym)
    k3 = dynamics_with_res_func()(X + DT / 2 * k2, U, Varphi,params_sym)
    k4 = dynamics_with_res_func()(X + DT * k3, U, Varphi,params_sym)
    x_next = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    func = Function('rk4_with_res', [DT, X, U, Varphi, params_sym], [x_next])
    return func

def rk4_with_disc_res():
    DT = MX.sym("DT", 1)
    X = MX.sym("X",12)
    U = MX.sym("U",4)    
    Varphi = MX.sym("Varphi",1)
    params_sym  = l4c_residual_model.get_sym_params()

    # Fixed step Runge-Kutta 4 integrator
    k1 = dynamics_func()(X, U, Varphi)
    k2 = dynamics_func()(X + DT / 2 * k1, U, Varphi)
    k3 = dynamics_func()(X + DT / 2 * k2, U, Varphi)
    k4 = dynamics_func()(X + DT * k3, U, Varphi)
    x_next = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    # get residual function
    F_res = f_res()

    # add the discrete residual dynamics
    # get conditioning variables
    cond = MX([])
    for state_idx in model_states_in_idx:
        cond = vertcat(cond,X[state_idx])
    for input_idx in model_inputs_in_idx:
        cond = vertcat(cond,U[input_idx])
    if model_phi_in:
        cond = vertcat(cond,Varphi)  
    if model_dt_in:
        cond = vertcat(cond,DT)

    # add residual dynamics to nominal dynamics
    F_residual = F_res(cond.T,params_sym)
    count = 0

    for state_idx in model_states_out_idx:
        x_next[state_idx] += F_residual[count]
        count += 1

    return Function('rk4_with_disc_res', [DT, X, U, Varphi, params_sym], [x_next])

def rk4_with_ground_effect():
    DT = MX.sym("DT", 1)
    X = MX.sym("X",12)
    U = MX.sym("U",4)    
    Varphi = MX.sym("Varphi",1)

    # Fixed step Runge-Kutta 4 integrator
    k1 = dynamics_ground_effect_func()(X, U, Varphi)
    k2 = dynamics_ground_effect_func()(X + DT / 2 * k1, U, Varphi)
    k3 = dynamics_ground_effect_func()(X + DT / 2 * k2, U, Varphi)
    k4 = dynamics_ground_effect_func()(X + DT * k3, U, Varphi)
    x_next = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    func = Function('rk4_with_ground_effect', [DT, X, U, Varphi], [x_next])
    return func    

def rk4_with_int_action():
    DT = MX.sym("DT", 1)
    X = MX.sym("X",13)
    U = MX.sym("U",4)    
    Varphi = MX.sym("Varphi",1)
    z_ref  = MX.sym('z_ref',1,1)

    # Fixed step Runge-Kutta 4 integrator
    k1 = dynamics_with_int_action_func()(X, U, Varphi, z_ref)
    k2 = dynamics_with_int_action_func()(X + DT / 2 * k1, U, Varphi, z_ref)
    k3 = dynamics_with_int_action_func()(X + DT / 2 * k2, U, Varphi, z_ref)
    k4 = dynamics_with_int_action_func()(X + DT * k3, U, Varphi, z_ref)
    x_next = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    func = Function('rk4', [DT, X, U, Varphi, z_ref], [x_next])
    return func

def A_func(x,u,varphi):
    X = MX.sym("X",12)
    U = MX.sym("U",4)   
    func = Function('A_func',[X,U],[jacobian(dynamics_func()(X,U,varphi),X)])
    return np.array(func(x,u))

def B_func(x,u,varphi):
    X = MX.sym("X",12)
    U = MX.sym("U",4)    
    func = Function('B_func',[X,U],[jacobian(dynamics_func()(X,U,varphi),U)])
    return np.array(func(x,u))

# useable acados models
def export_robot_model() -> AcadosModel:
    model_name = "morphing_lander"

    # tilt angle
    varphi = MX.sym("varphi",1)
    p = varphi

    # states and controls
    X = MX.sym("X",12)
    U = MX.sym("U",4)

    if use_residual_model:
        # add realtime approximation parameters to acados parameters
        dummy = l4c_residual_model(MX(model_ninputs,1))
        params_sym = l4c_residual_model.get_sym_params()
        p = vertcat(p,params_sym)   

        # Explicit dynamics with residual correction
        f_expl = dynamics_with_res_func()(X,U,varphi,params_sym)
    else:
        # Explicit nominal dynamics
        f_expl = dynamics_func()(X,U,varphi)

    # xdot
    x_B_dot      = MX.sym("x_B_dot")
    y_B_dot      = MX.sym("y_B_dot")
    z_B_dot      = MX.sym("z_B_dot")
    theta_Bz_dot = MX.sym("theta_Bz_dot")
    theta_By_dot = MX.sym("theta_By_dot")
    theta_Bx_dot = MX.sym("theta_Bx_dot")
    dx_B_dot     = MX.sym("dx_B_dot")
    dy_B_dot     = MX.sym("dy_B_dot")
    dz_B_dot     = MX.sym("dz_B_dot")
    omega_Bx_dot = MX.sym("omega_Bx_dot")
    omega_By_dot = MX.sym("omega_By_dot")
    omega_Bz_dot = MX.sym("omega_Bz_dot")

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

def export_hybrid_robot_model() -> AcadosModel:
    model_name = "hybrid_morphing_lander"

    # tilt angle
    varphi = MX.sym("varphi",1)

    # grounded 
    grounded = MX.sym("grounded",1)

    # write parameters
    p = vertcat(varphi,grounded)

    # states and controls
    X = MX.sym("X",12)
    U = MX.sym("U",6)

    # Explicit nominal dynamics
    f_expl = hybrid_dynamics_func()(X,U,varphi,grounded)

    # xdot
    x_B_dot      = MX.sym("x_B_dot")
    y_B_dot      = MX.sym("y_B_dot")
    z_B_dot      = MX.sym("z_B_dot")
    theta_Bz_dot = MX.sym("theta_Bz_dot")
    theta_By_dot = MX.sym("theta_By_dot")
    theta_Bx_dot = MX.sym("theta_Bx_dot")
    dx_B_dot     = MX.sym("dx_B_dot")
    dy_B_dot     = MX.sym("dy_B_dot")
    dz_B_dot     = MX.sym("dz_B_dot")
    omega_Bx_dot = MX.sym("omega_Bx_dot")
    omega_By_dot = MX.sym("omega_By_dot")
    omega_Bz_dot = MX.sym("omega_Bz_dot")

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

def export_discrete_robot_model(DT) -> AcadosModel:
    model_name = "morphing_lander"

    # tilt angle
    varphi = MX.sym("varphi",1)
    p = varphi

    # states and controls
    X = MX.sym("X",12)
    U = MX.sym("U",4)

    if use_residual_model:
        # add realtime approximation parameters to acados parameters
        dummy = l4c_residual_model(MX(model_ninputs,1))
        params_sym = l4c_residual_model.get_sym_params()
        p = vertcat(p,params_sym)   

        # get discrete dynamics with residual correction
        F = rk4_with_disc_res()(DT,X,U,varphi,params_sym)

    else:
        # get discrete nominal dynamics
        F = rk4()(DT,X,U,varphi)

    model = AcadosModel()
    model.name = model_name
    model.x = X
    model.u = U
    model.p = p
    model.disc_dyn_expr = F

    return model

def export_robot_model_driving() -> AcadosModel:
    model_name = "morphing_lander_driving"

    # states and controls
    X = MX.sym("X",3)
    U = MX.sym("U",2)

    # Explicit nominal dynamics
    f_expl = driving_dynamics_func()(X,U)

    # xdot
    x_dot      = MX.sym("x_B_dot")
    y_dot      = MX.sym("y_B_dot")
    theta_dot  = MX.sym("z_B_dot")

    Xdot = vertcat(x_dot,y_dot,theta_dot)

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

def export_robot_model_ground_effect() -> AcadosModel:
    model_name = "morphing_lander"

    # tilt angle
    varphi = MX.sym("varphi",1)
    p = varphi

    # states and controls
    X = MX.sym("X",12)
    U = MX.sym("U",4)

    # Explicit dynamics
    f_expl = dynamics_ground_effect_func()(X,U,varphi)

    # xdot
    x_B_dot      = MX.sym("x_B_dot")
    y_B_dot      = MX.sym("y_B_dot")
    z_B_dot      = MX.sym("z_B_dot")
    theta_Bz_dot = MX.sym("theta_Bz_dot")
    theta_By_dot = MX.sym("theta_By_dot")
    theta_Bx_dot = MX.sym("theta_Bx_dot")
    dx_B_dot     = MX.sym("dx_B_dot")
    dy_B_dot     = MX.sym("dy_B_dot")
    dz_B_dot     = MX.sym("dz_B_dot")
    omega_Bx_dot = MX.sym("omega_Bx_dot")
    omega_By_dot = MX.sym("omega_By_dot")
    omega_Bz_dot = MX.sym("omega_Bz_dot")

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

def export_discrete_robot_model_with_int_action(DT) -> AcadosModel:
    model_name = "morphing_lander"

    # tilt angle
    varphi = MX.sym("varphi",1)
    z_ref  = MX.sym("z_ref", 1)
    p = vertcat(varphi,z_ref)

    # states and controls
    X = MX.sym("X",13)
    U = MX.sym("U",4)

    # get discrete nominal dynamics
    F = rk4_with_int_action()(DT,X,U,varphi,z_ref)

    model = AcadosModel()
    model.name = model_name
    model.x = X
    model.u = U
    model.p = p
    model.disc_dyn_expr = F

    return model

def export_robot_model_phi() -> AcadosModel:
    model_name = "morphing_lander"

    # states and controls
    X = MX.sym("X",13)
    U = MX.sym("U",5)

    # Explicit dynamics
    f_expl = dynamics_phi_func()(X,U)

    # xdot
    x_B_dot = MX.sym("x_B_dot")
    y_B_dot = MX.sym("y_B_dot")
    z_B_dot = MX.sym("z_B_dot")
    theta_Bz_dot = MX.sym("theta_Bz_dot")
    theta_By_dot = MX.sym("theta_By_dot")
    theta_Bx_dot = MX.sym("theta_Bx_dot")
    dx_B_dot = MX.sym("dx_B_dot")
    dy_B_dot = MX.sym("dy_B_dot")
    dz_B_dot = MX.sym("dz_B_dot")
    omega_Bx_dot = MX.sym("omega_Bx_dot")
    omega_By_dot = MX.sym("omega_By_dot")
    omega_Bz_dot = MX.sym("omega_Bz_dot")
    varphi_dot = MX.sym("varphi_dot")

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

    # Create symbolic variables
    X_sym = SX.sym('X', len(X))
    phi_sym = SX.sym('phi',1)

    # Call the original function with symbolic variables
    symbolic_result = S_func(X_sym,phi_sym)

    # Create a function to evaluate the symbolic expression numerically
    numerical_function = Function('S_numeric', [X_sym,phi_sym], [symbolic_result])

    # Evaluate the symbolic expression numerically
    numerical_result = numerical_function(X,phi)

    return np.array(numerical_result)