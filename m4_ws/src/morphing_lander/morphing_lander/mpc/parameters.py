from os import getenv
from numpy import pi,diag,deg2rad
from pathlib import Path

# learning imports
# from l4casadi.realtime import RealTimeL4CasADi  
# import torch, yaml, os
# from morphing_lander.cvae.train import TrainConfigCVAE, TrainConfigMLP
# from morphing_lander.cvae.models import CVAE, MLP, MLPZero, CVAEZero

# def load_trained_cvae(model_path,ninputs,noutputs):
#     if model_path is None:
#         model = CVAE(
#             output_dim=noutputs,
#             latent_dim=2,
#             cond_dim=ninputs,
#             encoder_layers=[32,32],
#             decoder_layers=[32,32],
#             prior_layers=[32,32],
#         )
#         model.to('cuda')
#     else:
#         # Load the config from file
#         model_path = Path(model_path)
#         config_file = model_path / "config.yaml"
#         with open(config_file, "r") as f:
#             config_dict = yaml.safe_load(f)
#             config = TrainConfigCVAE(**config_dict)

#         # Load the latest checkpoint (of all .pth files under model_path)
#         model_file = max(model_path.glob("*.pth"), key=os.path.getctime)
#         model = CVAE(
#             output_dim=config.output_dim,
#             latent_dim=config.latent_dim,
#             cond_dim=config.cond_dim,
#             encoder_layers=config.encoder_layers,
#             decoder_layers=config.decoder_layers,
#             prior_layers=config.prior_layers,
#         )
#         model.load_state_dict(torch.load(model_file))
#         model.to(config.device)
#     model.eval()
#     return model

# def load_trained_mlp(model_path):
#     # Load the config from file
#     config_file = model_path / "config.yaml"
#     with open(config_file, "r") as f:
#         config_dict = yaml.safe_load(f)
#         config = TrainConfigMLP(**config_dict)

#     # Load the latest checkpoint (of all .pth files under model_path)
#     model_file = max(model_path.glob("*.pth"), key=os.path.getctime)
#     model = MLP(
#         input_dim=config.input_dim,
#         output_dim=config.output_dim,
#         hidden_dims=config.hidden_layers,
#     )
#     model.load_state_dict(torch.load(model_file))
#     model.to(config.device)
#     model.eval()
#     return model

# declare parameter dictionary
params_ = {}

# high level parameters
params_['Ts']                    = 0.007                          # control frequency of MPC
params_['Ts_tilt_controller']    = params_.get('Ts')              # control frequency of TiltController
params_['Ts_drive_controller']   = params_.get('Ts')              # control frequency of DriveController

params_['queue_size']            = 1                              # queue size of ros2 messages
params_['warmup_time']           = 1.0                            # time after which mpc controller is started (seconds)

# acados mpc solver and integrator paths
params_['acados_ocp_path']               = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados_models/'
params_['acados_sim_path']               = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados_sims/'
params_['acados_ocp_path_driving']       = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados_models_driving/'
params_['acados_sim_path_driving']       = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados_sims_driving/'
params_['acados_ocp_path_hybrid']       = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados_models_hybrid/'
params_['acados_sim_path_hybrid']       = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados_sims_hybrid/'

# generate and build flags
params_['generate_mpc']          = True
params_['build_mpc']             = True

# use residual model ?
params_['use_residual_model']    = False

# residual model training parameters
params_['model_states_in_idx']   = [2]
params_['model_inputs_in_idx']   = [0,1,2,3]
params_['model_phi_in']          = True
params_['model_dt_in']           = False
params_['model_states_out_idx']  = [8]
params_['model_ninputs']         = len(params_.get('model_states_in_idx')) + len(params_.get('model_inputs_in_idx')) + params_.get('model_phi_in') + params_.get('model_dt_in')
params_['model_noutputs']        = len(params_.get('model_states_out_idx'))

# l4casadi build path
params_['device']                = 'cuda'

params_['learned_model_path']    = None
params_['l4c_residual_model']    = None
# params_['l4c_residual_model']    = RealTimeL4CasADi(load_trained_cvae(params_.get('learned_model_path'),
#                                                                       params_.get('model_ninputs'),
#                                                                       params_.get('model_noutputs')), 
#                                                     approximation_order=1,
#                                                     device=params_.get('device'))

# gazebo real time factor
params_['real_time_factor']      = 1.0

# max velocities
params_['max_dx']                = 0.5
params_['max_dy']                = 0.5
params_['max_dz']                = 0.5

# rc inputs
params_['min']                   = 1094
params_['max']                   = 1934
params_['dead']                  = 1514

# safety parameters
params_['max_tilt_in_flight']    = deg2rad(50)
params_['max_tilt_on_land']      = deg2rad(85)

# ground detector parameters
params_['land_height']    = -0.07                    # height at which we consider robot landed
params_['takeoff_height'] = -0.60                     # height at which we consider robot in flight
# params_['land_height']    = -0.23                     # height at which we consider robot landed
# params_['takeoff_height'] = -0.60                     # height at which we consider robot in flight

# trajectory parameters
params_['z0'] = 0.0
params_['zf'] = -0.05
# params_['z0'] = -0.11
# params_['zf'] = -0.21

# emergency parameters
params_['emergency_descent_velocity'] = 0.3           # emergency descent velocity if in strange scenario

# mpc parameters
params_['N_horizon']  = 10
params_['T_horizon']  = 1.0
params_['N_horizon_driving']  = 20
params_['T_horizon_driving']  = 2.0


# kinematic driving parameters
params_['wheel_base']                 = 0.135      # half the distance between the wheels
params_['wheel_radius']               = 0.125      # the wheel radius 
params_['max_wheel_angular_velocity'] = 10.0      # rad/s
params_['max_drive_speed']            = params_.get('wheel_radius') * params_.get('max_wheel_angular_velocity')  # m/s
params_['max_turn_speed']             = params_.get('wheel_radius') * params_.get('max_wheel_angular_velocity') / params_.get('wheel_base')   # rad/s

# dynamics parameters (use z-down parameters from excel sheet even though urdf uses z-up parameters)
params_['g']                = 9.81                                                                   # gravitational acceleration
params_['kT']               = 28.15                                                                  # thrust coefficient
params_['kM']               = 0.018                                                                  # moment coefficient
params_['m_base']           = 2.33                                                                   # mass of base
params_['m_arm']            = 1.537                                                                  # mass of arm
params_['m_rotor']          = 0.021                                                                  # mass of rotor 
params_['m']                = params_['m_base'] + 2*params_['m_arm'] + 4*params_['m_rotor']          # total mass 
params_['I_base_xx']        = 0.0067
params_['I_base_yy']        = 0.011
params_['I_base_zz']        = 0.0088
params_['I_base_xy']        = -0.000031
params_['I_base_xz']        = 0.00046
params_['I_base_yz']        = 0.00004
params_['I_arm_xx']         = 0.008732
params_['I_arm_yy']         = 0.036926
params_['I_arm_zz']         = 0.043822
params_['I_arm_xy']         = 0.000007
params_['I_arm_xz']         = -0.000012
params_['I_arm_yz']         = 0.000571
params_['I_rotor_xx']       = 0.000022
params_['I_rotor_yy']       = 0.000022
params_['I_rotor_zz']       = 0.000043
params_['r_BA_right_x']     = 0.0066
params_['r_BA_right_y']     = 0.0685
params_['r_BA_right_z']     = -0.021
params_['r_AG_right_x']     = -0.00032
params_['r_AG_right_y']     = 0.16739
params_['r_AG_right_z']     = -0.02495
params_['r_AR1_x']          = 0.16491
params_['r_AR1_y']          = 0.13673
params_['r_AR1_z']          = -0.069563

# constraint variables
params_['u_max']            = 1.0
params_['v_max_absolute']   = (pi/2)/4
params_['T_max']            = 4*params_['kT']

# integral state feedback term
params_['integral_gain'] = 1.0

# cost function parameters
params_['w_x']        = 10.0
params_['w_y']        = 10.0
params_['w_z']        = 10.0
params_['w_dx']       = 0.1
params_['w_dy']       = 0.1
params_['w_dz']       = 0.1
params_['w_phi']      = 0.05
params_['w_th']       = 0.05
params_['w_psi']      = 0.05
params_['w_ox']       = 0.05
params_['w_oy']       = 0.05
params_['w_oz']       = 0.05
params_['w_u']        = 1.0
params_['w_int']      = 0.0

params_['rho']   = 0.1
params_['gamma'] = 1.0

# cost function
params_['Q_mat']          = diag([params_['w_x'],params_['w_y'],params_['w_z'],params_['w_psi'],params_['w_th'],params_['w_phi'],params_['w_dx'],params_['w_dy'],params_['w_dz'],params_['w_ox'],params_['w_oy'],params_['w_oz']])
params_['R_mat']          = params_['rho'] * diag([params_['w_u'],params_['w_u'],params_['w_u'],params_['w_u']])
params_['Q_mat_terminal'] = params_['gamma'] * params_['Q_mat']

# cost function driving
params_['Q_mat_driving']          = diag([1.,1.,1.])
params_['R_mat_driving']          = 1e-5 * diag([1.,1.])
params_['Q_mat_terminal_driving'] = params_['Q_mat_driving']