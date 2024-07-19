import numpy as np
import onnxruntime as ort
from os import getenv

class ONNXModel:
   def __init__(self, model_path):
       self.model = ort.InferenceSession(model_path)

   def preprocess_obs(self,x_current,phi_current):
        # obs shape should be :  (1, 19)
        # obs type should be  :  <class 'numpy.ndarray'>
        
        pos_transformed        = np.array([x_current[0], -x_current[1],-x_current[2]])

        # quat                   = R.from_euler('zyx', [x_current[3],x_current[4],x_current[5]]).as_quat()
        quat = x_current[3:7]
        quat_transformed       = np.array([quat[0], -quat[1], -quat[2], quat[3]])

        # quat_transformed       = np.array([quat[0], -quat[1], -quat[2], quat[3]])
        vel_transformed        = np.array([x_current[7],-x_current[8],-x_current[9]])
        rotvel_transformed     = np.array([x_current[10], -x_current[11], -x_current[12]])
        x_current_transformed  = np.hstack((pos_transformed,quat_transformed,vel_transformed,rotvel_transformed))
        obs = np.hstack((x_current_transformed, np.array([phi_current])/(np.pi/2)))
        obs = obs[np.newaxis,:]
        # print obs
        print(f"obs :  {obs}")
        return obs.astype(np.float32)
    
   def predict(self, obs):
       outputs = self.model.run(
           None,
           {"obs": obs},
       )
       return outputs
   
   def postprocess_actions(self, outputs):
    #    new_outputs = np.zeros((1, 3, 5), dtype=np.float32)
       new_outputs = np.zeros((1, 3, 4), dtype=np.float32)
       new_outputs[0][0][0] = np.clip(outputs[0][0][0], -1, 1)
       new_outputs[0][0][1] = np.clip(outputs[0][0][1], -1, 1)
       new_outputs[0][0][2] = np.clip(outputs[0][0][2], -1, 1)
       new_outputs[0][0][3] = np.clip(outputs[0][0][3], -1, 1)
    #    new_outputs[0][0][4] = np.clip(outputs[0][0][4], -1, 1)
       
       new_outputs[0][0][0] = (new_outputs[0][0][0] + 1) / 2
       new_outputs[0][0][1] = (new_outputs[0][0][1] + 1) / 2
       new_outputs[0][0][2] = (new_outputs[0][0][2] + 1) / 2
       new_outputs[0][0][3] = (new_outputs[0][0][3] + 1) / 2

    #    fifth action : set to 0 if between -0.25 and 0.25, if higher than 0.25 set to 1, if lower than -0.25 set to -1
    #    new_outputs[0][0][4] = 0 if -0.25 < new_outputs[0][0][4] < 0.25 else 0.95*np.sign(new_outputs[0][0][4])
    #    new_outputs[0][0][4] = -new_outputs[0][0][4]   
       return new_outputs[0][0].squeeze()

# declare parameter dictionary
params_ = {}

# high level parameters
params_['Ts']                    = 0.007                          # control frequency of MPC
params_['Ts_tilt_controller']    = params_.get('Ts')              # control frequency of TiltController
params_['Ts_drive_controller']   = params_.get('Ts')              # control frequency of DriveController
params_['queue_size']            = 1                              # queue size of ros2 messages
params_['warmup_time']           = 1.0                            # time after which mpc controller is started (seconds)
params_['cost_update_freq']      = 0.007                           # frequency at which cost is updated (seconds)

# generate and build flags
params_['generate_mpc']          = False
params_['build_mpc']             = False

# manual control parameters
params_['max_dx']                = 1.0                            # max x velocity
params_['max_dy']                = 1.0                            # max y velocity
params_['max_dz']                = 0.75                           # max z velocity
params_['max_dpsi']              = np.pi/4                        # max yaw angular velocity
params_['tilt_height']           = -1.5                           # height at which tilt is enabled
params_['initial_tilt_sim']      = 0.0                            # initial tilt angle in simulation (radian)

# safety parameters
params_['max_tilt_in_flight']    = np.deg2rad(50)
params_['max_tilt_on_land']      = np.deg2rad(85)

# transition parameters
params_['use_rl_for_transition'] = False
params_['l_pivot_wheel']         = 0.261                                                                                 # distance from pivot point to wheel exterior (look at y distance in SDF and add radius of wheel (0.125))
params_['h_bot_pivot']           = 0.094                                                                                 # distance from bottom plate to pivot point (look at z distance in SDF)
params_['varphi_g']              = np.arctan(params_.get('h_bot_pivot')/params_.get('l_pivot_wheel'))                    # angle of pivot point when robot is on ground and wheels just touch the ground
params_['z_ground_base']         = -0.0                                                                                # (exp: -0.113 TBD) height that optitrack registers when robot is on ground with arms at 0 degrees
params_['h_wheel_ground']        = -0.2         # distance from wheel to ground when transition begins                                                 
params_['z_star']                = params_.get('h_wheel_ground') + params_.get('z_ground_base') - (params_.get('l_pivot_wheel') * np.sin(params_.get('max_tilt_in_flight')) - params_.get('h_bot_pivot'))                
params_['u_ramp_down_rate']      = 0.15                                                                                  # rate at which reference input ramps down

# state machine parameters
params_['land_tolerance']        = -0.03                          # tolerance to consider robot landed
params_['takeoff_height']        = -2.0                           # (exp: -1.0) height at which we consider robot in flight

# trajectory parameters
params_['z0']                    = 0.0                            # (exp: -0.11)
params_['zf']                    = -0.125                          # (exp: -0.21)

# collocation parameters flight
params_['N_horizon']             = 10
params_['T_horizon']             = 1.0

# collocation parameters near ground
params_['N_horizon_near_ground'] = 10
params_['T_horizon_near_ground'] = 1.0

# collocation parameters driving
params_['N_horizon_driving']     = 20
params_['T_horizon_driving']     = 2.0

# paths flight model
params_['acados_ocp_path']               = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados/acados_models/'
params_['acados_sim_path']               = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados/acados_sims/'

# paths near ground model
params_['acados_ocp_path_near_ground']   = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados/acados_models_near_ground/'
params_['acados_sim_path_near_ground']   = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados/acados_sims_near_ground/'

# paths driving model
params_['acados_ocp_path_driving']       = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados/acados_models_driving/'
params_['acados_sim_path_driving']       = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados/acados_sims_driving/'

# paths hybrid model
params_['acados_ocp_path_hybrid']        = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados/acados_models_hybrid/'
params_['acados_sim_path_hybrid']        = getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/acados/acados_sims_hybrid/'

# path of RL model
params_['rl_model']                      = ONNXModel(getenv("HOME") +'/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/mpc/rl/MorphingLander2.onnx')

# roboclaw addresses
params_['tilt_roboclaw_address']         = "/dev/ttyACM1"
params_['drive_roboclaw_address']        = "/dev/ttyACM0"

# gazebo real time factor
params_['real_time_factor']              = 1.0

# rc inputs
params_['min']                           = 1094
params_['max']                           = 1934
params_['dead']                          = 1514

# kinematic driving parameters
params_['wheel_base']                    = 0.135      # half the distance between the wheels
params_['wheel_radius']                  = 0.125      # the wheel radius 
params_['max_wheel_angular_velocity']    = 10.0      # rad/s
params_['max_drive_speed']               = params_.get('wheel_radius') * params_.get('max_wheel_angular_velocity')  # m/s
params_['max_turn_speed']                = params_.get('wheel_radius') * params_.get('max_wheel_angular_velocity') / params_.get('wheel_base')   # rad/s

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
params_['v_max_absolute']   = (np.pi/2)/4
params_['T_max']            = 4*params_['kT']

# integral term gain and cost
params_['integral_gain'] = 1.0
params_['w_int']         = 20.0

# cost function parameters (velocity tracking) flight
params_['w_x']        = 1.0     # exp: 1.0
params_['w_y']        = 1.0     # exp: 1.0
params_['w_z']        = 1.0     # exp: 1.0
params_['w_dx']       = 10.0    # exp: 10.0
params_['w_dy']       = 10.0    # exp: 10.0
params_['w_dz']       = 20.0    # exp: 10.0
params_['w_phi']      = 0.1     # exp: 0.1
params_['w_th']       = 0.1     # exp: 0.1
params_['w_psi']      = 0.1     # exp: 0.1
params_['w_ox']       = 3.0     # exp: 3.0, sim: 1.5
params_['w_oy']       = 5.0     # exp: 5.0, sim: 1.5
params_['w_oz']       = 1.5     # exp: 1.5, sim: 1.5
params_['w_u']        = 1.0     # exp: 1.0
params_['rho']        = 0.1     # exp: 0.1
params_['gamma']      = 1.0     # exp: 1.0

# cost function parameters (position tracking) flight
# params_['w_x']        = 10.0    # exp: 10.0
# params_['w_y']        = 10.0    # exp: 10.0
# params_['w_z']        = 10.0    # exp: 10.0
# params_['w_dx']       = 1.0     # exp: 1.0
# params_['w_dy']       = 1.0     # exp: 1.0
# params_['w_dz']       = 1.0     # exp: 1.0
# params_['w_phi']      = 0.1     # exp: 0.1
# params_['w_th']       = 0.1     # exp: 0.1
# params_['w_psi']      = 0.1     # exp: 0.1
# params_['w_ox']       = 1.5     # exp: 3.0
# params_['w_oy']       = 1.5     # exp: 5.0
# params_['w_oz']       = 1.5     # exp: 1.5
# params_['w_u']        = 1.0     # exp: 1.0
# params_['rho']        = 0.1     # exp: 0.1
# params_['gamma']      = 1.0     # exp: 1.0

# cost function flight
params_['Q_mat']          = np.diag([params_['w_x'],params_['w_y'],params_['w_z'],params_['w_psi'],params_['w_th'],params_['w_phi'],params_['w_dx'],params_['w_dy'],params_['w_dz'],params_['w_ox'],params_['w_oy'],params_['w_oz']])
params_['R_mat']          = params_['rho'] * np.diag([params_['w_u'],params_['w_u'],params_['w_u'],params_['w_u']])
params_['Q_mat_terminal'] = params_['gamma'] * params_['Q_mat']

# cost function parameters near ground
params_['w_x_near_ground']        = 0.0   # exp: ?  TBD  
params_['w_y_near_ground']        = 0.0   # exp: ?  TBD
params_['w_z_near_ground']        = 0.0   # exp: ?  TBD
params_['w_dx_near_ground']       = 0.0   # exp: ?  TBD
params_['w_dy_near_ground']       = 0.0   # exp: ?  TBD
params_['w_dz_near_ground']       = 0.0   # exp: ?  TBD
params_['w_phi_near_ground']      = 0.0   # exp: ?  TBD
params_['w_th_near_ground']       = 0.0   # exp: ?  TBD
params_['w_psi_near_ground']      = 0.0   # exp: ?  TBD  
params_['w_ox_near_ground']       = 3.0   # exp: ?  TBD
params_['w_oy_near_ground']       = 5.0   # exp: ?  TBD
params_['w_oz_near_ground']       = 1.5   # exp: ?  TBD
params_['w_u_near_ground']        = 1.0   # exp: ?  TBD
params_['rho_near_ground']        = 0.1   # exp: ?  TBD
params_['gamma_near_ground']      = 1.0   # exp: ?  TBD

# cost function near ground
params_['Q_mat_near_ground']          = np.diag([params_['w_x_near_ground'],params_['w_y_near_ground'],params_['w_z_near_ground'],params_['w_psi_near_ground'],params_['w_th_near_ground'],params_['w_phi_near_ground'],params_['w_dx_near_ground'],params_['w_dy_near_ground'],params_['w_dz_near_ground'],params_['w_ox_near_ground'],params_['w_oy_near_ground'],params_['w_oz_near_ground']])
params_['R_mat_near_ground']          = params_['rho_near_ground'] * np.diag([params_['w_u_near_ground'],params_['w_u_near_ground'],params_['w_u_near_ground'],params_['w_u_near_ground']])
params_['Q_mat_terminal_near_ground'] = params_['gamma_near_ground'] * params_['Q_mat_near_ground']

# cost function driving
params_['Q_mat_driving']          = np.diag([1.,1.,1.])
params_['R_mat_driving']          = 1e-5 * np.diag([1.,1.])
params_['Q_mat_terminal_driving'] = params_['Q_mat_driving']

# use residual model
params_['use_residual_model']    = False

if params_.get('use_residual_model'):
    # learning imports
    from pathlib import Path
    from l4casadi.realtime import RealTimeL4CasADi  
    import torch, yaml, os
    from morphing_lander.cvae.train import TrainConfigCVAE, TrainConfigMLP
    from morphing_lander.cvae.models import CVAE, MLP, MLPZero, CVAEZero

    def load_trained_cvae(model_path,ninputs,noutputs):
        if model_path is None:
            model = CVAE(
                output_dim=noutputs,
                latent_dim=2,
                cond_dim=ninputs,
                encoder_layers=[32,32],
                decoder_layers=[32,32],
                prior_layers=[32,32],
            )
            model.to('cuda')
        else:
            # Load the config from file
            model_path = Path(model_path)
            config_file = model_path / "config.yaml"
            with open(config_file, "r") as f:
                config_dict = yaml.safe_load(f)
                config = TrainConfigCVAE(**config_dict)

            # Load the latest checkpoint (of all .pth files under model_path)
            model_file = max(model_path.glob("*.pth"), key=os.path.getctime)
            model = CVAE(
                output_dim=config.output_dim,
                latent_dim=config.latent_dim,
                cond_dim=config.cond_dim,
                encoder_layers=config.encoder_layers,
                decoder_layers=config.decoder_layers,
                prior_layers=config.prior_layers,
            )
            model.load_state_dict(torch.load(model_file))
            model.to(config.device)
        model.eval()
        return model

    def load_trained_mlp(model_path):
        # Load the config from file
        config_file = model_path / "config.yaml"
        with open(config_file, "r") as f:
            config_dict = yaml.safe_load(f)
            config = TrainConfigMLP(**config_dict)

        # Load the latest checkpoint (of all .pth files under model_path)
        model_file = max(model_path.glob("*.pth"), key=os.path.getctime)
        model = MLP(
            input_dim=config.input_dim,
            output_dim=config.output_dim,
            hidden_dims=config.hidden_layers,
        )
        model.load_state_dict(torch.load(model_file))
        model.to(config.device)
        model.eval()
        return model

    params_['l4c_residual_model']    = RealTimeL4CasADi(load_trained_cvae(params_.get('learned_model_path'),
                                                                          params_.get('model_ninputs'),
                                                                          params_.get('model_noutputs')), 
                                                        approximation_order=1,
                                                        device=params_.get('device'))
else:
    params_['l4c_residual_model']    = None

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
