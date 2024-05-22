from os import getenv
from numpy import pi,diag,deg2rad
from pathlib import Path
from l4casadi import L4CasADi

import torch, yaml, os
from morphing_lander.cvae.train import TrainConfig
from morphing_lander.cvae.models import CVAE

def load_trained_cvae(model_path):
    # Load the config from file
    config_file = model_path / "config.yaml"
    with open(config_file, "r") as f:
        config_dict = yaml.safe_load(f)
        config = TrainConfig(**config_dict)

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

class MLPZero(torch.nn.Module):
    def __init__(self):
        super().__init__()

        self.input_layer = torch.nn.Linear(11, 512)

        hidden_layers = []
        for i in range(2):
            hidden_layers.append(torch.nn.Linear(512, 512))

        self.hidden_layer = torch.nn.ModuleList(hidden_layers)
        self.out_layer = torch.nn.Linear(512, 12)

        # Model is not trained -- setting output to zero
        with torch.no_grad():
            self.out_layer.bias.fill_(0.)
            self.out_layer.weight.fill_(0.)

    def forward(self, x):
        x = self.input_layer(x)
        for layer in self.hidden_layer:
            x = torch.tanh(layer(x))
        x = self.out_layer(x)
        return x

# declare parameter dictionary
params_ = {}

# high level parameters
params_['Ts']                 = 0.01                           # control frequency of MPC
params_['Ts_tilt_controller'] = 0.01                           # control frequency of TiltController
params_['queue_size']         = 1                              # queue size of ros2 messages
params_['warmup_time']        = 1.0                            # time after which mpc controller is started (seconds)

# acados mpc solver and integrator paths
params_['acados_ocp_path'] = getenv("HOME") +'/m4v2-code/m4_home/m4_ws/src/morphing_lander/morphing_lander/acados_models/'
params_['acados_sim_path'] = getenv("HOME") +'/m4v2-code/m4_home/m4_ws/src/morphing_lander/morphing_lander/acados_sims/'

# gazebo real time factor
params_['real_time_factor'] = 1.0

# max velocities
params_['max_dx'] = 0.5
params_['max_dy'] = 0.5
params_['max_dz'] = 0.5

# learned model path
params_['use_residual_model']  = False
# learned_model = CVAE(output_dim=12,latent_dim=8,cond_dim=11,encoder_layers=[32,32],decoder_layers=[32,32],prior_layers=[32,32])
learned_model = load_trained_cvae(Path('/home/m4pc/m4v2-code/m4_ws/src/morphing_lander/learned_models/2024-05-15_17-38-22'))
# learned_model = MLPZero()
l4c_residual_model = L4CasADi(learned_model, model_expects_batch_dim=True, device='cpu')  # device='cuda' for GPU
params_['l4c_residual_model']  = l4c_residual_model

# rc inputs
params_['min']  = 1094
params_['max']  = 1934
params_['dead'] = 1514

# safety parameters
params_['max_tilt_in_flight'] = deg2rad(50)
params_['max_tilt_on_land'] = deg2rad(85)

# ground detector parameters
# params_['land_height'] = -0.40                        # height at which we consider robot landed
params_['land_height'] = 0.0                            # height at which we consider robot landed

# emergency parameters
params_['emergency_descent_velocity'] = 0.3             # emergency descent velocity if in strange scenario

# mpc parameters
params_['N_horizon']  = 10
params_['T_horizon']  = 1.0

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

# cost function parameters
params_['w_x']   = 10.0
params_['w_y']   = 10.0
params_['w_z']   = 10.0
params_['w_dx']  = 1.0
params_['w_dy']  = 1.0
params_['w_dz']  = 1.0
params_['w_phi'] = 0.1
params_['w_th']  = 0.1
params_['w_psi'] = 0.1
params_['w_ox']  = 1.5
params_['w_oy']  = 1.5
params_['w_oz']  = 1.5
params_['w_u']   = 1.0

params_['rho'] = 0.1
params_['gamma'] = 1.0

# cost function
params_['Q_mat'] = diag([params_['w_x'],params_['w_y'],params_['w_z'],params_['w_psi'],params_['w_th'],params_['w_phi'],params_['w_dx'],params_['w_dy'],params_['w_dz'],params_['w_ox'],params_['w_oy'],params_['w_oz']])
params_['R_mat'] = params_['rho'] * diag([params_['w_u'],params_['w_u'],params_['w_u'],params_['w_u']])
params_['Q_mat_terminal'] = params_['gamma'] * params_['Q_mat']

# lqr terminal cost

# from control import lqr, ctrb


# phi = 0.0
# Ac = A(X_ref,U_ref/cos(phi),phi)
# Bc = B(X_ref,U_ref/cos(phi),phi)
# K,P,_ = lqr(Ac,Bc,Q_mat,R_mat)

# Q_mat_terminal = gamma * P 
