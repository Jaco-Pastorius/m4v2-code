from numpy import pi,diag,deg2rad

# declare parameter dictionary
params_ = {}

# high level parameters
params_['Ts']                 = 0.01                           # control frequency of MPC
params_['Ts_tilt_controller'] = 0.01                           # control frequency of TiltController
params_['queue_size']         = 1                              # queue size of ros2 messages
params_['warmup_time']        = 1.0                            # time after which mpc controller is started (seconds)

# rc inputs
params_['min']  = 1094
params_['max']  = 1934
params_['dead'] = 1514

# safety parameters
params_['max_tilt_in_flight'] = deg2rad(50)
params_['max_tilt_on_land'] = deg2rad(85)

# ground detector parameters
params_['land_height'] = -0.40                          # height at which we consider robot landed

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
