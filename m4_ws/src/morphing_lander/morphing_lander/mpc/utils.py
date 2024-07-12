import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
from morphing_lander.mpc.parameters import params_

wheel_base   = params_.get('wheel_base')
wheel_radius = params_.get('wheel_radius')

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw    

def quaternion_from_euler(phi,th,psi):
    w = np.cos(phi/2)*np.cos(th/2)*np.cos(psi/2) + np.sin(phi/2)*np.sin(th/2)*np.sin(psi/2)
    x = np.sin(phi/2)*np.cos(th/2)*np.cos(psi/2) - np.cos(phi/2)*np.sin(th/2)*np.sin(psi/2)
    y = np.cos(phi/2)*np.sin(th/2)*np.cos(psi/2) + np.sin(phi/2)*np.cos(th/2)*np.sin(psi/2)
    z = np.cos(phi/2)*np.cos(th/2)*np.sin(psi/2) - np.sin(phi/2)*np.sin(th/2)*np.cos(psi/2)
    return np.array([w,x,y,z])

def u_to_w(acados_ocp_solver,tilt_angle,S_numeric,T_max):

    # body rate control conversion
    u_opt = acados_ocp_solver.get(0, "u")
    x_opt = acados_ocp_solver.get(1, "x")  
    c = (S_numeric(np.zeros(12),tilt_angle).T @ u_opt)[2]/T_max
    omega_d = [x_opt[9],x_opt[10],x_opt[11]]
    w_opt = omega_d + [c]
    return w_opt
    
def create_interpolators(t_vec, x_vec, u_vec):
    # Create interpolation functions for each column in x_vec and u_vec
    x_interpolators = [interp1d(t_vec, x_vec[:, i], kind='linear', fill_value='extrapolate') for i in range(x_vec.shape[1])]
    u_interpolators = [interp1d(t_vec, u_vec[:, i], kind='linear', fill_value='extrapolate') for i in range(u_vec.shape[1])]
    
    return x_interpolators, u_interpolators

def interpolate_values(x_interpolators, u_interpolators, t_new):
    # Interpolate values for the new time value
    x_new = np.array([interp(t_new) for interp in x_interpolators])
    u_new = np.array([interp(t_new) for interp in u_interpolators])
    
    return x_new, u_new

def drive_mixer(drive_speed,turn_speed):
    R,l = wheel_radius,wheel_base
    u_right = 1/R * (drive_speed + l*turn_speed)
    u_left  = 1/R * (drive_speed - l*turn_speed)
    return u_left, u_right

def theta_fit(varphi):
    # theta as a function of varphi fit from matlab kinematics script
    return -0.5988*varphi**4 + 1.55*varphi**3 - 1.69*varphi**2 + 0.3304*varphi + 1.439

