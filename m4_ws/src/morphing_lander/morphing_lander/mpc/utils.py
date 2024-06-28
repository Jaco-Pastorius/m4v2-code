import numpy as np
from scipy.interpolate import interp1d
# import onnxruntime as ort
from scipy.spatial.transform import Rotation as R
from IPython import embed

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
    u_right = drive_speed + turn_speed
    u_left  = drive_speed - turn_speed
    return u_left, u_right

def theta_fit(varphi):
    # theta as a function of varphi fit from matlab kinematics script
    return -0.5988*varphi**4 + 1.55*varphi**3 - 1.69*varphi**2 + 0.3304*varphi + 1.439

# class ONNXModel:
#    def __init__(self, model_path):
#        self.model = ort.InferenceSession(model_path)

#    def preprocess_obs(self,x_current,phi_current, previous_input_rl):
#         # obs shape should be :  (1, 19)
#         # obs type should be  :  <class 'numpy.ndarray'>
#         pos_transformed        = np.array([x_current[0], x_current[1],-x_current[2]])
#         # quat                   = R.from_euler('zyx', [x_current[3],x_current[4],x_current[5]]).as_quat()
#         quat                   = quaternion_from_euler(x_current[5],x_current[4],x_current[3])
#         quat_transformed       = np.array([quat[1],quat[2],-quat[3],quat[0]])
#         vel_transformed        = np.array([x_current[6],x_current[7],-x_current[8]])
#         rotvel_transformed     = np.array([x_current[9],x_current[10],x_current[11]])
#         x_current_transformed  = np.hstack((pos_transformed,quat_transformed,vel_transformed,rotvel_transformed))
#         obs = np.hstack((x_current_transformed, np.array([phi_current])/(np.pi/2), previous_input_rl))
#         obs = obs[np.newaxis,:]
#         # print obs
#         print(f"obs :  {obs}")
#         return obs.astype(np.float32)
    
#    def predict(self, obs):
#        outputs = self.model.run(
#            None,
#            {"obs": obs},
#        )
#        return outputs
   
#    def postprocess_actions(self, outputs):
#        outputs[0][0][0] = np.clip(outputs[0][0][0], -1, 1)
#        outputs[0][0][1] = np.clip(outputs[0][0][1], -1, 1)
#        outputs[0][0][2] = np.clip(outputs[0][0][2], -1, 1)
#        outputs[0][0][3] = np.clip(outputs[0][0][3], -1, 1)
#        outputs[0][0][4] = np.clip(outputs[0][0][4], -1, 1)
       
#        outputs[0][0][0] = (outputs[0][0][0] + 1) / 2
#        outputs[0][0][1] = (outputs[0][0][1] + 1) / 2
#        outputs[0][0][2] = (outputs[0][0][2] + 1) / 2
#        outputs[0][0][3] = (outputs[0][0][3] + 1) / 2
#        return outputs[0].squeeze()
