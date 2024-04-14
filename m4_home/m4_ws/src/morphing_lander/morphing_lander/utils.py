from numpy import arctan2,arcsin,cos,sin,array,zeros

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
    roll = arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw    

def quaternion_from_euler(phi,th,psi):
    w = cos(phi/2)*cos(th/2)*cos(psi/2) + sin(phi/2)*sin(th/2)*sin(psi/2)
    x = sin(phi/2)*cos(th/2)*cos(psi/2) - cos(phi/2)*sin(th/2)*sin(psi/2)
    y = cos(phi/2)*sin(th/2)*cos(psi/2) + sin(phi/2)*cos(th/2)*sin(psi/2)
    z = cos(phi/2)*cos(th/2)*sin(psi/2) - sin(phi/2)*sin(th/2)*cos(psi/2)
    return array([w,x,y,z])

def u_to_w(acados_ocp_solver,tilt_angle,S_numeric,T_max):
    # body rate control conversion
    u_opt = acados_ocp_solver.get(0, "u")
    x_opt = acados_ocp_solver.get(1, "x")  
    c = (S_numeric(zeros(12),tilt_angle).T @ u_opt)[2]/T_max
    omega_d = [x_opt[9],x_opt[10],x_opt[11]]
    w_opt = omega_d + [c]
    return w_opt