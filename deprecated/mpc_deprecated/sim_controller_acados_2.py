# ROS imports
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data

# Message imports
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleThrustSetpoint
from px4_msgs.msg import VehicleTorqueSetpoint
from px4_msgs.msg import VehicleOdometry

# Python imports
import numpy as np
from numpy import array
import scipy.linalg
from IPython import embed
import time 

# Acados/Casadi
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, sin, cos, inv

# reference states
# X0 = np.array([0,0,0,0,0,0,0,0,0,0])  # initial states
# X_ref = np.array([0.0,0.0,1.0,0,0,0,0,0,0,0])  # reference states
X0 = np.array([0,0,0,0,0,0,0,0,0,0,0,0])  # initial states
X_ref = np.array([0.0,0.0,1.0,0,0,0,0,0,0,0,0,0])  # reference states
U_ref = np.array([0.5,0.5,0.5,0.5]) # reference inputs

# cost function
# Q_mat = np.diag([1,1,1,1,1,1,1,1,1,1])
Q_mat = np.diag([1,1,1,1,1,1,1,1,1,1,1,1])
R_mat = 1e-1 * np.diag([1,1,1,1])

# constraint variables
u_max  = 1.0     # Max allowable input
vx_max = 1.0     # Max allowable velocity
vy_max = 1.0     # Max allowable velocity
vz_max = 1.0     # Max allowable velocity

# collocation parametrs
N_horizon = 40   # Define the number of discretization steps
T_horizon = 4.0  # Define the prediction horizon

# # model parameters
# w = 0.0775
# d = 0.1325
# l = 0.16
# m = 4.7
# g = 9.81

# control allocation matrix
M_alloc = array([[-6.09,  6.09,  6.09, -6.09],
                 [4.64,  -4.64, 4.64,  -4.64],
                 [2.61, 2.61,  -2.61,  -2.61],
                 [29.  , 29.  , 29.  , 29.]])

torque_max_x = np.sum(np.abs(M_alloc[0,:]))/2
torque_max_y = np.sum(np.abs(M_alloc[1,:]))/2
torque_max_z = np.sum(np.abs(M_alloc[2,:]))/2
thrust_max   = np.sum(M_alloc[3,:])

def M_(q,phi):
    x_B = q[0]
    y_B = q[1]
    z_B = q[2]
    theta_Bz = q[3]
    theta_By = q[4]
    theta_Bx = q[5]

    out = SX(
        np.array([
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 3.93,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                  0, 0.03219456*cos(theta_Bx)*sin(theta_Bz) + 0.0013047599999999959230478907556972*sin(theta_Bx)*sin(theta_Bz) - 0.3334736*cos(theta_Bx)*sin(phi)*sin(theta_Bz) - 0.03219456*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 0.074402759999999999035447117989861*cos(phi)*cos(theta_Bx)*sin(theta_Bz) + 0.0013047599999999959230478907556972*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 0.074402759999999999035447117989861*cos(phi)*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 0.3334736*cos(theta_Bz)*sin(phi)*sin(theta_Bx)*sin(theta_By), 0.03219456*cos(theta_By)*cos(theta_Bz) - 0.033122039999999999828672603285895*sin(theta_Bx)*sin(theta_Bz) + 0.074402759999999999035447117989861*cos(phi)*cos(theta_By)*cos(theta_Bz) - 0.3334736*cos(theta_By)*cos(theta_Bz)*sin(phi) - 0.033122039999999999828672603285895*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By), 0.033122039999999999828672603285895*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) - 0.033122039999999999828672603285895*cos(theta_Bx)*sin(theta_Bz) - 0.0013047599999999959230478907556972*cos(theta_By)*cos(theta_Bz)],
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  3.93,                                                                                                                                                                                                                                                  0, 0.0013047599999999959230478907556972*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 0.0013047599999999959230478907556972*cos(theta_Bz)*sin(theta_Bx) - 0.03219456*cos(theta_Bx)*cos(theta_Bz) - 0.03219456*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 0.074402759999999999035447117989861*cos(phi)*cos(theta_Bx)*cos(theta_Bz) + 0.3334736*cos(theta_Bx)*cos(theta_Bz)*sin(phi) - 0.074402759999999999035447117989861*cos(phi)*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 0.3334736*sin(phi)*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz), 0.033122039999999999828672603285895*cos(theta_Bz)*sin(theta_Bx) + 0.03219456*cos(theta_By)*sin(theta_Bz) - 0.3334736*cos(theta_By)*sin(phi)*sin(theta_Bz) - 0.033122039999999999828672603285895*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 0.074402759999999999035447117989861*cos(phi)*cos(theta_By)*sin(theta_Bz), 0.033122039999999999828672603285895*cos(theta_Bx)*cos(theta_Bz) - 0.0013047599999999959230478907556972*cos(theta_By)*sin(theta_Bz) + 0.033122039999999999828672603285895*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)],
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                               3.93,                                                                                                                                                                                                                                                                                    0.0000000000000000000011635137298071640543639659881592*cos(theta_By)*(1121396307215250000.0*cos(theta_Bx) - 27670116110564327424.0*sin(theta_Bx) - 63946611109033671875.0*cos(phi)*sin(theta_Bx) + 286609080285858365440.0*sin(phi)*sin(theta_Bx)),                                                                                                                           0.3334736*sin(phi)*sin(theta_By) - 0.033122039999999999828672603285895*cos(theta_Bx)*cos(theta_By) - 0.074402759999999999035447117989861*cos(phi)*sin(theta_By) - 0.03219456*sin(theta_By),                                                                                               0.0013047599999999959230478907556972*sin(theta_By) + 0.033122039999999999828672603285895*cos(theta_By)*sin(theta_Bx)],
[0.03219456*cos(theta_Bx)*sin(theta_Bz) + 0.0013047599999999959230478907556972*sin(theta_Bx)*sin(theta_Bz) - 0.3334736*cos(theta_Bx)*sin(phi)*sin(theta_Bz) - 0.03219456*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 0.074402759999999999035447117989861*cos(phi)*cos(theta_Bx)*sin(theta_Bz) + 0.0013047599999999959230478907556972*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 0.074402759999999999035447117989861*cos(phi)*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 0.3334736*cos(theta_Bz)*sin(phi)*sin(theta_Bx)*sin(theta_By), 0.0013047599999999959230478907556972*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 0.0013047599999999959230478907556972*cos(theta_Bz)*sin(theta_Bx) - 0.03219456*cos(theta_Bx)*cos(theta_Bz) - 0.03219456*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 0.074402759999999999035447117989861*cos(phi)*cos(theta_Bx)*cos(theta_Bz) + 0.3334736*cos(theta_Bx)*cos(theta_Bz)*sin(phi) - 0.074402759999999999035447117989861*cos(phi)*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 0.3334736*sin(phi)*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz), 0.0000000000000000000011635137298071640543639659881592*cos(theta_By)*(1121396307215250000.0*cos(theta_Bx) - 27670116110564327424.0*sin(theta_Bx) - 63946611109033671875.0*cos(phi)*sin(theta_Bx) + 286609080285858365440.0*sin(phi)*sin(theta_Bx)),                                                                                                                                                                                                                                                                                                                                                                                                     0.047180931829760003234319754028547*cos(phi) + 0.0019233281664000005957329918260257*sin(phi) + 0.04251113652376000173655589549071,                                                                                                                                                                                                                                                                              -0.000016494775919999948373850391369899,                                                                              0.0041490785311999999781934489462287*sin(phi) - 0.0009257191399199999831336776301427*cos(phi) - 0.00040700362751999999789472894917708],
[                                                                                                                                                                                                                 0.03219456*cos(theta_By)*cos(theta_Bz) - 0.033122039999999999828672603285895*sin(theta_Bx)*sin(theta_Bz) + 0.074402759999999999035447117989861*cos(phi)*cos(theta_By)*cos(theta_Bz) - 0.3334736*cos(theta_By)*cos(theta_Bz)*sin(phi) - 0.033122039999999999828672603285895*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By),                                                                                                                                                                                                                  0.033122039999999999828672603285895*cos(theta_Bz)*sin(theta_Bx) + 0.03219456*cos(theta_By)*sin(theta_Bz) - 0.3334736*cos(theta_By)*sin(phi)*sin(theta_Bz) - 0.033122039999999999828672603285895*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 0.074402759999999999035447117989861*cos(phi)*cos(theta_By)*sin(theta_Bz),                                                         0.3334736*sin(phi)*sin(theta_By) - 0.033122039999999999828672603285895*cos(theta_Bx)*cos(theta_By) - 0.074402759999999999035447117989861*cos(phi)*sin(theta_By) - 0.03219456*sin(theta_By),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               -0.000016494775919999948373850391369899,                                                                                                                                                                                                       0.0018285222297599999762951483717188*cos(phi) - 0.0081954471936*sin(phi) + 0.090814230782959999995668158101481,                                                                           0.00016606985279999948108553553538513*sin(phi) - 0.00003705257447999988374236666643924*cos(phi) - 0.000016032890879999949902412481606007],
[                                                                                                                                                                                                                                                                                                                   0.033122039999999999828672603285895*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) - 0.033122039999999999828672603285895*cos(theta_Bx)*sin(theta_Bz) - 0.0013047599999999959230478907556972*cos(theta_By)*cos(theta_Bz),                                                                                                                                                                                                                                                                                                                    0.033122039999999999828672603285895*cos(theta_Bx)*cos(theta_Bz) - 0.0013047599999999959230478907556972*cos(theta_By)*sin(theta_Bz) + 0.033122039999999999828672603285895*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz),                                                                                                                               0.0013047599999999959230478907556972*sin(theta_By) + 0.033122039999999999828672603285895*cos(theta_By)*sin(theta_Bx),                                                                                                                                                                                                                                                                                                                                                                                                 0.0041490785311999999781934489462287*sin(phi) - 0.0009257191399199999831336776301427*cos(phi) - 0.00040700362751999999789472894917708,                                                                                                                                                                             0.00016606985279999948108553553538513*sin(phi) - 0.00003705257447999988374236666643924*cos(phi) - 0.000016032890879999949902412481606007,                                                                                   0.045352409600000003258024605656828*cos(phi) + 0.010118775360000000595732991826026*sin(phi) + 0.10253415380016000173222405359219]        ])
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
0.0013047599999999959230478907556972*omega_Bx**2*cos(theta_Bx)*sin(theta_Bz) - 0.033122039999999999828672603285895*omega_Bz**2*cos(theta_By)*cos(theta_Bz) - 0.033122039999999999828672603285895*omega_By**2*cos(theta_By)*cos(theta_Bz) + 0.0013047599999999959230478907556972*omega_Bz**2*cos(theta_Bx)*sin(theta_Bz) - 0.03219456*omega_Bx**2*sin(theta_Bx)*sin(theta_Bz) - 0.03219456*omega_By**2*sin(theta_Bx)*sin(theta_Bz) + 0.033122039999999999828672603285895*omega_Bx*omega_Bz*sin(theta_Bx)*sin(theta_Bz) + 0.0013047599999999959230478907556972*omega_By*omega_Bz*sin(theta_Bx)*sin(theta_Bz) - 0.03219456*omega_Bx**2*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 0.03219456*omega_By**2*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 0.074402759999999999035447117989861*omega_Bx**2*cos(phi)*sin(theta_Bx)*sin(theta_Bz) - 0.074402759999999999035447117989861*omega_By**2*cos(phi)*sin(theta_Bx)*sin(theta_Bz) - 0.0013047599999999959230478907556972*omega_Bx**2*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) - 0.0013047599999999959230478907556972*omega_Bz**2*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 0.3334736*omega_Bx**2*sin(phi)*sin(theta_Bx)*sin(theta_Bz) + 0.3334736*omega_By**2*sin(phi)*sin(theta_Bx)*sin(theta_Bz) + 0.0013047599999999959230478907556972*omega_Bx*omega_By*cos(theta_By)*cos(theta_Bz) + 0.03219456*omega_Bx*omega_Bz*cos(theta_By)*cos(theta_Bz) - 0.033122039999999999828672603285895*omega_Bx*omega_By*cos(theta_Bx)*sin(theta_Bz) - 0.03219456*omega_By*omega_Bz*cos(theta_Bx)*sin(theta_Bz) - 0.074402759999999999035447117989861*omega_Bx**2*cos(phi)*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 0.074402759999999999035447117989861*omega_By**2*cos(phi)*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) + 0.3334736*omega_Bx**2*cos(theta_Bx)*cos(theta_Bz)*sin(phi)*sin(theta_By) + 0.3334736*omega_By**2*cos(theta_Bx)*cos(theta_Bz)*sin(phi)*sin(theta_By) + 0.074402759999999999035447117989861*omega_Bx*omega_Bz*cos(phi)*cos(theta_By)*cos(theta_Bz) - 0.3334736*omega_Bx*omega_Bz*cos(theta_By)*cos(theta_Bz)*sin(phi) - 0.074402759999999999035447117989861*omega_By*omega_Bz*cos(phi)*cos(theta_Bx)*sin(theta_Bz) + 0.033122039999999999828672603285895*omega_Bx*omega_Bz*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) + 0.0013047599999999959230478907556972*omega_By*omega_Bz*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) + 0.3334736*omega_By*omega_Bz*cos(theta_Bx)*sin(phi)*sin(theta_Bz) + 0.033122039999999999828672603285895*omega_Bx*omega_By*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 0.03219456*omega_By*omega_Bz*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) + 0.074402759999999999035447117989861*omega_By*omega_Bz*cos(phi)*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By) - 0.3334736*omega_By*omega_Bz*cos(theta_Bz)*sin(phi)*sin(theta_Bx)*sin(theta_By),
0.03219456*omega_Bx**2*cos(theta_Bz)*sin(theta_Bx) - 0.0013047599999999959230478907556972*omega_Bz**2*cos(theta_Bx)*cos(theta_Bz) - 0.0013047599999999959230478907556972*omega_Bx**2*cos(theta_Bx)*cos(theta_Bz) + 0.03219456*omega_By**2*cos(theta_Bz)*sin(theta_Bx) - 0.033122039999999999828672603285895*omega_By**2*cos(theta_By)*sin(theta_Bz) - 0.033122039999999999828672603285895*omega_Bz**2*cos(theta_By)*sin(theta_Bz) + 0.074402759999999999035447117989861*omega_Bx**2*cos(phi)*cos(theta_Bz)*sin(theta_Bx) + 0.074402759999999999035447117989861*omega_By**2*cos(phi)*cos(theta_Bz)*sin(theta_Bx) - 0.3334736*omega_Bx**2*cos(theta_Bz)*sin(phi)*sin(theta_Bx) - 0.3334736*omega_By**2*cos(theta_Bz)*sin(phi)*sin(theta_Bx) - 0.03219456*omega_Bx**2*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 0.03219456*omega_By**2*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 0.0013047599999999959230478907556972*omega_Bx**2*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 0.0013047599999999959230478907556972*omega_Bz**2*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 0.033122039999999999828672603285895*omega_Bx*omega_By*cos(theta_Bx)*cos(theta_Bz) + 0.03219456*omega_By*omega_Bz*cos(theta_Bx)*cos(theta_Bz) + 0.0013047599999999959230478907556972*omega_Bx*omega_By*cos(theta_By)*sin(theta_Bz) - 0.033122039999999999828672603285895*omega_Bx*omega_Bz*cos(theta_Bz)*sin(theta_Bx) + 0.03219456*omega_Bx*omega_Bz*cos(theta_By)*sin(theta_Bz) - 0.0013047599999999959230478907556972*omega_By*omega_Bz*cos(theta_Bz)*sin(theta_Bx) - 0.074402759999999999035447117989861*omega_Bx**2*cos(phi)*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 0.074402759999999999035447117989861*omega_By**2*cos(phi)*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 0.3334736*omega_Bx**2*cos(theta_Bx)*sin(phi)*sin(theta_By)*sin(theta_Bz) + 0.3334736*omega_By**2*cos(theta_Bx)*sin(phi)*sin(theta_By)*sin(theta_Bz) + 0.074402759999999999035447117989861*omega_By*omega_Bz*cos(phi)*cos(theta_Bx)*cos(theta_Bz) + 0.074402759999999999035447117989861*omega_Bx*omega_Bz*cos(phi)*cos(theta_By)*sin(theta_Bz) - 0.3334736*omega_By*omega_Bz*cos(theta_Bx)*cos(theta_Bz)*sin(phi) - 0.3334736*omega_Bx*omega_Bz*cos(theta_By)*sin(phi)*sin(theta_Bz) + 0.033122039999999999828672603285895*omega_Bx*omega_Bz*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 0.0013047599999999959230478907556972*omega_By*omega_Bz*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 0.033122039999999999828672603285895*omega_Bx*omega_By*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 0.03219456*omega_By*omega_Bz*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) + 0.074402759999999999035447117989861*omega_By*omega_Bz*cos(phi)*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 0.3334736*omega_By*omega_Bz*sin(phi)*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0.033122039999999999828672603285895*omega_By**2*sin(theta_By) + 0.033122039999999999828672603285895*omega_Bz**2*sin(theta_By) - 0.0013047599999999959230478907556972*omega_Bx*omega_By*sin(theta_By) - 0.03219456*omega_Bx*omega_Bz*sin(theta_By) - 0.03219456*omega_Bx**2*cos(theta_Bx)*cos(theta_By) - 0.03219456*omega_By**2*cos(theta_Bx)*cos(theta_By) - 0.0013047599999999959230478907556972*omega_Bx**2*cos(theta_By)*sin(theta_Bx) - 0.0013047599999999959230478907556972*omega_Bz**2*cos(theta_By)*sin(theta_Bx) + 0.3334736*omega_Bx*omega_Bz*sin(phi)*sin(theta_By) - 0.074402759999999999035447117989861*omega_Bx**2*cos(phi)*cos(theta_Bx)*cos(theta_By) - 0.074402759999999999035447117989861*omega_By**2*cos(phi)*cos(theta_Bx)*cos(theta_By) + 0.3334736*omega_Bx**2*cos(theta_Bx)*cos(theta_By)*sin(phi) + 0.3334736*omega_By**2*cos(theta_Bx)*cos(theta_By)*sin(phi) + 0.033122039999999999828672603285895*omega_Bx*omega_Bz*cos(theta_Bx)*cos(theta_By) + 0.0013047599999999959230478907556972*omega_By*omega_Bz*cos(theta_Bx)*cos(theta_By) - 0.074402759999999999035447117989861*omega_Bx*omega_Bz*cos(phi)*sin(theta_By) + 0.033122039999999999828672603285895*omega_Bx*omega_By*cos(theta_By)*sin(theta_Bx) + 0.03219456*omega_By*omega_Bz*cos(theta_By)*sin(theta_Bx) + 0.074402759999999999035447117989861*omega_By*omega_Bz*cos(phi)*cos(theta_By)*sin(theta_Bx) - 0.3334736*omega_By*omega_Bz*cos(theta_By)*sin(phi)*sin(theta_Bx),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     0.00003705257447999988374236666643924*omega_Bz**2*cos(phi) - 0.00003705257447999988374236666643924*omega_By**2*cos(phi) + 0.00016606985279999948108553553538513*omega_By**2*sin(phi) - 0.00016606985279999948108553553538513*omega_Bz**2*sin(phi) - 0.00040700362751999999789472894917708*omega_Bx*omega_By + 0.000016494775919999948373850391369899*omega_Bx*omega_Bz + 0.01171992301720000173655589549071*omega_By*omega_Bz - 0.000016032890879999949902412481606007*omega_By**2 + 0.000016032890879999949902412481606007*omega_Bz**2 - 0.0009257191399199999831336776301427*omega_Bx*omega_By*cos(phi) + 0.04352388737024000328172945728511*omega_By*omega_Bz*cos(phi) + 0.0041490785311999999781934489462287*omega_Bx*omega_By*sin(phi) + 0.018314222553600000595732991826026*omega_By*omega_Bz*sin(phi),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0.0009257191399199999831336776301427*omega_Bx**2*cos(phi) - 0.0009257191399199999831336776301427*omega_Bz**2*cos(phi) - 0.0041490785311999999781934489462287*omega_Bx**2*sin(phi) + 0.0041490785311999999781934489462287*omega_Bz**2*sin(phi) + 0.000016032890879999949902412481606007*omega_Bx*omega_By - 0.060023017276399999995668158101481*omega_Bx*omega_Bz - 0.000016494775919999948373850391369899*omega_By*omega_Bz + 0.00040700362751999999789472894917708*omega_Bx**2 - 0.00040700362751999999789472894917708*omega_Bz**2 + 0.00003705257447999988374236666643924*omega_Bx*omega_By*cos(phi) + 0.0018285222297599999762951483717188*omega_Bx*omega_Bz*cos(phi) - 0.00016606985279999948108553553538513*omega_Bx*omega_By*sin(phi) - 0.0081954471936*omega_Bx*omega_Bz*sin(phi),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              0.04830309425919999825911226261077*omega_Bx*omega_By - 0.000016032890879999949902412481606007*omega_Bx*omega_Bz + 0.00040700362751999999789472894917708*omega_By*omega_Bz - 0.000016494775919999948373850391369899*omega_Bx**2 + 0.000016494775919999948373850391369899*omega_By**2 - 0.045352409600000003258024605656828*omega_Bx*omega_By*cos(phi) - 0.00003705257447999988374236666643924*omega_Bx*omega_Bz*cos(phi) + 0.0009257191399199999831336776301427*omega_By*omega_Bz*cos(phi) - 0.010118775360000000595732991826026*omega_Bx*omega_By*sin(phi) + 0.00016606985279999948108553553538513*omega_Bx*omega_Bz*sin(phi) - 0.0041490785311999999781934489462287*omega_By*omega_Bz*sin(phi)        ])
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
                                                                                                                                                                                                                                          38.5533,
0.000000000000000000011414069689408279373310506343842*cos(theta_By)*(1121396307215250000.0*cos(theta_Bx) - 27670116110564327424.0*sin(theta_Bx) - 63946611109033671875.0*cos(phi)*sin(theta_Bx) + 286609080285858365440.0*sin(phi)*sin(theta_Bx)),
                                                     3.271376016*sin(phi)*sin(theta_By) - 0.32492721239999999831927823823463*cos(theta_Bx)*cos(theta_By) - 0.72989107559999999053773622748054*cos(phi)*sin(theta_By) - 0.3158286336*sin(theta_By),
                                                                                                                               0.012799695599999960005099808313389*sin(theta_By) + 0.32492721239999999831927823823463*cos(theta_By)*sin(theta_Bx)
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
[-(400.0*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)))/(53.0*cos(phi) + 31.0), (400.0*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)))/(53.0*cos(phi) + 31.0), (400.0*cos(theta_By)*sin(phi)*sin(theta_Bx))/(53.0*cos(phi) + 31.0), 1.0,   0,   0],
[                                                                                                                     0,                                                                                                                 0,                                                                   0,   0, 1.0,   0],
[                                                                                                                     0,                                                                                                                 0,                                                                   0,   0,   0, 1.0],
[                                    cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)),                       -1.0*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)),                                cos(phi)*cos(theta_Bx)*cos(theta_By),   0,   0,   0]

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

def export_robot_model() -> AcadosModel:
    model_name = "torque_dynamics"

    # parameters
    varphi  = SX.sym("varphi",1)
    # theta_Bz  = SX.sym("theta_Bz",1)
    # omega_Bz  = SX.sym("omega_Bz",1)

    # parameters
    # p = vertcat(varphi,theta_Bz,omega_Bz)
    p = varphi

    # states
    X = SX.sym("X",12)
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

    # controls
    U = SX.sym("U",4)

    # get applied wrench
    wrench = M_alloc @ U

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
    du = Minv @ (S_x.T @ wrench - b_x - g_x)

    chid = F(theta_Bx,theta_By,theta_Bz) @ vertcat(omega_Bx,omega_By,omega_Bz)

    # finally write explicit dynamics
    f_expl = vertcat(dx_B,dy_B,dz_B,du[0],du[1],du[2],chid[2],chid[1],chid[0],du[3],du[4],du[5])

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

    Xdot = vertcat(x_B_dot,y_B_dot,z_B_dot,dx_B_dot,dy_B_dot,dz_B_dot,theta_Bx_dot,theta_By_dot,theta_Bz_dot,omega_Bx_dot,omega_By_dot,omega_Bz_dot)

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

    model_name = "torque_dynamics"

    # parameters
    varphi  = SX.sym("varphi",1)
    theta_Bz  = SX.sym("theta_Bz",1)
    omega_Bz  = SX.sym("omega_Bz",1)

    # parameters
    p = vertcat(varphi,theta_Bz,omega_Bz)
    # p = varphi

    # states
    X = SX.sym("X",10)
    x_B      = X[0]
    y_B      = X[1]
    z_B      = X[2]
    dx_B     = X[3]
    dy_B     = X[4]
    dz_B     = X[5]
    theta_Bx = X[6]
    theta_By = X[7]
    # theta_Bz = X[8]
    omega_Bx = X[8]
    omega_By = X[9]
    # omega_Bz = X[11]

    # controls
    U = SX.sym("U",4)

    # get applied wrench
    wrench = M_alloc @ U

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
    du = Minv @ (S_x.T @ wrench - b_x - g_x)

    chid = F(theta_Bx,theta_By,theta_Bz) @ vertcat(omega_Bx,omega_By,omega_Bz)

    # finally write explicit dynamics
    f_expl = vertcat(dx_B,dy_B,dz_B,du[0],du[1],du[2],chid[2],chid[1],du[3],du[4])
    # f_expl = vertcat(dx_B,dy_B,dz_B,du[0],du[1],du[2],chid[2],chid[1],chid[0],du[3],du[4],du[5])

    # xdot
    x_B_dot = SX.sym("x_B_dot")
    y_B_dot = SX.sym("y_B_dot")
    z_B_dot = SX.sym("z_B_dot")
    dx_B_dot = SX.sym("dx_B_dot")
    dy_B_dot = SX.sym("dy_B_dot")
    dz_B_dot = SX.sym("dz_B_dot")
    theta_Bx_dot = SX.sym("theta_Bx_dot")
    theta_By_dot = SX.sym("theta_By_dot")
    # theta_Bz_dot = SX.sym("theta_Bz_dot")
    omega_Bx_dot = SX.sym("omega_Bx_dot")
    omega_By_dot = SX.sym("omega_By_dot")
    # omega_Bz_dot = SX.sym("omega_Bz_dot")

    Xdot = vertcat(x_B_dot,y_B_dot,z_B_dot,dx_B_dot,dy_B_dot,dz_B_dot,theta_Bx_dot,theta_By_dot,omega_Bx_dot,omega_By_dot)
    # Xdot = vertcat(x_B_dot,y_B_dot,z_B_dot,dx_B_dot,dy_B_dot,dz_B_dot,theta_Bx_dot,theta_By_dot,theta_Bz_dot,omega_Bx_dot,omega_By_dot,omega_Bz_dot)

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
    ocp.constraints.lbu = np.array([0,0,0,0])
    ocp.constraints.ubu = np.array([+1,+1,+1,+1])
    ocp.constraints.idxbu = np.array([0,1,2,3])

    # ocp.constraints.lbx = np.array([-vx_max,-vy_max,-vz_max])
    # ocp.constraints.ubx = np.array([+vx_max,+vy_max,+vz_max])
    # ocp.constraints.idxbx = np.array([3,4,5])    

    # set initial state
    ocp.constraints.x0 = X0

    # set parameters
    # ocp.parameter_values = np.zeros((3,))
    ocp.parameter_values = np.zeros((1,))

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

class OffboardControl(Node):
    def __init__(self):
        super().__init__('OffboardControl')
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.thrust_publisher_   = self.create_publisher(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", 10)
        self.torque_publisher_   = self.create_publisher(VehicleTorqueSetpoint, "/fmu/in/vehicle_torque_setpoint", 10)

        # subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
                                            VehicleOdometry,
                                            '/fmu/out/vehicle_odometry',
                                            self.vehicle_odometry_callback,
                                            qos_profile_sensor_data)
        self.vehicle_odometry_subscriber  # prevent unused variable warning

        # timer callback
        self.Ts = 0.01
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)
        self.offboard_setpoint_counter_ = 0

        # robot state
        self.state = np.zeros(12)

        # robot control input
        self.thrust_body = [0,0,0]
        self.torque      = [0,0,0]

        # acados solver
        self.ocp = create_ocp_solver_description()
        self.acados_ocp_solver = AcadosOcpSolver(
            self.ocp, json_file="acados_ocp_" + self.ocp.model.name + ".json"
        )

        # solver initialization
        for stage in range(N_horizon + 1):
            self.acados_ocp_solver.set(stage, "x", X0)
        for stage in range(N_horizon):
            self.acados_ocp_solver.set(stage, "u", U_ref)

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

            # Arm the vehicle
            self.arm()

        # Offboard_control_mode heartbeat
        self.publish_offboard_control_mode()

        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

    def vehicle_odometry_callback(self, msg):
        # get state from odometry
        p = msg.position
        phi,th,psi = self.euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        v = msg.velocity
        o = msg.angular_velocity
        # self.state = np.array([p[0],p[1],-p[2],v[0],v[1],-v[2],-phi,th,-o[0],o[1]])
        self.state = np.array([p[0],p[1],-p[2],v[0],v[1],-v[2],phi,th,psi,o[0],o[1],o[2]])
        # self.parameters = np.array([0.0,-psi,-o[2]])
        self.parameters = 0.0

        print(f"phi,th,psi: {phi:.2f},{th:.2f},{psi:.2f}".format())
        print(f"x,y,z: {p[0]:.2f},{p[1]:.2f},{p[2]:.2f}".format())

        # run mpc
        self.mpc_update()

    def mpc_update(self):
        start_time = time.process_time()

        # set initial state constraint
        self.acados_ocp_solver.set(0, "lbx", self.state)
        self.acados_ocp_solver.set(0, "ubx", self.state)

        # update yref and parameters
        for j in range(N_horizon):
            yref = np.hstack((X_ref,U_ref))
            self.acados_ocp_solver.set(j, "yref", yref)
            self.acados_ocp_solver.set(j, "p", self.parameters)
        yref_N = X_ref
        self.acados_ocp_solver.set(N_horizon, "yref", yref_N)

        # solve ocp
        status = self.acados_ocp_solver.solve()  

        # get first input
        u_opt = self.acados_ocp_solver.get(0, "u")      

        # transform to wrench
        wrench = M_alloc @ u_opt

        # write wrench to torque and thrust
        self.torque = [wrench[0]/torque_max_x,wrench[1]/torque_max_y,wrench[2]/torque_max_z]
        self.thrust_body = [0.0,0,-wrench[3]/thrust_max]
        print(f"torque: {self.torque}")
        print(f"thrust: {self.thrust_body}")

        # Publish current commanded thrust and torque by MPC controller
        self.publish_vehicle_thrust_and_torque_setpoint(self.torque,self.thrust_body)

        print("* comp time = %5g seconds\n" % (time.process_time() - start_time))

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False  # True for position control
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_vehicle_thrust_and_torque_setpoint(self,torque,thrust):
        msg_thrust = VehicleThrustSetpoint()
        msg_torque = VehicleTorqueSetpoint()

        msg_thrust.xyz[0] = thrust[0]
        msg_thrust.xyz[1] = thrust[1]
        msg_thrust.xyz[2] = thrust[2]

        msg_torque.xyz[0] = torque[0]
        msg_torque.xyz[1] = torque[1]
        msg_torque.xyz[2] = torque[2]

        timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        msg_thrust.timestamp = timestamp
        msg_torque.timestamp = timestamp 
        self.thrust_publisher_.publish(msg_thrust)
        self.torque_publisher_.publish(msg_torque)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
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
    
    def quaternion_from_euler(self,phi,th,psi):
        w = cos(phi/2)*cos(th/2)*cos(psi/2) + sin(phi/2)*sin(th/2)*sin(psi/2)
        x = sin(phi/2)*cos(th/2)*cos(psi/2) - cos(phi/2)*sin(th/2)*sin(psi/2)
        y = cos(phi/2)*sin(th/2)*cos(psi/2) + sin(phi/2)*cos(th/2)*sin(psi/2)
        z = cos(phi/2)*cos(th/2)*sin(psi/2) - sin(phi/2)*sin(th/2)*cos(psi/2)
        return np.array([w,x,y,z])

def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
