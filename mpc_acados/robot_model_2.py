import numpy as np 
# Acados
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import scipy.linalg
from casadi import SX, MX, DM, vertcat, sin, cos, cross, inv
from numpy import array
from IPython import embed
import time 

# control allocation matrix
M_alloc = array([[-6.09,  6.09,  6.09, -6.09],
                 [4.64,  -4.64, 4.64,  -4.64],
                 [2.61, 2.61,  -2.61,  -2.61],
                 [-29.  , -29.  , -29.  , -29.]])

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
