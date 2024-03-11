import numpy as np 
# Acados
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, sin, cos, inv

def M_(q,phi):
    x_B = q[0]
    y_B = q[1]
    z_B = q[2]
    theta_Bz = q[3]
    theta_By = q[4]
    theta_Bx = q[5]

    out = SX(
        np.array([
[                                                                                                                             4.688,                                                                                                                             0,                                                                               0, -0.00004*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By))*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0),                                                                               -0.00004*cos(theta_By)*cos(theta_Bz)*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0),                                                                                                                                                             0],
[                                                                                                                                 0,                                                                                                                         4.688,                                                                               0,      0.00004*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz))*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0),                                                                               -0.00004*cos(theta_By)*sin(theta_Bz)*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0),                                                                                                                                                             0],
[                                                                                                                                 0,                                                                                                                             0,                                                                           4.688,                                                    0.00004*cos(theta_By)*sin(theta_Bx)*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0),                                                                                              0.00004*sin(theta_By)*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0),                                                                                                                                                             0],
[-0.00004*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By))*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0), 0.00004*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz))*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0), 0.00004*cos(theta_By)*sin(theta_Bx)*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0),                                                      0.043917599999999999999579980078379 - 0.0033016*sin(phi) - 0.0485688*cos(phi),                                                                                                                                                              0,                                                                                                                                                             0],
[                                                  -0.00004*cos(theta_By)*cos(theta_Bz)*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0),                                              -0.00004*cos(theta_By)*sin(theta_Bz)*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0),               0.00004*sin(theta_By)*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0),                                                                                                                                  0, 0.0067336*sin(phi) - 0.0014336*cos(phi) - 0.0004704*cos(phi)*sin(phi) - 0.00048000000000000000194007136744556*cos(phi)**2 + 0.091682800000000000001520051445825,                                                                                                                                                             0],
[                                                                                                                                 0,                                                                                                                             0,                                                                               0,                                                                                                                                  0,                                                                                                                                                              0, 0.0004704*cos(phi)*sin(phi) - 0.0100352*sin(phi) - 0.0471352*cos(phi) - 0.00048000000000000000194007136744556*sin(phi)**2 + 0.10439320000000000000152005144582]        ])
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
 0.00004*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0)*(omega_Bx**2*sin(theta_Bx)*sin(theta_Bz) + omega_By**2*sin(theta_Bx)*sin(theta_Bz) + omega_Bx**2*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) + omega_By**2*cos(theta_Bx)*cos(theta_Bz)*sin(theta_By) - 1.0*omega_Bx*omega_Bz*cos(theta_By)*cos(theta_Bz) + omega_By*omega_Bz*cos(theta_Bx)*sin(theta_Bz) - 1.0*omega_By*omega_Bz*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)),
-0.00004*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0)*(omega_Bx**2*cos(theta_Bz)*sin(theta_Bx) + omega_By**2*cos(theta_Bz)*sin(theta_Bx) - 1.0*omega_Bx**2*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) - 1.0*omega_By**2*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz) + omega_By*omega_Bz*cos(theta_Bx)*cos(theta_Bz) + omega_Bx*omega_Bz*cos(theta_By)*sin(theta_Bz) + omega_By*omega_Bz*sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)),
                                                                                                                                                                                               0.00004*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0)*(cos(theta_Bx)*cos(theta_By)*omega_Bx**2 + omega_Bz*sin(theta_By)*omega_Bx + cos(theta_Bx)*cos(theta_By)*omega_By**2 - 1.0*omega_Bz*cos(theta_By)*sin(theta_Bx)*omega_By),
                                                                                                                                              0.000000000000000000000000043368086899420177360298112034798*omega_By*omega_Bz*(21693371030682432700416.0*cos(phi)*sin(phi) - 386662202529025911422976.0*sin(phi) - 1053807148698805555167232.0*cos(phi) + 22136092888451462028670.0*cos(phi)**2 + 282013823398871625060673.0),
                                                                                                                                              -0.000000000000000000000000043368086899420177360298112034798*omega_Bx*omega_Bz*(33056565380087516495872.0*cos(phi) - 155266244868413295951872.0*sin(phi) + 10846685515341216350208.0*cos(phi)*sin(phi) + 11068046444225731014335.0*cos(phi)**2 + 1383404348435810968666112.0),
                                                                                                                                             0.000000000000000000000000043368086899420177360298112034798*omega_Bx*omega_By*(1086863714078893071663104.0*cos(phi) + 231395957660612615471104.0*sin(phi) - 10846685515341216350208.0*cos(phi)*sin(phi) - 11068046444225731014335.0*cos(phi)**2 + 1101390525036939343605439.0)        ])
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
                                                                         -45.98928,
-0.0003924*cos(theta_By)*sin(theta_Bx)*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0),
              -0.0003924*sin(theta_By)*(8417.0*sin(phi) - 1792.0*cos(phi) + 662.0),
                                                                                 0        
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
[- 29.0*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)) - 29.0*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)), 29.0*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)) + 29.0*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)),     -29.0*cos(phi + theta_Bx)*cos(theta_By), 0.29*sin(phi) - 2.03*cos(phi) - 4.06, 4.93*cos(phi) - 2.61*sin(phi),   2.61*cos(phi) + 4.93*sin(phi)],
[  29.0*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)) - 29.0*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)), 29.0*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)) - 29.0*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)), -29.0*cos(phi - 1.0*theta_Bx)*cos(theta_By), 2.03*cos(phi) - 0.29*sin(phi) + 4.06, 2.61*sin(phi) - 4.93*cos(phi),   2.61*cos(phi) + 4.93*sin(phi)],
[  29.0*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)) - 29.0*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)), 29.0*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)) - 29.0*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)), -29.0*cos(phi - 1.0*theta_Bx)*cos(theta_By), 2.03*cos(phi) - 0.29*sin(phi) + 4.06, 4.93*cos(phi) - 2.61*sin(phi), - 2.61*cos(phi) - 4.93*sin(phi)],
[- 29.0*sin(phi)*(cos(theta_Bx)*sin(theta_Bz) - 1.0*cos(theta_Bz)*sin(theta_Bx)*sin(theta_By)) - 29.0*cos(phi)*(sin(theta_Bx)*sin(theta_Bz) + cos(theta_Bx)*cos(theta_Bz)*sin(theta_By)), 29.0*sin(phi)*(cos(theta_Bx)*cos(theta_Bz) + sin(theta_Bx)*sin(theta_By)*sin(theta_Bz)) + 29.0*cos(phi)*(cos(theta_Bz)*sin(theta_Bx) - 1.0*cos(theta_Bx)*sin(theta_By)*sin(theta_Bz)),     -29.0*cos(phi + theta_Bx)*cos(theta_By), 0.29*sin(phi) - 2.03*cos(phi) - 4.06, 2.61*sin(phi) - 4.93*cos(phi), - 2.61*cos(phi) - 4.93*sin(phi)]
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
    du = Minv @ (S_x.T @ U - b_x - g_x)

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
