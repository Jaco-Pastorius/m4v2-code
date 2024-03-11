import os
import matplotlib.pyplot as plt
import numpy as np
from acados_template import latexify_plot
from casadi import SX, MX, DM, vertcat, sin, cos, cross
from numpy import array

w = 0.0775
d = 0.1325
l = 0.16
m = 3.5
g = 9.81
varphi = 0.0

def plot_robot(
    shooting_nodes,
    u_max,
    U,
    X_true,
    X_est=None,
    Y_measured=None,
    latexify=False,
    plt_show=True,
    X_true_label=None,
):
    """
    Params:
        shooting_nodes: time values of the discretization
        u_max: maximum absolute value of u
        U: arrray with shape (N_sim-1, nu) or (N_sim, nu)
        X_true: arrray with shape (N_sim, nx)
        X_est: arrray with shape (N_sim-N_mhe, nx)
        Y_measured: array with shape (N_sim, ny)
        latexify: latex style plots
    """

    if latexify:
        latexify_plot()

    WITH_ESTIMATION = X_est is not None and Y_measured is not None

    N_sim = X_true.shape[0]
    nx = X_true.shape[1]
    nu = U.shape[1]

    Tf = shooting_nodes[N_sim - 1]
    t = shooting_nodes

    Ts = t[1] - t[0]
    if WITH_ESTIMATION:
        N_mhe = N_sim - X_est.shape[0]
        t_mhe = np.linspace(N_mhe * Ts, Tf, N_sim - N_mhe)

    control_lables = ["$\tau_x$", "$\tau_y","$\tau_z$", "$T$"]

    for j in range(U.shape[0]):
        utemp = array([[1,1,1,1],[-1,1,-1,1],[-1,-1,1,1],[1,-1,-1,1]]) @ U[j,:]
        U[j,:] = array([[0,w+d*cos(varphi),0,0],[0,0,l*cos(varphi),0],[0,0,0,l*sin(varphi)],[1,0,0,0]]) @ utemp

    for i in range(nu):
        plt.subplot(nx + nu, 1, i+1)
        (line,) = plt.step(t, np.append([U[0, i]], U[:, i]))
        if X_true_label is not None:
            line.set_label(X_true_label)
        else:
            line.set_color("r")
        # plt.title('closed-loop simulation')
        plt.ylabel(control_lables[i])
        plt.xlabel("$t$")
        if u_max[i] is not None:
            plt.hlines(u_max[i], t[0], t[-1], linestyles="dashed", alpha=0.7)
            plt.hlines(-u_max[i], t[0], t[-1], linestyles="dashed", alpha=0.7)
            plt.ylim([-1.2 * u_max[i], 1.2 * u_max[i]])
        plt.grid()

    states_lables = ["x","y","$z$","dx","dy", "$dz$", "$phi$", "$th$", "$psi$","$ox$", "$oy$", "$oz$"]
    for i in range(nx):
        plt.subplot(nx + nu, 1, i + nu+1)
        (line,) = plt.plot(t, X_true[:, i], label="true")
        if X_true_label is not None:
            line.set_label(X_true_label)

        if WITH_ESTIMATION:
            plt.plot(t_mhe, X_est[:, i], "--", label="estimated")
            plt.plot(t, Y_measured[:, i], "x", label="measured")

        plt.ylabel(states_lables[i])
        plt.xlabel("$t$")
        plt.grid()
        plt.legend(loc=1)

    plt.subplots_adjust(left=None, bottom=None, right=None, top=None, hspace=0.4)

    if plt_show:
        plt.show()