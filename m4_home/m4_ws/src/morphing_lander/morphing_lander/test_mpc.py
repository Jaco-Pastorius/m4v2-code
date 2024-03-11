from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosModel
from casadi import SX, vertcat, sin, cos, inv
import numpy as np
import scipy.linalg
import time
import matplotlib.pyplot as plt
from acados_template import latexify_plot
from IPython import embed
from matplotlib.backends.backend_pdf import PdfPages
import os

def multipage(filename, figs=None, dpi=200):
    pp = PdfPages(filename)
    if figs is None:
        figs = [plt.figure(n) for n in plt.get_fignums()]
    for fig in figs:
        fig.savefig(pp, format='pdf')
    pp.close()

from morphing_lander_mpc import *

Nsim = 100

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

    control_lables = [r"$u_1$", r"$u_2$",r"$u_3$", r"$u_4$", r"v"]

    f1 = plt.figure()
    for i in range(nu):
        plt.subplot(nu, 1, i+1)
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
        # plt.grid()


    states_lables = [r"$x$",r"$y$",r"$z$", r"$\psi$", r"$ \theta $", r"$ \phi $", r"$\dot x$",r"$\dot y$", r"$\dot z$",r"$\omega_x$", r"$\omega_y$",r"$\omega_z$",r"φ"]
    f2 = plt.figure(figsize=(10, 6))

    X_true[:,12] = np.rad2deg(X_true[:,12])

    # Plotting states in the first column
    for i, state_index in enumerate([0, 1, 2, 5, 4, 3]):
        plt.subplot(6, 2, i*2 + 1)
        (line,) = plt.plot(t, X_true[:, state_index], label="true")
        if WITH_ESTIMATION:
            plt.plot(t_mhe, X_est[:, state_index], "--", label="estimated")
            plt.plot(t, Y_measured[:, state_index], "x", label="measured")
        plt.ylabel(states_lables[state_index])
        plt.xlabel("$t$")
        plt.grid()

        max_value = np.max(np.abs(X_true[:, state_index]))
        if max_value > 1e-2:
            plt.autoscale(axis='y',tight=True)
        else:
            plt.ylim([-0.1, 0.1])

    # Plotting states in the second column
    for i, state_index in enumerate([6, 7, 8, 9, 10, 11]):
        plt.subplot(6, 2, i*2 + 2)
        (line,) = plt.plot(t, X_true[:, state_index], label="true")
        if WITH_ESTIMATION:
            plt.plot(t_mhe, X_est[:, state_index], "--", label="estimated")
            plt.plot(t, Y_measured[:, state_index], "x", label="measured")
        plt.ylabel(states_lables[state_index])
        plt.xlabel("$t$")
        plt.grid(which='both')
        plt.minorticks_on()

        max_value = np.max(np.abs(X_true[:, state_index]))
        if max_value > 1e-2:
            if state_index==12:
                plt.ylim([0,90])
            else:
                plt.autoscale(axis='y',tight=True)
        else:
            plt.ylim([-0.1, 0.1])
    # plt.subplots_adjust(left=None, bottom=None, right=None, top=None, hspace=0.4)
    
    f3 = plt.figure()
    (line,) = plt.plot(t, X_true[:, -1], label="true")
    plt.ylabel(states_lables[-1])
    plt.xlabel("$t$")
    plt.grid(which='both')
    plt.minorticks_on()

    if plt_show:
        plt.show()
    else:
       multipage(os.getenv("HOME") + '/m4v2-code/test_mpc.pdf')

def closed_loop_simulation():

    # create solvers
    ocp = create_ocp_solver_description()
    acados_ocp_solver = AcadosOcpSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )
    acados_integrator = AcadosSimSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )

    # prepare simulation
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    simX = np.ndarray((Nsim + 1, nx))
    simU = np.ndarray((Nsim, nu))

    xcurrent = X0
    simX[0, :] = xcurrent

    # initialize solver
    for stage in range(N_horizon + 1):
        acados_ocp_solver.set(stage, "x", X_ref)
    for stage in range(N_horizon):
        acados_ocp_solver.set(stage, "u", U_ref)

    # closed loop
    for i in range(Nsim):

        start_time = time.process_time()
        # set initial state constraint
        acados_ocp_solver.set(0, "lbx", xcurrent)
        acados_ocp_solver.set(0, "ubx", xcurrent)

        # update yref
        for j in range(N_horizon):
            yref = np.hstack((X_ref,U_ref))
            acados_ocp_solver.set(j, "yref", yref)
            # acados_ocp_solver.set(j, "p", varphi)   # set parameter
        yref_N = X_ref
        acados_ocp_solver.set(N_horizon, "yref", yref_N)

        # solve ocp
        status = acados_ocp_solver.solve()
        print("* comp time = %5g seconds\n" % (time.process_time() - start_time))

        if status not in [0, 2]:
            acados_ocp_solver.print_statistics()
            plot_robot(
                np.linspace(0, T_horizon / N_horizon * i, i + 1),
                [None,None,None,None,None],
                simU[:i, :],
                simX[: i + 1, :],
            )
            raise Exception(
                f"acados acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            )

        if status == 2:
            print(
                f"acados acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            )
        simU[i, :] = acados_ocp_solver.get(0, "u")

        # simulate system
        acados_integrator.set("x", xcurrent)
        acados_integrator.set("u", simU[i, :])

        status = acados_integrator.solve()
        if status != 0:
            raise Exception(
                f"acados integrator returned status {status} in closed loop instance {i}"
            )

        # update state
        xcurrent = acados_integrator.get("x")
        simX[i + 1, :] = xcurrent

    # plot results
    plot_robot(
        np.linspace(0, T_horizon / N_horizon * Nsim, Nsim + 1), [None, None, None, None, None], simU, simX,latexify=True,plt_show=False
    )

if __name__ == "__main__":
    closed_loop_simulation()
