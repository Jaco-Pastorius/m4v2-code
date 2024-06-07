from acados_template import AcadosOcpSolver, AcadosSimSolver
import numpy as np
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

from morphing_lander.mpc.mpc import create_ocp_solver_description
from morphing_lander.mpc.parameters import params_
from morphing_lander.mpc.trajectories import traj_jump_time
from morphing_lander.mpc.integrator import create_sim_solver_description
from morphing_lander.mpc.dynamics import dynamics_func, dynamics_with_res_func

# get parameters
Ts                         = params_.get('Ts')
N_horizon                  = params_.get('N_horizon')
T_horizon                  = params_.get('T_horizon')
acados_ocp_path            = params_.get('acados_ocp_path')
acados_sim_path            = params_.get('acados_sim_path')
use_residual_model         = params_.get('use_residual_model')
l4c_residual_model         = params_.get('l4c_residual_model')
generate_mpc               = params_.get('generate_mpc')
build_mpc                  = params_.get('build_mpc')
model_states_in_idx        = params_.get('model_states_in_idx')
model_inputs_in_idx        = params_.get('model_inputs_in_idx')
model_phi_in               = params_.get('model_phi_in')
model_states_out_idx       = params_.get('model_states_out_idx')
model_ninputs              = params_.get('model_ninputs')
model_noutputs             = params_.get('model_noutputs')
v_max_absolute             = params_.get('v_max_absolute')
max_tilt_in_flight         = params_.get('max_tilt_in_flight')

# set initial state and reference
X0    = np.zeros(12)
x_ref = np.zeros(12)
x_ref[2] = -1.5
u_ref = np.zeros(4)

Nsim = int(7.0/Ts)

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
    phi_vec=None
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

    control_lables = [r"$u_1$", r"$u_2$",r"$u_3$", r"$u_4$"]

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

    states_lables = [r"$x$",r"$y$",r"$z$", r"$\psi$", r"$ \theta $", r"$ \phi $", r"$\dot x$",r"$\dot y$", r"$\dot z$",r"$\omega_x$", r"$\omega_y$",r"$\omega_z$",r"\varphi"]
    f2 = plt.figure(figsize=(10, 6))

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
            plt.autoscale(axis='y')
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
    (line,) = plt.plot(t, np.rad2deg(phi_vec), label="true")
    plt.ylabel(states_lables[-1])
    plt.xlabel("$t$")
    plt.grid(which='both')
    plt.minorticks_on()

    plt.ticklabel_format(useOffset=False, style='plain')

    if plt_show:
        plt.show()
    else:
       multipage(os.getenv("HOME") + '/m4v2-code/test_mpc.pdf')

def closed_loop_simulation():

    # create solvers
    ocp = create_ocp_solver_description()
    sim = create_sim_solver_description()
    acados_ocp_solver = AcadosOcpSolver(
        ocp, 
        json_file=os.path.join(acados_ocp_path, ocp.model.name + '_acados_ocp.json'),
        generate =generate_mpc,
        build    =build_mpc
    )
    acados_integrator = AcadosSimSolver(
        sim, 
        json_file=os.path.join(acados_sim_path, ocp.model.name + '_acados_sim.json'),
        generate =generate_mpc,
        build    =build_mpc
    )

    # prepare simulation
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    # simulation arrays 
    simX = np.ndarray((Nsim + 1, nx))
    simU = np.ndarray((Nsim,     nu))

    xcurrent   = X0
    simX[0, :] = xcurrent

    # solver initialization
    for stage in range(N_horizon + 1):
        acados_ocp_solver.set(stage, "x", np.zeros(12))
        if use_residual_model:
            l4c_params = l4c_residual_model.get_params(np.zeros((1,model_ninputs)))
            acados_ocp_solver.set(stage, "p", np.hstack((np.array([0.0]),l4c_params.squeeze())))
        else:
            acados_ocp_solver.set(stage, "p", np.array([0.0]))

    for stage in range(N_horizon):
        acados_ocp_solver.set(stage, "u", np.zeros(4))

    # closed loop
    t = 0.0
    phicurrent = 0.0
    phi_vec = [phicurrent]
    for i in range(Nsim):
        start_time = time.process_time()

        # get reference
        x_ref,u_ref,tilt_vel,tracking_done = traj_jump_time(t)
        
        # set initial state constraint
        acados_ocp_solver.set(0, "lbx", xcurrent)
        acados_ocp_solver.set(0, "ubx", xcurrent)

        # solve ocp
        mpc_status = acados_ocp_solver.solve()  
        print(f"mpc_status: {mpc_status}")

        # get first input
        u_opt = acados_ocp_solver.get(0, "u")   
        print(u_opt)

        # set the reference and parameters
        if use_residual_model:
            cond_vec = []
            for j in range(N_horizon):
                yref = np.hstack((x_ref,u_ref))
                acados_ocp_solver.set(j, "yref", yref)
                cond = np.hstack((xcurrent[model_states_in_idx],u_opt[model_inputs_in_idx]))
                if model_phi_in:
                    cond = np.hstack((cond,phicurrent))
                cond_vec.append(cond)
            params = l4c_residual_model.get_params(np.stack(cond_vec,axis=0))
            for jj in range(N_horizon):
                acados_ocp_solver.set(jj, "p", np.hstack((np.array([phicurrent]),params[jj])))
        else:
            for j in range(N_horizon):
                yref = np.hstack((x_ref,u_ref))
                acados_ocp_solver.set(j, "yref", yref)
                acados_ocp_solver.set(j, "p", np.array([phicurrent]))

        comp_time = time.process_time() - start_time
        print("* comp time = %5g seconds\n" % (comp_time))

        # apply the first input to the simulation
        simU[i,:] = u_opt

        # simulate system
        acados_integrator.set("x", xcurrent)
        acados_integrator.set("u", simU[i, :])
        acados_integrator.set("p",np.array([phicurrent]))
        acados_integrator.solve()

        # update state
        xcurrent = acados_integrator.get("x")
        simX[i + 1, :] = xcurrent

        # update phi
        phicurrent += Ts * tilt_vel * v_max_absolute
        phicurrent=np.clip(phicurrent,0,max_tilt_in_flight)
        phi_vec.append(phicurrent)

        # update time
        t += Ts

        # print residual dynamics and normal dynamics 
        # print(f"dynamics residual: {dynamics_with_res_func()(xcurrent,u_opt,phicurrent,params[0]) - dynamics_func()(xcurrent,u_opt,phicurrent)}")

    # plot results
    plot_robot(
        np.linspace(0, Ts * Nsim, Nsim + 1), 
        [None, None, None, None, None], 
        simU, 
        simX,
        latexify=True,
        plt_show=True,
        phi_vec=np.array(phi_vec)
    )

if __name__ == "__main__":
    closed_loop_simulation()
