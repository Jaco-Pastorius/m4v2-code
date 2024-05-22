import os 
from acados_template import AcadosOcpSolver, AcadosSimSolver
from morphing_lander.morphing_lander_mpc import create_ocp_solver_description
from morphing_lander.morphing_lander_integrator import create_sim_solver_description
from morphing_lander.parameters          import params_

acados_ocp_path            = params_.get('acados_ocp_path')
acados_sim_path            = params_.get('acados_sim_path')

# compile acados solver
ocp        = create_ocp_solver_description()
sim        = create_sim_solver_description()

acados_ocp_solver = AcadosOcpSolver(
    ocp, 
    json_file=os.path.join(acados_ocp_path, ocp.model.name + '_acados_ocp.json'),
    generate=False,
    build   =False
)
acados_integrator = AcadosSimSolver(
    sim,
    json_file=os.path.join(acados_sim_path, sim.model.name + '_acados_sim.json'),
    generate=True,
    build   =True
)