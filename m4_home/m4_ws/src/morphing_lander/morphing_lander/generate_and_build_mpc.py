import os 
from acados_template import AcadosOcpSolver
from morphing_lander.morphing_lander_mpc import create_ocp_solver_description

ocp = create_ocp_solver_description()
acados_models_path = os.getenv("HOME") +'/m4v2-code/m4_home/m4_ws/src/morphing_lander/morphing_lander/acados_models/'
acados_ocp_solver = AcadosOcpSolver(
    ocp, 
    json_file=os.path.join(acados_models_path, ocp.model.name + '_acados_ocp.json'),
)