# Acados/Casadi
from numpy import array, zeros, hstack
from acados_template import AcadosSim
from morphing_lander.mpc.parameters import params_
from morphing_lander.mpc.dynamics import export_robot_model, export_robot_model_ground_effect

Ts                 = params_.get('Ts')
acados_sim_path    = params_.get('acados_sim_path')
use_residual_model = params_.get('use_residual_model')
l4c_residual_model = params_.get('l4c_residual_model')
model_ninputs      = params_.get('model_ninputs')

def create_sim_solver_description() -> AcadosSim:
    sim = AcadosSim()
    sim.model = export_robot_model_ground_effect()
    sim.model.name = 'integrator'
    sim.solver_options.integrator_type = 'IRK'
    sim.solver_options.num_stages = 6
    sim.solver_options.num_steps = 3
    sim.solver_options.T = Ts
    sim.parameter_values = array([0.0])
    sim.code_export_directory = acados_sim_path
    return sim