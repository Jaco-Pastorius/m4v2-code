# Acados/Casadi
from numpy import array, zeros, hstack, pi
from acados_template import AcadosSim
from morphing_lander.mpc.parameters import params_
from morphing_lander.mpc.dynamics import export_robot_model, export_robot_model_ground_effect, export_robot_model_driving, export_hybrid_robot_model

Ts                 = params_.get('Ts')
acados_sim_path    = params_.get('acados_sim_path')
acados_sim_path_driving    = params_.get('acados_sim_path_driving')
acados_sim_path_hybrid    = params_.get('acados_sim_path_hybrid')
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

def create_sim_solver_description_driving() -> AcadosSim:
    sim = AcadosSim()
    sim.model = export_robot_model_driving()
    sim.model.name = 'integrator'
    sim.solver_options.integrator_type = 'IRK'
    sim.solver_options.num_stages = 6
    sim.solver_options.num_steps = 3
    sim.solver_options.T = Ts
    sim.code_export_directory = acados_sim_path_driving
    return sim

def create_sim_solver_description_hybrid() -> AcadosSim:
    sim = AcadosSim()
    sim.model = export_hybrid_robot_model()
    sim.model.name = 'integrator'
    sim.solver_options.integrator_type = 'IRK'
    sim.solver_options.num_stages = 6
    sim.solver_options.num_steps = 3
    sim.solver_options.T = Ts
    sim.parameter_values = array([pi/2,True])
    sim.code_export_directory = acados_sim_path_hybrid
    return sim