import casadi as cs
import l4casadi as l4c
from morphing_lander.cvae.models import CVAE,MLPZero
from pathlib import Path
from morphing_lander.cvae.train import TrainConfig
import torch
import numpy as np 
from IPython import embed

model = MLPZero()

l4c_model = l4c.realtime.RealTimeL4CasADi(model, approximation_order=1)

in_sym = cs.MX.sym('in_sym',12,1)
y_sym = l4c_model(in_sym)
residual_func = cs.Function('model_rt_approx',
                        [in_sym, l4c_model.get_sym_params()],
                        [y_sym])

x = np.ones([1, 12])  # torch needs batch dimension
casadi_param = l4c_model.get_params(x)
casadi_out = residual_func(x, casadi_param)  # transpose for vector rep. expected by casadi

t_out = model(torch.tensor(x, dtype=torch.float32))

print(casadi_out)
print(t_out)