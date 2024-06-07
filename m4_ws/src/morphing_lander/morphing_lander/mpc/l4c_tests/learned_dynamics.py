import casadi as cs
import l4casadi as l4c
from morphing_lander.cvae.models import CVAE
from pathlib import Path
from morphing_lander.cvae.train import TrainConfig
import torch
import numpy as np 

# Specify the path for the learned models
output_path = Path(
    "/home/m4pc/m4v2-code/m4_ws/src/morphing_lander/learned_models/"
)

train_config = TrainConfig(
    output_dim=12,
    latent_dim=6,
    cond_dim=12,
    encoder_layers=[32, 32],
    decoder_layers=[32, 32],
    prior_layers=[32, 32],
    batches_per_epoch=10,
    epochs=200,
    step_size=50,  # steps per decay for lr scheduler
    gamma=0.75,  # multiplicative decay for lr scheduler
    lr=5e-4,
    save_epochs=10,
    val_epochs=10,
    device="cuda" if torch.cuda.is_available() else "cpu",
)

print(f"Device is: {train_config.device}")

model = CVAE(
    output_dim=train_config.output_dim,
    latent_dim=train_config.latent_dim,
    cond_dim=train_config.cond_dim,
    encoder_layers=train_config.encoder_layers,
    decoder_layers=train_config.decoder_layers,
    prior_layers=train_config.prior_layers,
    # cond_mean=train_dataset.cond_mean,
    # cond_var=train_dataset.cond_var,
    # output_mean=train_dataset.d_mean,
    # output_var=train_dataset.d_var,
).to(train_config.device)

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
# print(ddf(x))