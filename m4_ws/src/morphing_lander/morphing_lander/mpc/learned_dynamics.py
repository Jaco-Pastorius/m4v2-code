import casadi as cs
import l4casadi as l4c
from morphing_lander.cvae.models import CVAE
from pathlib import Path
from morphing_lander.cvae.train import TrainConfig
import torch

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

l4c_model = l4c.L4CasADi(model, model_expects_batch_dim=True, device='cuda')  # device='cuda' for GPU

x_sym = cs.MX.sym('x', 12, 1)
y_sym = l4c_model(x_sym)
f = cs.Function('y', [x_sym], [y_sym])
df = cs.Function('dy', [x_sym], [cs.jacobian(y_sym, x_sym)])
# ddf = cs.Function('ddy', [x_sym], [cs.hessian(y_sym, x_sym)[0]])

x = cs.DM([[0.], [2.], [0.],[0.], [2.], [0.],[0.], [2.], [0.],[0.], [2.], [0.]])
print(l4c_model(x))
print(f(x))
print(df(x))
# print(ddf(x))