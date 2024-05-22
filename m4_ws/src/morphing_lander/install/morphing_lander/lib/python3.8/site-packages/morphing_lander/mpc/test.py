from l4casadi import L4CasADi
import torch
from IPython import embed
from casadi import SX, vertcat, MX, Function, DM, jacobian, hessian

class MLPZero(torch.nn.Module):
    def __init__(self):
        super().__init__()

        self.input_layer = torch.nn.Linear(11, 512)

        hidden_layers = []
        for i in range(2):
            hidden_layers.append(torch.nn.Linear(512, 512))

        self.hidden_layer = torch.nn.ModuleList(hidden_layers)
        self.out_layer = torch.nn.Linear(512, 12)

        # Model is not trained -- setting output to zero
        with torch.no_grad():
            self.out_layer.bias.fill_(0.)

    def forward(self, x):
        x = self.input_layer(x)
        for layer in self.hidden_layer:
            x = torch.tanh(layer(x))
        x = self.out_layer(x)
        return x

learned_model = MLPZero()
l4c_model = L4CasADi(learned_model, model_expects_batch_dim=True, device='cpu')  # device='cuda' for GPU

cond = MX.sym('X', 11, 1)
y_sym = l4c_model(cond)
f = Function('y', [cond], [y_sym])
df = Function('dy', [cond], [jacobian(y_sym, cond)])
# ddf = Function('ddy', [cond], [hessian(y_sym, cond)[0]])

cond = DM([[0.], [2.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.]])
print(l4c_model(cond))
print(f(cond))
print(df(cond))
# print(ddf(cond))
