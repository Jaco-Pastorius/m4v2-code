import casadi as cs
import torch
import l4casadi as l4c
from typing import Optional


class Normalizer(torch.nn.Module):
    """
    Helper module for normalizing / unnormalzing torch tensors with a given mean and variance.
    """

    def __init__(self, mean: torch.tensor, variance: torch.tensor):
        """
        Initializes Normalizer object.

        Args:
            mean: torch.tensor with shape (n,); mean of normalization.
            variance: torch.tensor, shape (n,); variance of normalization.
        """
        super(Normalizer, self).__init__()

        # Check shapes -- need data to be 1D.
        assert len(mean.shape) == 1, ""
        assert len(variance.shape) == 1

        self.register_buffer("mean", mean)
        self.register_buffer("variance", variance)

    def normalize(self, x: torch.tensor):
        """
        Applies the normalization, i.e., computes (x - mean) / sqrt(variance).
        """
        assert x.shape[-1] == self.mean.shape[0]

        return (x - self.mean) / torch.sqrt(self.variance)

    def unnormalize(
        self,
        mean_normalized: torch.tensor,
        var_normalized: Optional[torch.tensor] = None,
    ):
        """
        Applies the unnormalization to the mean and variance, i.e., computes
        mean_normalized * sqrt(variance) + mean and var_normalized * variance.
        """
        assert mean_normalized.shape[-1] == self.mean.shape[0]
        if var_normalized is not None:
            assert var_normalized.shape[-1] == self.variance.shape[0]

        mean = mean_normalized * torch.sqrt(self.variance) + self.mean
        if var_normalized is not None:
            variance = var_normalized * self.variance
            return mean, variance
        else:
            return mean

class MLP(torch.nn.Module):
    """
    Implements a generic feedforward neural network.
    """

    def __init__(self, input_dim: int, output_dim: int, hidden_dims: list):
        """
        Initializes the MLP.

        Args:
            input_dim: int; dimension of the input.
            output_dim: int; dimension of the output.
            hidden_dims: list of int; dimensions of the hidden layers.
        """
        super(MLP, self).__init__()

        self.input_dim = input_dim
        self.output_dim = output_dim
        self.hidden_dims = hidden_dims

        # Create the layers.
        self.layers = torch.nn.ModuleList()
        prev_dim = input_dim
        for dim in hidden_dims:
            self.layers.append(torch.nn.Linear(prev_dim, dim))
            self.layers.append(torch.nn.ReLU())
            prev_dim = dim
        self.layers.append(torch.nn.Linear(prev_dim, output_dim))

    def forward(self, x: torch.tensor):
        """
        Forward pass through the network.
        """
        for layer in self.layers:
            x = layer(x)
        return x

class CVAE(torch.nn.Module):
    """
    Implements a conditional variational autoencoder (CVAE) with
    normalization of the conditioning and output data.
    """

    def __init__(
        self,
        output_dim: int,
        latent_dim: int,
        cond_dim: int,
        encoder_layers: list,
        decoder_layers: list,
        prior_layers: list,
        cond_mean: Optional[torch.tensor] = None,
        cond_var: Optional[torch.tensor] = None,
        output_mean: Optional[torch.tensor] = None,
        output_var: Optional[torch.tensor] = None,
    ):
        """
        Initializes the CVAE.

        Args:
            output_dim: int; dimension of the output data.
            latent_dim: int; dimension of the latent space.
            cond_dim: int; dimension of the conditioning data.
            encoder_layers: list of int; dimensions of the hidden layers of the encoder.
            decoder_layers: list of int; dimensions of the hidden layers of the decoder.
            cond_mean: torch.tensor with shape (cond_dim,); mean of the conditioning data.
            cond_var: torch.tensor with shape (cond_dim,); variance of the conditioning data.
            output_mean: torch.tensor with shape (output_dim,); mean of the output data.
            output_var: torch.tensor with shape (output_dim,); variance of the output data.
        """
        super(CVAE, self).__init__()

        # Create the normalization layers.
        if cond_mean is None:
            cond_mean = torch.zeros(cond_dim)
        if cond_var is None:
            cond_var = torch.ones(cond_dim)
        if output_mean is None:
            output_mean = torch.zeros(output_dim)
        if output_var is None:
            output_var = torch.ones(output_dim)

        self.cond_normalizer = Normalizer(cond_mean, cond_var)
        self.output_normalizer = Normalizer(output_mean, output_var)

        self.output_dim = output_dim
        self.latent_dim = latent_dim
        self.cond_dim = cond_dim

        # Create the encoder and decoder.
        self.encoder = MLP(
            input_dim=cond_dim + output_dim,
            output_dim=2 * latent_dim,
            hidden_dims=encoder_layers,
        )
        self.decoder = MLP(
            input_dim=cond_dim + latent_dim,
            output_dim=2 * output_dim,
            hidden_dims=decoder_layers,
        )
        self.prior_network = MLP(
            input_dim=cond_dim, output_dim=2 * latent_dim, hidden_dims=prior_layers
        )

        # self.samples = torch.randn((100, 1, 1))

    def encode(self, x: torch.tensor, cond: torch.tensor):
        """
        Encodes the input data and returns the mean and variance of the latent space.
        """
        input_data = torch.cat([x, cond], dim=-1)
        z_params = self.encoder(input_data)
        z_mean, z_logvar = torch.chunk(z_params, 2, dim=-1)
        z_variance = torch.exp(z_logvar)
        return z_mean, z_variance

    def decode(self, z: torch.tensor, cond: torch.tensor, unnormalize: bool):
        """
        Decodes the latent space and returns the output data.
        """
        input_data = torch.cat([z, cond], dim=-1)
        output_params = self.decoder(input_data)
        output_mean_normalized, output_logvar_normalized = torch.chunk(
            output_params, 2, dim=-1
        )
        output_variance_normalized = torch.exp(output_logvar_normalized)

        if unnormalize:
            output_mean, output_variance = self.output_normalizer.unnormalize(
                output_mean_normalized, output_variance_normalized
            )
        else:
            output_mean = output_mean_normalized
            output_variance = output_variance_normalized

        return output_mean, output_variance

    def prior(self, cond: torch.tensor):
        """
        Returns the mean and variance of the prior distribution.
        """
        prior_params = self.prior_network(cond)
        prior_mean, prior_logvar = torch.chunk(prior_params, 2, dim=-1)
        prior_variance = torch.exp(prior_logvar)
        # prior_mean = torch.zeros_like(prior_mean)
        # prior_variance = torch.ones_like(prior_variance)

        return prior_mean, prior_variance

    def forward(self, cond: torch.tensor):
        num_samples = 100
        cond = self.cond_normalizer.normalize(cond)

        # Sample the prior.
        prior_mean, prior_var = self.prior(cond)

        prior_dist = torch.distributions.MultivariateNormal(
            loc=prior_mean, covariance_matrix=torch.diag_embed(prior_var)
        )

        z = prior_dist.rsample((num_samples,))
        #z = torch.ones((num_samples, 1, 1))

        cond_expanded = cond.unsqueeze(0).expand(num_samples, -1, -1)

        # Decode the samples
        pred_mean_expanded, pred_var_expanded = self.decode(
            z, cond_expanded, unnormalize=True
        )


        pred_mean = pred_mean_expanded.mean(dim=0)
        pred_var_expanded = torch.diag_embed(pred_var_expanded)

        pred_var = torch.mean(
            pred_var_expanded
            + pred_mean_expanded.unsqueeze(-1) * pred_mean_expanded.unsqueeze(-2),
            dim=0,
        )

        pred_var = pred_var - pred_mean.unsqueeze(-1) @ pred_mean.unsqueeze(-2)

        return pred_mean #, pred_var #L4CasADi expects a single output

pyTorch_model = CVAE(output_dim=1,
                     latent_dim=1,
                     cond_dim=2,
                     encoder_layers=[16,16],
                     decoder_layers=[16,16],
                     prior_layers  =[16,16],
                    )
l4c_model = l4c.L4CasADi(pyTorch_model, model_expects_batch_dim=True, device='cpu')  # device='cuda' for GPU

x_sym = cs.MX.sym('x', 2, 1)
y_sym = l4c_model(x_sym)
f = cs.Function('y', [x_sym], [y_sym])
df = cs.Function('dy', [x_sym], [cs.jacobian(y_sym, x_sym)])
ddf = cs.Function('ddy', [x_sym], [cs.hessian(y_sym, x_sym)[0]])

x = cs.DM([[0.], [2.]])
print(l4c_model(x))
print(f(x))
print(df(x))
print(ddf(x))