import torch
from torch.utils.data import DataLoader
from morphing_lander.cvae.models import CVAE
from morphing_lander.cvae.cvae_utils import elbo_loss
from dataclasses import dataclass, asdict
from pathlib import Path
from datetime import datetime
import yaml
from morphing_lander.cvae.data import MLDataset
from IPython import embed

@dataclass
class TrainConfig:
    # Model architecture
    output_dim: int
    latent_dim: int
    cond_dim: int
    encoder_layers: list
    decoder_layers: list
    prior_layers: list

    # Training hyperparams
    batches_per_epoch: int
    epochs: int
    step_size: int
    gamma: float
    lr: float
    save_epochs: int
    val_epochs: int
    device: str

def train_loop(
    model: CVAE,
    train_dataloader: DataLoader,
    val_dataloader: DataLoader,
    output_dir: Path,
    config: TrainConfig,
):
    """
    Trains the CVAE model.

    Args:
        model: CVAE; the CVAE model.
        train_dataloader: DataLoader; the training dataloader.
        epochs: int; the number of epochs to train for.
    """
    # Set the model to training mode.
    model.train()

    # Create the optimizer.
    optimizer = torch.optim.Adam(model.parameters(), lr=config.lr, weight_decay=1e-2)

    # Train the model.
    for epoch in range(config.epochs):
        # Train the model for one epoch.
        for batch in train_dataloader:
            # Zero the gradients.
            optimizer.zero_grad()

            # Compute the loss.
            loss = elbo_loss(
                model, batch["d"].to(config.device), batch["cond"].to(config.device)
            )

            # Backpropagate the gradients.
            loss.backward()
            # torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)

            # Update the weights.
            optimizer.step()

        # Print the loss.
        if epoch % config.val_epochs == 0:
            val_loss = 0
            for batch in val_dataloader:
                val_loss += elbo_loss(
                    model, batch["d"].to(config.device), batch["cond"].to(config.device)
                ).item()
            val_loss /= len(val_dataloader)
            print(f"Epoch {epoch}: train_loss={loss}, val_loss={val_loss}")

        # Save the model.
        if epoch % config.save_epochs == 0:
            torch.save(model.state_dict(), output_dir / f"model_{epoch}.pth")

def train(train_data_path,val_data_path,train_config,output_path):
    train_dataset = MLDataset(train_data_path)
    val_dataset   = MLDataset(val_data_path)
    batch_size = int(len(train_dataset) / train_config.batches_per_epoch)
    train_dataloader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_dataloader = DataLoader(val_dataset, batch_size=batch_size, shuffle=True)
    run_id = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    # Create the output directory.
    output_dir = output_path / run_id
    output_dir.mkdir(parents=True, exist_ok=True)

    # Dump the training configuration.
    with open(output_dir / "config.yaml", "w") as f:
        yaml.dump(asdict(train_config), f)

    model = CVAE(
        output_dim=train_config.output_dim,
        latent_dim=train_config.latent_dim,
        cond_dim=train_config.cond_dim,
        encoder_layers=train_config.encoder_layers,
        decoder_layers=train_config.decoder_layers,
        prior_layers=train_config.prior_layers,
        cond_mean=train_dataset.cond_mean,
        cond_var=train_dataset.cond_var,
        output_mean=train_dataset.d_mean,
        output_var=train_dataset.d_var,
    ).to(train_config.device)

    train_loop(model, train_dataloader, val_dataloader, output_dir, train_config)
    torch.save(model.state_dict(), output_dir / f"model_{train_config.epochs}.pth")

    return model