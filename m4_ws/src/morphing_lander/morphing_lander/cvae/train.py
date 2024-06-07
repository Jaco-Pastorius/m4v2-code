import torch
from torch.utils.data import DataLoader
from dataclasses import dataclass, asdict
from datetime import datetime
import yaml

@dataclass 
class TrainHyperParams:
    # Training hyperparams
    batches_per_epoch: int
    epochs: int
    step_size: int
    gamma: float
    lr: float
    save_epochs: int
    val_epochs: int
    device: str

@dataclass
class TrainConfigMLP(TrainHyperParams):
    # Model architecture
    input_dim: int
    output_dim: int
    hidden_layers: list


@dataclass
class TrainConfigCVAE(TrainHyperParams):
    # Model architecture
    output_dim: int
    latent_dim: int
    cond_dim: int
    encoder_layers: list
    decoder_layers: list
    prior_layers: list

def train_loop(
    model,
    train_dataloader,
    val_dataloader,
    output_dir,
    config,
):
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
            loss = model.loss(batch["d"].to(config.device), 
                              batch["cond"].to(config.device))

            # Backpropagate the gradients.
            loss.backward()

            # Update the weights.
            optimizer.step()

        # Print the loss.
        if epoch % config.val_epochs == 0:
            val_loss = 0
            for batch in val_dataloader:
                val_loss += model.loss(
                    batch["d"].to(config.device), batch["cond"].to(config.device)
                ).item()
            val_loss /= len(val_dataloader)
            print(f"Epoch {epoch}: train_loss={loss}, val_loss={val_loss}")

        # Save the model.
        if epoch % config.save_epochs == 0:
            torch.save(model.state_dict(), output_dir / f"model_{epoch}.pth")

def train(model,train_dataset,val_dataset,train_config,output_path):
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

    train_loop(model, train_dataloader, val_dataloader, output_dir, train_config)
    torch.save(model.state_dict(), output_dir / f"model_{train_config.epochs}.pth")

    return model