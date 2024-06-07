import torch
from torch.utils.data import Dataset

class ResidualDataset(Dataset):
    def __init__(self, path: str,n_targets):
        """
        Initialize the morphing lander residual dataset

        Args:
            path: str; path to the dataset.
        """
        # Load the data.
        self.data = torch.load(path)

        # get subset of states to be predicted
        self.d       = self.data[:, :n_targets]
        self.x       = self.data[:, n_targets:]
        self.cond    = self.x

        self.cond_mean = self.cond.mean(dim=0)
        self.cond_var = self.cond.var(dim=0)
        self.d_mean = self.d.mean(dim=0)
        self.d_var = self.d.var(dim=0)

    def __len__(self):
        return len(self.x)

    def __getitem__(self, idx):
        return {
            "d": self.d[idx],
            "x": self.x[idx],
            "cond": self.cond[idx],
        }
