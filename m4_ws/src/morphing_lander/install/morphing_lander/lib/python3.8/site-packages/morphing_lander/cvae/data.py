import torch
from torch.utils.data import Dataset

class MLDataset(Dataset):
    def __init__(self, path: str):
        """
        Initialize the morphing lander dataset

        Args:
            path: str; path to the dataset.
        """
        # Load the data.
        self.data = torch.load(path)
        self.d       = self.data[:, :12]
        self.x       = self.data[:, 12:]
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
