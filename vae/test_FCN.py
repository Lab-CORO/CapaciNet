

import os

import torch
from torch import nn
import torch.nn.functional as F
from torch.utils.data import DataLoader
from torchsummary import summary

import numpy as np
import torchvision
from torchvision import transforms
from reachability_map_dataset import ReachabilityMapDataset
from tqdm import tqdm
import matplotlib
import matplotlib.pyplot as plt

from sklearn.manifold import TSNE



DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
print("Project running on device: ", DEVICE)

config = {
    "batch_size": 32,
    "epochs": 10,
    "lr": 1e-3,
    "hidden_dim": 2
}

LAYERS = 3
KERNELS = [3, 3, 3]
CHANNELS = [32, 64, 128]
STRIDES = [2, 2, 2]
LINEAR_DIM = 2048

class FcnVoxel(nn.Module):
    def __init__(self,  output_dim=2):
        super(FcnVoxel, self).__init__()

        # bottleneck dimentionality
        self.output_dim = output_dim
    
        # convolutional layer hyper parameters
        self.layers = LAYERS
        self.kernels = KERNELS
        self.channels = CHANNELS
        self.strides = STRIDES
        self.conv = self.get_convs()

    def get_convs(self):
        model = nn.Sequential()

        # Encodeur
        for i in range(self.layers):

            if i == 0: model.append(nn.Conv3d(1, 
                                              self.channels[i], 
                                              kernel_size=self.kernels[i],
                                              stride=self.strides[i],
                                              padding=0))
            
            else: model.append(nn.Conv3d(self.channels[i-1], 
                                         self.channels[i],
                                         kernel_size=self.kernels[i],
                                         stride=self.strides[i],
                                         padding=0))

            model.append(nn.ReLU()) # Here we use GELU as activation function
        # Decodeur
        for i in range(self.layers):
            
            if i == self.layers: model.append(
                            nn.ConvTranspose3d(self.channels[self.layers - i],
                                               self.channels[1],
                                               kernel_size=self.kernels[i],
                                               stride=self.strides[i],
                                               padding=0,
                                               output_padding=1)
                            )
            elif i == 0 : model.append(
                            nn.ConvTranspose3d(self.channels[-1 ],
                                               self.channels[self.layers - i - 1],
                                               kernel_size=self.kernels[i],
                                               stride=self.strides[i],
                                               padding=0,
                                               output_padding=1)
                            )
                
            else: model.append(
                            nn.ConvTranspose3d(self.channels[self.layers - i],
                                               self.channels[self.layers - i -1],
                                               kernel_size=self.kernels[i],
                                               stride=self.strides[i],
                                               padding=0,
                                               output_padding=1)
                            )
            model.append(nn.ReLU()) # Here we use GELU as activation function        
        model.append(nn.Conv3d(self.channels[0], 1, kernel_size=1, stride=1))

        return model

    def forward(self, x):
        print(x.shape)

        x_hat = self.conv(x)
        return x_hat


def train(model, dataloader, optimizer):

    model.train()
    train_loss = 0.0

    batch_bar = tqdm(total=len(dataloader), 
                    leave=False, position=0, desc="Train")
    
    for i, batch in enumerate(dataloader):
        
        optimizer.zero_grad()
        x = batch['voxel_grid']
        with torch.cuda.amp.autocast():
            x_hat = model(x.unsqueeze(1))
            print("loss compute")
            loss_fn = nn.MSELoss()
            loss = loss_fn(x_hat, x)
            
            # loss = x - x_hat / config["batch_size"]
        
        train_loss += loss.item()

        scaler.scale(loss).backward()
        scaler.step(optimizer)
        scaler.update()

        print("bar update")
        batch_bar.set_postfix(
            loss = f"{train_loss/(i+1):.4f}",
            lr = f"{optimizer.param_groups[0]['lr']:.4f}"
        )
        
        batch_bar.update()
        torch.cuda.empty_cache()
        del x, x_hat
    
    batch_bar.close()
    train_loss /= len(dataloader)

    return train_loss


if __name__ == "__main__":


    dataset = ReachabilityMapDataset(root_dir='/workspace/capacitynet/data/data.h5')

    train_loader = DataLoader(dataset, batch_size=config['batch_size'], shuffle=True)
    valid_loader = DataLoader(dataset, batch_size=config['batch_size'], shuffle=True)

    model = FcnVoxel().to(DEVICE)

    summary(model, (1, 38, 38, 38))

    optimizer = torch.optim.AdamW(model.parameters(), lr=config["lr"], weight_decay=1e-5)
    scaler = torch.cuda.amp.GradScaler()
    train_loss = train(model, train_loader, optimizer)
    print(f"Epoch 1: \t Train loss: {train_loss:.4f}\t")
