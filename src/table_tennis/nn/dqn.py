#!/usr/bin/env python
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np

class ActionNN(nn.Module):
    
    def __init__(self, inputs, outputs):
        super(ActionNN, self).__init__()
        self.pipe = nn.Sequential(
            nn.Linear(inputs, 32),
            nn.ReLU(),
            nn.Linear(32,32),
            nn.ReLU(),
            nn.Linear(32,32),
            nn.ReLU(),
            nn.Linear(32, outputs)
        )
    
    def forward(self, x):
        action = self.pipe(x)
        return action