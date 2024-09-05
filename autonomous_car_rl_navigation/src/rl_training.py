#!/usr/bin/python3

import rospy 
import numpy as np 

import torch 
import torch.nn as nn 
import torch.functional as f 
from torch.optim import Adam 

import matplotlib.pyplot as plt 


class ReplayBuffer:
    def __init__(self, max_size: int = 1e6) -> None:
        self.storage = []
        self.max_size = max_size
        self.ptr = 0 

    def add(self, data):
        if len(self.storage) == self.max_size:
            self.storage[int(self.ptr)] = data 
            self.ptr = (self.ptr + 1) % self.max_size
        else:
            self.storage.append(data)

    def sample(self, batch_size):
        ind = np.random.randint(0, len(self.storage), size=batch_size)
        x, y, u, r, d, t = [], [], [], [], []

        for i in ind:
            state, next_state, action, reward, terminated, truncated = self.storage[i]
            x.append(np.array(state, copy=False))
            y.append(np.array(state, copy=False))
            x.append(np.array(state, copy=False))
            x.append(np.array(state, copy=False))
            x.append(np.array(state, copy=False))
            x.append(np.array(state, copy=False))

