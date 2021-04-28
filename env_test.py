import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

import time
import random
import copy
from copy import deepcopy
import threading

import gym3
from procgen import ProcgenGym3Env


train_num_levels = 100_000 #500 #1500
train_start_level = 0

for _ in range(10):
    bs = 144 * 10

    env = ProcgenGym3Env(num=bs, env_name="testgame", num_levels=train_num_levels, start_level=train_start_level,
                    color_theme=[5], color_theme_road=[5], background_noise_level=0)

    s = np.array([[.0,.0] for _ in range(bs)], dtype=np.float32)

    seq_len = 30

    for i in range(seq_len):
        env.act(s)
        rew, obs, first = env.observe()
        img = obs['rgb']
        info = env.get_info()
    
    env.close()