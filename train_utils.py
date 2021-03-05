import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import random


aux_properties = ['angle_to_wp', 'last_applied_steer', 'last_applied_throttle', 'current_speed', 'dv']
control_properties = ['steer', 'throttle']
aux_pred = [] #['front_angle']

targets = control_properties + aux_pred

from procgen import ProcgenGym3Env
import time
import threading

class DataLoader:
    def __init__(self, env=None, bs=16, seq_len=400):
        self.bs = bs
        self.seq_len = seq_len
        self.queued_chunks = []
        self.retry_counter = 0
    
        self.DAGGER_CADENCE = 100
        self.DAGGER_DURATION = 10
        self.do_dagger = False
        self.dagger_counter = 0


        self.env = ProcgenGym3Env(num=bs, env_name="testgame") if not env else env
        self.queue_up_chunk()

    
    def get_chunk(self):

        while len(self.queued_chunks)==0 and self.retry_counter < 200:
            print("Waiting for next chunk...")
            self.retry_counter += 1
            time.sleep(1)

        thread = threading.Thread(target=self.queue_up_chunk)
        thread.start()
        
        if len(self.queued_chunks)==0:
            print("Finished dataloader!")
            return None
        else:
            self.retry_counter = 0
            return self.queued_chunks.pop(0)
        
    def queue_up_chunk(self):
        t1 = time.time()
        bs = self.bs
        env = self.env
        seq_len = self.seq_len

        s = np.array([[.0, .0] for _ in range(bs)], dtype=np.float32)
        
        # Empty np containers
        front_container = np.empty((seq_len, bs, IMG_SZ, IMG_SZ, 3), dtype=np.uint8)
        aux_container = np.empty((seq_len, bs, len(aux_properties)), dtype=np.float)
        targets_container = np.empty((seq_len, bs, len(targets)), dtype=np.float)
        
        # Env interaction. Fill out np containers
        for i in range(seq_len):
            env.act(s)
            rew, obs, first = env.observe()
            img = obs['rgb']
            info = env.get_info()

            autopilot_controls = np.array([[e['autopilot_steer'], e['autopilot_throttle']] for e in info]) # piping back in autopilot to align inputs-outputs.

            # DAGGER

            # Create dagger controls, toggle it on and off
            if i % self.DAGGER_CADENCE == 0:
                self.dagger_counter = 0
                steer_aug = random.uniform(-.3, .3)
                #throttle_aug = random.uniform(.5, 1.5)
                daggerized_controls = np.array([[c[0]+steer_aug, c[1]] for c in autopilot_controls])
                self.do_dagger = True
            elif self.dagger_counter == self.DAGGER_DURATION:
                self.do_dagger = False

            # Implement control
            if self.do_dagger:
                s = daggerized_controls
                self.dagger_counter+=1
            else:
                s = autopilot_controls
            

            front_container[i,:,:,:,:] = img
            targets_container[i,:,0] = np.array([e['autopilot_steer'] for e in info])
            targets_container[i,:,1] = np.array([e['autopilot_throttle'] for e in info])
            #targets_container[i,:,2] = np.array([e['front_angle'] for e in info])

            for aux_i, aux_col in enumerate(aux_properties):
                aux_container[i,:,aux_i] = np.array([e[aux_col] for e in info])

            
        # np containers to pytorch containers
        front_container = torch.from_numpy(front_container.astype(np.float32)/255.).permute(0,1,4,2,3)
        aux_container = torch.from_numpy(aux_container.astype(np.float32))
        targets_container = torch.from_numpy(targets_container.astype(np.float32))
        
        # add to queued chunks
            
        self.queued_chunks.append((front_container,
                                    aux_container, 
                                    targets_container))
        
        print(f"Queueing chunk of size {front_container.shape} took {np.round(time.time() - t1, 2)} seconds")  




n_aux = len(aux_properties)
n_targets = len(targets)
DROP = .20
N_LSTM_LAYERS = 1
N_LSTM_HIDDEN = 512

IMG_SZ = 64

class Flatten(nn.Module):
    def forward(self, x):
        return x.contiguous().view(x.size(0), -1) #contiguous is bc of how the tensors are stored on disk. Can also do reshape instead of view



class VizCNN(nn.Module):
    def __init__(self, use_rnn=False):
        super(VizCNN, self).__init__()

        self.pooler = nn.MaxPool2d(kernel_size=2)
        self.act = nn.ReLU()

        self.conv_1a = nn.Conv2d(in_channels=3, out_channels=16, kernel_size=7)
        self.conv_2a = nn.Conv2d(in_channels=16, out_channels=32, kernel_size=5)
        self.conv_2b = nn.Conv2d(in_channels=32, out_channels=32, kernel_size=5)
        self.conv_3a = nn.Conv2d(in_channels=32, out_channels=32, kernel_size=5)
        self.conv_4a = nn.Conv2d(in_channels=32, out_channels=32, kernel_size=5)

        n = 5408 + n_aux # flattened CNN activations + aux

        self.fc0 = nn.Linear(n, N_LSTM_HIDDEN)

        if use_rnn:
            self.lstm = nn.LSTM(N_LSTM_HIDDEN, N_LSTM_HIDDEN, N_LSTM_LAYERS)
        self.use_rnn = use_rnn

        self.fc1 = nn.Linear(N_LSTM_HIDDEN, 512) 
        self.fc2 = nn.Linear(512, n_targets)

        # placeholder for the gradients
        self.gradients = None
        
    def activations_hook(self, grad):
        self.gradients = grad
    
    def get_activations_gradient(self):
        return self.gradients
        
    def forward(self, x, aux, hidden, return_salmap=False, register_activations=False):

        # 7x7 convolutional layer with 16 channels (layer 1a)
        # 2x2 L2 pooling layer

        # 5x5 convolutional layer with 32 channels (layer 2a)
        # 5x5 convolutional layer with 32 channels (layer 2b)
        # 2x2 L2 pooling layer

        # 5x5 convolutional layer with 32 channels (layer 3a)
        # 2x2 L2 pooling layer

        # 5x5 convolutional layer with 32 channels (layer 4a)
        # 2x2 L2 pooling layer

        # 256-unit dense layer
        # 512-unit dense layer
        # 10-unit dense layer (1 unit for the value function, 9 units for the policy logits)

        # Reshape for cnn
        seq_len, bs, C, H, W = x.shape
        _, _, n_aux = aux.shape

        x = x.view(seq_len*bs, C, H, W)
        aux = aux.view(seq_len*bs, n_aux)

        x = self.conv_1a(x)
        x = self.act(x)
        x = self.pooler(x)

        ################
        # Grabbing gradients and activations for viz
        activations = x
        if register_activations:
            activations.register_hook(self.activations_hook)
        salmap = activations.detach().cpu().numpy()
        x = activations
        ################


        x = self.conv_2a(x)
        # TODO NOTE WHY ARE WE MISSING ACTIVATIONS HERE??
        x = self.conv_2b(x)
        #x = self.pooler(x)
        x = self.act(x)

        # Distil activations come from this layer

        x = self.conv_3a(x)
        #x = self.pooler(x)
        x = self.act(x)

        x = self.conv_4a(x)
        #x = self.pooler(x)
        x = self.act(x)

        x = Flatten()(x)
        
        features = torch.cat([x, aux], dim=-1); #print(features.shape)

        # Back to shape for RNN
        features = features.view(seq_len, bs, -1)

        features = self.fc0(features)
        features = self.act(features)

        # Not in original Distill model
        if self.use_rnn:
            features, hidden = self.lstm(features, hidden)

        features = self.fc1(features)
        features = self.act(features)

        features = self.fc2(features)

        
        if return_salmap:
            return features, hidden, salmap
        else:
            return features, hidden








# Commenting out while simplifying CNN above


def get_hidden(bs, device='cuda'):
    return (torch.zeros(N_LSTM_LAYERS, bs, N_LSTM_HIDDEN).to(device), 
            torch.zeros(N_LSTM_LAYERS, bs, N_LSTM_HIDDEN).to(device))


class ResBlock(nn.Module):
    def __init__(self, n_channels):
        super(ResBlock, self).__init__()
        self.conv1 = nn.Conv2d(in_channels=n_channels, out_channels=n_channels, kernel_size=3, stride=1, padding=1)
        self.conv2 = nn.Conv2d(in_channels=n_channels, out_channels=n_channels, kernel_size=3, stride=1, padding=1)
        self.bn1 = nn.BatchNorm2d(n_channels)
        self.bn2 = nn.BatchNorm2d(n_channels)
    
    def forward(self, x):
        out = nn.ReLU()(x)
        out = self.conv1(out)
        out = self.bn1(out)
        out = nn.ReLU()(out)
        out = self.conv2(out)
        out = self.bn2(out)
        
        return out + x
    
class ImpalaBlock(nn.Module):
    def __init__(self, in_channels, out_channels, stride=1): 
        # Stride was always 1 before but bigger img was resulting in too-big output.
        super(ImpalaBlock, self).__init__()
        self.conv1 = nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=3, stride=stride)
        self.res1 = ResBlock(out_channels)
        # dstill piece put the viz here?
        self.res2 = ResBlock(out_channels)
        self.bn = nn.BatchNorm2d(out_channels)
        
    def forward(self, x):
        x = self.conv1(x)
        x = self.bn(x)
        x = nn.MaxPool2d(kernel_size=3, stride=2)(x)
        x = self.res1(x)
        x = self.res2(x)
        return x

class ImpalaCNN(nn.Module):
    def __init__(self, use_rnn=False):
        scale = 2
        super(ImpalaCNN, self).__init__()
        self.block1 = ImpalaBlock(in_channels=3, out_channels=16 * scale)
        self.block2 = ImpalaBlock(in_channels=16 * scale, out_channels=32 * scale, stride=1)
        self.block3 = ImpalaBlock(in_channels=32 * scale, out_channels=64 * scale, stride=1)

        self.side_block1 = ImpalaBlock(in_channels=3, out_channels=16)
        self.side_block2 = ImpalaBlock(in_channels=16, out_channels=16)
        self.side_block3 = ImpalaBlock(in_channels=16, out_channels=16)

        c = 3200 #1603-n_aux #3200

        n = c + n_aux
        self.n = n

        self.use_rnn = use_rnn

        self.fc0 = nn.Linear(n, N_LSTM_HIDDEN)

        if use_rnn:
            self.lstm = nn.LSTM(N_LSTM_HIDDEN, N_LSTM_HIDDEN, N_LSTM_LAYERS)

        self.fc1 = nn.Linear(N_LSTM_HIDDEN, 512) 
        self.fc2 = nn.Linear(512, n_targets)

                # placeholder for the gradients
        self.gradients = None
        
    def activations_hook(self, grad):
        self.gradients = grad
    
    def get_activations_gradient(self):
        return self.gradients
        
    def forward(self, x, aux, hidden, return_salmap=False, register_activations=False):

        # Reshape for cnn
        seq_len, bs, C, H, W = x.shape
        _, _, n_aux = aux.shape

        x = x.view(seq_len*bs, C, H, W)
        aux = aux.view(seq_len*bs, n_aux)

        # Front
        x = self.block1(x) 
        x = self.block2(x); #print(x.shape)

        activations = x
        # register the hook
        if register_activations:
            activations.register_hook(self.activations_hook)
        salmap = activations.detach().cpu().numpy()
        x = activations


        x = self.block3(x); #print(x.shape) Makes it too small for 32x32 model
        
        x = nn.ReLU()(x)
        x = Flatten()(x)

        features = torch.cat([x, aux], dim=-1); #print(features.shape)
        features = features.view(seq_len, bs, -1)

        features = self.fc0(features)
        #features = nn.Dropout(p=DROP)(features)

        if self.use_rnn:
            features, hidden = self.lstm(features, hidden)

        x = self.fc1(features)
        x = nn.ReLU()(x)
        #x = nn.Dropout(p=DROP)(x)

        x = self.fc2(x)

        #x[:, :, 0] = nn.Sigmoid()(x[:,:,0]) # brake
        ##x[:, :, 0] = nn.Tanh()(x[:,:,0]) # steer
        #x[:, :, 2] = nn.Sigmoid()(x[:,:,2]) # throttle
        
        if return_salmap:
            return x, hidden, salmap
        else:
            return x, hidden

