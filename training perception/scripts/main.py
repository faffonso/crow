#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: runs the neural network inference and generate the model based on the "runid" from the date and time. 
model: mobilenet v2
dataloader: see "dataloader.py" and "test_dataloader.py" for further information 

@author: Felipe-Tommaselli
"""

import os
import sys
import torch
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime 

import torch.nn as nn
from torchvision import datasets
import torchvision.models as models
from efficientnet_pytorch import EfficientNet

torch.cuda.empty_cache()

from dataloader import *
from pre_process import *


def getData(csv_path, train_path, batch_size, runid, num_workers=0):
    ''' get images from the folder "data" and return a DataLoader object '''
    
    ############ CREATE DATASET OBJECT ############
    dataset = NnDataLoader(csv_path, train_path, runid)

    ############ DATA AUGMENTATION ############
    print(f'dataset size: {len(dataset)}')

    ############ DATASET SPLIT (TRAIN & VAL) ############
    train_size, val_size = int(0.7*len(dataset)), np.ceil(0.3*len(dataset)).astype('int')
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [train_size, val_size])
    
    ############ DATASET DEFINITION ############
    train_data = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    val_data  = DataLoader(val_dataset, batch_size=batch_size, shuffle=True, num_workers=num_workers)

    print(f'train size: {train_size}, val size: {val_size}')
    _ = input('----------------- Press Enter to continue -----------------')
    return train_data, val_data


def train_model(model, criterion, optimizer, scheduler, train_loader, val_loader, num_epochs):
    ''' Train model function: train (if) and validate (else):
    Forward pass predicts outputs; backward pass adjusts parameters; training optimizes parameters for minimizing loss, 
    while validation assesses model performance on unseen data.
    - Feed forward pass: Input data is passed through the neural network to produce a prediction. 
    - Backward pass: Prediction errors are propagated back through the network to adjust the weights, optimizing the 
    model's performance during training and validating.
    '''
    train_losses = []
    val_losses = []

    for epoch in range(num_epochs):
        model.train()
        running_loss = 0.0
        ############ TRAINING ############
        for i, data in enumerate(train_loader):
            ############ FORMAT CONVERT ############
            images, labels = data['image'], data['labels']
            # image dimension: (batch, channels, height, width)
            images = images.type(torch.float32).to(device)
            images = images.unsqueeze(1)
            # convert to format: tensor([[value1, value2, value3, value4], ...])
            # this is: labels for each image, "batch" times -> shape: (batch, 4)
            labels = [label.type(torch.float32).to(device) for label in labels]
            labels = torch.stack(labels)
            labels = labels.permute(1, 0) 
            ############ MODEL TRAINING ############
            outputs = model(images)
            loss = criterion(outputs, labels) 
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            running_loss += loss.item()
        else:
        ############ VALIDATING ############
            with torch.no_grad():                
                model.eval() # evaluation mode
                val_loss = 0
                for i, data in enumerate(val_loader):
                    ############ FORMAT CONVERT ############
                    images, labels = data['image'], data['labels']
                    images = images.type(torch.float32).to(device)
                    images = images.unsqueeze(1)
                    labels = [label.type(torch.float32).to(device) for label in labels]
                    labels = torch.stack(labels)
                    labels = labels.permute(1, 0)
                    outputs = model.forward(images)
                    val_loss += criterion(outputs, labels).item()
                val_losses.append(val_loss/len(val_loader))
            pass
        scheduler.step()
        train_losses.append(running_loss/len(train_loader))
        print(f'[{epoch+1}/{num_epochs}] .. Train Loss: {train_losses[-1]:.5f} .. val Loss: {val_losses[-1]:.5f}')

    results = {'train_losses': train_losses, 'val_losses': val_losses,}
    return results

def plotResults(results, epochs, lr, runid):
    ''' Plot the results with matplotlib: train and val loss on the same plot  '''
    fig, ax = plt.subplots()
    ax.plot(results['train_losses'], label='Training Loss', marker='o')
    ax.plot(results['val_losses'], label='Validation Loss', marker='o')
    ax.set_xlabel('Epochs')
    ax.set_ylabel('Loss')
    ax.set_title('Learning Loss Plot (L1 Loss)\nFinal Training Loss: {:.4f}'.format(results['train_losses'][-1]))
    ax.grid(True)
    ax.legend()
    plt.tight_layout()

    fig.savefig(f'losses_{runid}.png')

if __name__ == '__main__':
    ############ START ############
    global device
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print('Using {} device'.format(device))

    ############ RUN ID ############
    day_time = datetime.now().strftime("%d-%m-%Y_%H-%M-%S") 
    runid = str(day_time) # id of this particular run

    ############ PARAMETERS ############    
    epochs = 50 # empirical
    lr = float(0.009) # empirical
    step_size = 8 # empirical
    gamma = 0.4 # empirical
    batch_size = 140 # empirical
    weight_decay = 0 # empirical

    ############ DATA ############
    csv_path = os.path.join(os.getcwd(), 'data', 'artificial_data', 'tags', 'Artificial_Label_Data11.csv')
    train_path = os.path.join(os.getcwd(), 'data', 'artificial_data', 'train11')
    train_data, val_data = getData(batch_size=batch_size, csv_path=csv_path, train_path=train_path, runid=runid)

    ############ MODEL ############
    model = models.mobilenet_v2()

    ########### MOBILE NET ########### 
    model.features[0][0] = nn.Conv2d(1, 32, kernel_size=3, stride=2, padding=1, bias=False)
    num_ftrs = model.classifier[1].in_features
    model.classifier[1] = nn.Sequential(
    nn.Linear(num_ftrs, 512),
    nn.BatchNorm1d(512),
    nn.ReLU(inplace=True),
    nn.Linear(512, 3)
    )
    model = model.to(device)

    ############ NETWORK ############
    criterion = nn.L1Loss()
    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=step_size, gamma=gamma)

    ############ TRAINING ############
    results = train_model(model=model, criterion=criterion, optimizer=optimizer, scheduler=scheduler, train_loader=train_data, val_loader=val_data, num_epochs=epochs)

    ############ RESULTS ############
    plotResults(results, epochs, lr, runid)

    ############ SAVE MODEL ############
    path = os.getcwd() + '/models/' + 'model' + '_' + runid + '.pth'
    torch.save(model.state_dict(), path)
    print(f'Saved PyTorch Model State to:\n{path}')