#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
The class that loads the dataset for the neural network. 
This data loader loads all data from the "Images" folder and the specific labels from the ".csv" file.
With that, two processes take place: Data normalization and Dataset creation. 
1. Data normalization: The normalization with the best results is the "standard": (xi - mean(x)) / (std(x))
    - the mean an std used for each label is stored in the params.json per "ruind" as an identifier
    - this distribution guarantees mean = 0 and std = 1, with a balanced dataset 
2. Dataset creation succeed through the __get_item__() called from the main.py
    - it is worth noticing that there are two options: load each image at each __get_item__() call or 
    load the entire image dataset. The second option consumes a lot of RAM memory but is faster during the training. 

Difference from 'dataloader.py' and 'test_dataloader.py'
dataloader.py: loads the entire image dataset once
test_dataloader.py: load each image at each get_item call

@author: Felipe-Tommaselli
""" 
import warnings
warnings.filterwarnings("ignore")

import sys
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import json
import copy

import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split, ConcatDataset, Subset

from pre_process import *

# move from root (\src) to \assets\images
if os.getcwd().split(r'/')[-1] == 'src':
    os.chdir('..') 

class NnDataLoader(Dataset):
    ''' Dataset class for the lidar data with images. '''
    
    def __init__(self, csv_path, train_path, runid):
        ''' Constructor of the class. '''
        labels = pd.read_csv(csv_path)
        self.train_path = train_path
        self.images = list()
        self.labels_list = list()

        ############ LOAD DATASET ############
        for idx in range(len(labels)):
            step = labels.iloc[idx, 0] # step number by the index
            full_path = os.path.join(self.train_path, 'image'+ str(step) +'.png') 

            ############ PROCESS IMAGE ############
            image = cv2.imread(full_path, -1)
            image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_LINEAR)
            image = image[:, :, 1] # only green channel
            self.images.append(image)

            ############ PROCESS LABEL ############
            wq_labels = NnDataLoader.process_label(labels.iloc[idx, 1:])
            self.labels_list.append(wq_labels) # take step out of labels

        ############ OBTAIN MEAN AND STD FOR NORMALIZATION ############
        labels_numpy = np.asarray(self.labels_list)
        self.std = np.std(labels_numpy, axis=0)
        self.mean = np.mean(labels_numpy, axis=0)
        
        ############ SAVE MEAN AND STD IN "params.json" ############
        new_params = {
            'id': runid,  # You can set the appropriate id value
            'mean0': self.mean[0],
            'mean1': self.mean[1],
            'mean2': self.mean[2],
            'mean3': self.mean[3],
            'std0': self.std[0],
            'std1': self.std[1],
            'std2': self.std[2],
            'std3': self.std[3]
        }

        if os.getcwd() == 'src':
            os.chdir('..')
        filename = './models/params.json'

        with open(filename, 'r') as file:
            existing_data = json.load(file)

        existing_data.append(new_params)
        with open(filename, 'w') as file:
            json.dump(existing_data, file, indent=4)


    def __len__(self) -> int:
        ''' Returns the length of the dataset (based on the labels). '''
        return len(self.labels_list)

    def __getitem__(self, idx: int) -> dict:
        ''' Returns the sample image of the dataset. '''
        image = copy.deepcopy(self.images[idx])
        labels = copy.deepcopy(self.labels_list[idx])
        
        labels = PreProcess.standard_extract_label(labels, self.mean, self.std)

        #! suppose m1 = m2
        w1, w2, q1, q2 = labels
        labels = [w1, q1, q2] # removing w2
        return {"labels": labels, "image": image, "angle": 0}

    @staticmethod
    def process_label(labels):
        ''' Process the labels to be used in the network. Normalize azimuth and distance intersection.'''
        IMG_SIZE = 224 #px

        m1 = -labels[0]
        m2 = -labels[1]
        b1 = IMG_SIZE - labels[2]
        b2 = IMG_SIZE - labels[3]

        # obs: IMG_SIZE change matplotlib and opencv start y to the origin 

        # NORMALIZATION WITH w1, w2, q1, q2        
        w1, w2, q1, q2 = PreProcess.parametrization(m1, m2, b1, b2)
        return [w1, w2, q1, q2]



