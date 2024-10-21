#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Class that preprocess the dataset for the neural network. The data it is loaded from the data_loader class.
This class contains a lot of old and new implementations and the idea is exactly to storage all this process 
in case it is needed in the future. Nowadays, I am only using the artificial_data with the "parametrization",
"standard_extract_label" and "standard_deprocess".

@author: Felipe-Tommaselli
"""

import copy
import os
import cv2
import matplotlib.pyplot as plt
import torch


class PreProcess:

    def __init__(self, dataset) -> None:
        ''' Constructor of the class. '''

        self.labels = copy.deepcopy(dataset['labels'])
        self.image = copy.deepcopy(dataset['image'])
        

    def pre_process(self) -> list:
        ''' Returns the processed data. '''
        self.image = self.process_image(self.image, self.labels)
        self.labels = self.process_label(self.labels)

        return self.labels, self.image

    #*############[[[ARTIFICIAL DATASET]]]#############

    @staticmethod
    def parametrization(m1, m2, b1, b2):
        w1 = 1/m1
        w2 = 1/m2
        q1 = -b1/m1
        q2 = -b2/m2
        # note that the in (process) and the out (deprocess) are the same operations
        # we are using the same operations for process and deprocess :)
        return [w1, w2, q1, q2]

    # ############################################################################################
    #   (MEAN AND STD IMPLENTATION) UTILITIES FUNCTIONS FOR DEPROCESSING AND ROUTINE OPERATIONS
    # ############################################################################################

    @staticmethod
    def standard_deprocess(image, label, mean, std):
        ''' Returns the deprocessed image and label. '''

        if len(label) == 3:
            # we suppose m1 = m2, so we can use the same deprocess
            print('supposing m1 = m2')   
            w1, q1, q2 = label
            w2 = w1
        elif len(label) == 4:
            print('not supposing m1 = m2')        
            w1, w2, q1, q2 = label

        # DEPROCESS THE LABEL
        w1_original = (w1 * std[0]) + mean[0]
        w2_original = (w2 * std[1]) + mean[1]
        q1_original = (q1 * std[2]) + mean[2]
        q2_original = (q2 * std[3]) + mean[3]

        m1, m2, b1, b2 = PreProcess.parametrization(w1_original, w2_original, q1_original, q2_original)

        label = [m1, m2, b1, b2]

        return label

    @staticmethod
    def standard_extract_label(labels, mean, std):
        ''' This function aims to extract infos more relevants for the neural network. 
        For now, the only thing that gave performance to the cnn was "b" intersection in 
        a differente parametrization. '''

        m1, m2, b1, b2 = labels

        w1 = (m1 - mean[0]) / std[0]
        w2 = (m2 - mean[1]) / std[1]
        q1 = (b1 - mean[2]) / std[2]
        q2 = (b2 - mean[3]) / std[3]

        return [w1, w2, q1, q2]
