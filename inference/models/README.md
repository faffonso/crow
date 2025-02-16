# Inference Models

The **models** folder contains the pre-trained neural network models and configuration files used by the inference system in the CROW project. These models are responsible for processing LiDAR data and detecting crop rows in real-time. The outputs from these models are then used to generate waypoints for navigation.

## Overview

### Pre-Trained Models

The models in this folder, such as `model_06-04-2024_21-22-37.pth` (the one used in all benchmarks), are the neural network weights that have been trained on LiDAR data from agricultural fields. These models enable the perception system to accurately detect crop rows and provide reliable information for navigation.

> Please refer to the `training_perception` folder for more informations about the training process.

- **model_*.pth**: These are the PyTorch model files containing the weights and architecture of the neural network used for crop row detection.

### params.json

The **params.json** file contains the mean and standard deviation values used to normalize the input data before it is passed into the model. This normalization ensures consistent model performance and accurate crop row detection.

Key fields in **params.json**:
- **mean0, mean1, mean2, mean3**: Mean values used for normalizing the corresponding input channels.
- **std0, std1, std2, std3**: Standard deviation values for normalizing the input channels.
- **id**: Model ID that helps to track which configuration corresponds to which trained model.

### How to Use

The inference system loads these models automatically during runtime. To modify or update the models, simply replace the existing `.pth` files or adjust the normalization parameters in **params.json**.

When running the inference system:

```bash
rosrun inference RTInference.py
``` 