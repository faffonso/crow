# Scripts Training Perception Module

> This package contains the "source/script" of the project. The files in this folder are basically about the **training** of the Neural Network models. 

## Folder archtecture

This folder is as follows:
* parent: `/src`
* child 1: `/utils`

With that, all files in the `src` folder, are related to the main files in the training pipeline:

### Training `/src` scripts

We have 3 scripts: `main.py`, `dataloader.py`, `pre_process.py` (training).
The `main.py` script runs the neural network params and the model_fit (the heart of the project). This script also uses the 
`dataloader.py` class to create the dataset. Finally, the `dataloader.py` uses some functions on the `pre_process.py`, but it is worth noticing that the `pre_process.py` contains a library of functions for different purposes.


### `/utils` scripts

This folder contains different scripts for many applications united by their usefulness. In general, the `artificial_generator.py`, `artificial_test.ipynb`, `create_dataset.py` do not share much in common, but they are handy scripts that are used often (not every time). For that, they are classified as "utils".

* `artificial_generator.py`: Create the dataset based on some parameters in the script that generate a new raw_data dataset full of images and labels;
* `artificial_test.ipynb`: Test the points distributions before using the `artificial_generator.py`;
* `create_dataset.py`: Create a real-time dataset based on the `/terrasentia/scan` rostopic;

It is worth noticing that both `lidar_tag.py` and `lidar2images.py` are deprecated and have not been used for a long time since there is no need to use hand-made labelling anymore.
