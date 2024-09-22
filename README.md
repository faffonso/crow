# CROW: A Self-Supervised Crop Row Navigation Algorithm for Agricultural Fields
![alt text](<assets/system.png>)

## Overview

This repository provides an implementation of the paper


> [**CROW: A Self-Supervised Crop Row Navigation Algorithm for Agricultural Fields**](),  
> Francisco Affonso, Felipe Andrade G. Tommaselli1, Gianluca Capezzuto, Mateus V. Gasparino, Girish Chowdhary, Marcelo Becker,
> *Paper at ?*  

CROW is a navigation system designed for agricultural fields, specifically for operation under crop canopies. The system begins with a perception module that employs a self-supervised neural network to identify crop lines from LiDAR data. Based on this information, waypoints are generated for a Model Predictive Controller (MPC) to follow the crop lines, ensuring the system remains on the reference path.

The code is organized into navigation modules, each accompanied by a brief explanation of how to use them individually and how to modify the parameters in the configuration files:

- **Inference:** LiDAR-Based Perception 
- **Wp_gen:** Waypoint Generator
- **Controllers:**
    - **NMPC:** Non-linear Model Predictive Controller solved using IPOPT 
    - **iLQR:** Non-linear Model Predictive Controller solved using Constrained iLQR


## Citation

If you find our work useful in your research, please consider citing our publication:

```bibtex
@inproceedings{affonso023navigating,
    title={Navigating with finesse: Leveraging neural network-based lidar perception and ilqr control for intelligent agriculture robotics},
    author={Pinto, Francisco Affonso and Tommaselli, Felipe Andrade G and Gasparino, Mateus V and Becker, Marcelo},
    booktitle={2023 Latin American Robotics Symposium (LARS), 2023 Brazilian Symposium on Robotics (SBR), and 2023 Workshop on Robotics in Education (WRE)},
    pages={502--507},
    year={2023},
    organization={IEEE}
}
```

## Instalation

The requirements are not strict hard requirements, but there may be some differences in performance or compatibility (not tested).

- Linux - Ubuntu 20.04
- Python3
- CUDA

To install CasADi (an optimization framework), run:
```shell
source install_casadi.sh
```

To set up the environment needed:
```shell
# Create a virtual environment
python3 -m venv crow_env
source crow_env/bin/activate

# Install dependencies
pip3 install -r requirements.txt
```

If you prefer not to use a virtual environment, you can install the dependencies directly from `requirements.txt`.

