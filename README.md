# CROW: A Self-Supervised Crop Row Navigation Algorithm for Agricultural Fields

<p align="center">
    <img src="assets/ts.png" alt="alt text" width="400"/>
</p>

## Overview

This repository provides the **official implementation** of the paper:


> [**CROW: A Self-Supervised Crop Row Navigation Algorithm for Agricultural Fields**](https://link.springer.com/article/10.1007/s10846-025-02219-2),    
> Francisco Affonso, Felipe Andrade G. Tommaselli, Gianluca Capezzuto,     
> Mateus V. Gasparino, Girish Chowdhary, Marcelo Becker,  
> *Published in Journal of Intelligent & Robotic Systems, 2025*  

We introduce CROW, a navigation system tailored for agricultural fields, specifically for operation under crop canopies. The system starts with a perception module that utilizes a self-supervised neural network to detect crop lines from LiDAR data. Using this information, waypoints are generated for a Model Predictive Controller (MPC), allowing the system to follow the crop lines and maintain its position on the reference path.

The code is organized into navigation modules, each accompanied by how to modify the parameters in the configuration files:

- **Inference:** LiDAR-Based Perception 
- **Wp_gen:** Waypoint Generator
- **Controllers:**
    - **NMPC:** Non-linear Model Predictive Controller solved using IPOPT 
    - **iLQR:** Non-linear Model Predictive Controller solved using Constrained iLQR

All datasets and trained models used in this work are [fully available online](https://drive.google.com/drive/folders/1IeAhgs2SlV-Fgol1CqfxPrg_Yv2mBH2O).

## Citation

If you find our work useful in your research, please consider citing our publication:

```bibtex
@article{AffonsoTommaselli2025,
  title={CROW: A Self-Supervised Crop Row Navigation Algorithm for Agricultural Fields},
  author={Affonso, Francisco and Tommaselli, Felipe Andrade G. and Capezzuto, Gianluca and Gasparino, Mateus V. and Chowdhary, Girish and Becker, Marcelo},
  journal={Journal of Intelligent \& Robotic Systems},
  year={2025},
  publisher={Springer},
  doi={10.1007/s10846-025-02219-2}
}
```

## Instalation

The requirements are not strict hard requirements, but there may be some differences in performance or compatibility (not tested).

- Linux - Ubuntu 20.04
- ROS Noetic
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


## System Execution

This repository does not include the robot (TerraSentia) or the world environment (plantations) used in the paper. To run the code in a suitable environment, you will need a robot equipped with a 2D LiDAR. For the plantation environment, we recommend using the open-source repository [Virtual Maize Field](https://github.com/FieldRobotEvent/virtual_maize_field/tree/6896db468cec98af7a9a7ee83fdbb89a34da1816).

### Steps to Run the Code:

Once the environment is set up, follow these steps

1. **Run the perception node**:  
   Open a terminal and execute the following command:

   ```bash
   rosrun inference RTInference.py
   ```

2. **Run the waypoint generator and controller**
    
    In another terminal, run:

    ```
    roslaunch ilqr nav.launch
    ```

The system will begin controlling the robot towards the generated waypoints. You can visualize the process in RViz. If the waypoint generator or controller isn't performing robustly, you may need to adjust the parameters

#### Notes

- If you want to run the system without a suitable environment, you can proceed with the steps above as-is.
- The system's navigation modules can be customized through the respective config files, as previously mentioned.
- For more information about the Perception network training (self-supervised), please refer to the [training perception](training%20perception/README.md) section.

## Experimental Results

<p align="center">
    <img src="assets/results.png" alt="alt text" width="800"/>
</p>

We conducted six experiments at the Illinois Autonomous Farm (University of Illinois Urbana-Champaign) using 115-meter crop rows, as illustrated by the GPS/RTK trajectories in the figure above. An accompanying image shows the condition of the crops on the date of the experiments. The table below provides details on collisions that occurred during the experiments and the time taken to traverse each row.

<div align="center">

| Run # | Coll. | Av. Dist. p/ Coll. [m] | Traj. Time [s] |
|-------|-------|------------------------|----------------|
| 1     | 5     | 23.00                  | 235            |
| 2     | 4     | 28.75                  | 245            |
| 3     | 0     | -                      | 155            |
| 4     | 1     | 115.0                  | 172            |
| 5     | 4     | 28.75                  | 252            |
| 6     | 6     | 19.17                  | 278            |
| **Average** | **3.33** | **34.5** | **222.83** |
</div>


## License

This code is released under the [MIT](https://opensource.org/license/mit) for academic usage.
