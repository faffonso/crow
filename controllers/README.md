# Controllers

The **Controllers** folder contains the core control algorithms used in the CROW project for crop row navigation in agricultural fields. These algorithms are responsible for steering the robot along crop rows by following waypoints generated from LiDAR-based perception data. The folder includes two main control approaches: iLQR (Iterative Linear Quadratic Regulator) and NMPC (Non-linear Model Predictive Control), both designed to ensure smooth and accurate navigation through complex, real-world agricultural environments. The NMPC was used as benchmark for our iLQR implementation.

## Overview

### iLQR (Iterative Linear Quadratic Regulator)
iLQR is a gradient-based optimization algorithm that computes an optimal trajectory by minimizing a quadratic approximation of the system's cost function. It iteratively refines control inputs to keep the robot on the desired path while adhering to dynamic constraints. This approach is particularly effective as a Model Predictive Control (MPC) solver, enabling real-time optimization for action control solutions in dynamic environments.

### NMPC (Non-linear Model Predictive Control)
NMPC is a more general and flexible control strategy that formulates nonlinear optimization problems. By incorporating nonlinear dynamics and constraints, it predicts the robot's future trajectory, making it particularly effective for handling complex scenarios such as agricultural navigation. Since our approach leverages iLQR, we developed a parallel package that utilizes traditional methods like IPOPT to solve the NMPC formulation, serving as a baseline for performance comparison.

 > Please explore further the iLQR and NMPC folders for detailed information on each controller's structure, files, and how to run them.