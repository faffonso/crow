# Controllers

The **Controllers** folder contains the core control algorithms used in the CROW project for crop row navigation in agricultural fields. These algorithms are responsible for steering the robot along crop rows by following waypoints generated from LiDAR-based perception data. The folder includes two main control approaches: iLQR (Iterative Linear Quadratic Regulator) and NMPC (Non-linear Model Predictive Control), both designed to ensure smooth and accurate navigation through complex, real-world agricultural environments. The NMPC was used as benchmark for our iLQR implementation.

## Overview

### iLQR (Iterative Linear Quadratic Regulator)
iLQR is a gradient-based optimization algorithm that computes an optimal trajectory by minimizing a quadratic approximation of the system’s cost function. It iteratively adjusts control inputs to ensure that the robot stays on the desired path while respecting dynamic constraints. This approach is well-suited for tasks that require precision, such as navigating narrow crop rows, and performs efficiently in real-time environments.

### NMPC (Non-linear Model Predictive Control)
NMPC is a more general and flexible control method that solves non-linear optimization problems at each time step. It predicts the future trajectory of the robot by accounting for non-linear dynamics and constraints, making it ideal for handling complex scenarios encountered in agricultural navigation. NMPC can adapt to dynamic environments, providing robust control even under challenging conditions like uneven terrain or crop variability.

 > Please explore further the iLQR and NMPC folders for detailed information on each controller's structure, files, and how to run them.