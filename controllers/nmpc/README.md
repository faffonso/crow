# NMPC Controller

This folder contains the implementation of the NMPC (Non-linear Model Predictive Control) controller for crop row navigation. The NMPC controller optimizes the trajectory in real-time by solving a non-linear control problem, making it highly suitable for complex navigation tasks like those in agricultural environments.

## Structure

### Files
1. **controller.launch**: Launch file for running the NMPC controller node in ROS.
2. **nav.launch**: Launch file for the navigation stack, which includes waypoint generation and NMPC control.
3. **nav_crop.launch**: Launch file specifically designed for running the navigation stack in a crop environment.
4. **nmpc_controller.py**: Main NMPC controller implementation. This script performs the optimization of the robot's trajectory using NMPC principles.
5. **nmpc_node.py**: ROS node that integrates the NMPC controller into the navigation stack, handling real-time control and interactions with other modules.
6. **setup.py**: Setup script to configure the NMPC package in the ROS environment. It uses the `catkin` build system to register the NMPC controller.

### Overview of NMPC

The NMPC controller solves the non-linear optimal control problem at each timestep, predicting the future trajectory and determining the optimal set of control actions to follow the crop rows. It handles complex dynamics and constraints, providing real-time performance for navigating agricultural environments.
