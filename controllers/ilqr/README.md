# iLQR Controller

This folder contains the implementation of the iLQR (Iterative Linear Quadratic Regulator) controller for crop row navigation. The iLQR controller optimizes the trajectory of the robot through a cost-minimization approach while accounting for system dynamics and control constraints.

## Structure

### Files
1. **controller.launch**: Launch file for running the iLQR controller node in ROS.
2. **nav.launch**: Launch file for the navigation stack, including both waypoint generation and iLQR control.
3. **ControllerTime.msg**: Defines a custom ROS message for managing timing and synchronization in the iLQR controller.
4. **ilqr_node.cpp**: Main iLQR controller node implementation. It handles the control loop and runs the iLQR optimization in real-time.
5. **ilqr.cpp**: Contains the core iLQR algorithm. This file defines how the iLQR optimizes the trajectory by minimizing the cost function.
6. **dynamics.cpp**: Defines the system dynamics that the iLQR uses to predict the state evolution of the robot over time.
7. **cost.cpp**: Implements the cost function used by iLQR to evaluate and optimize the trajectory.
8. **utils.cpp**: Utility functions supporting various operations in the iLQR codebase.

### Overview of iLQR

The iLQR controller is a gradient-based optimization method that solves for the optimal control trajectory by iteratively minimizing a quadratic approximation of the system’s cost function. It is particularly suited for navigation tasks like crop row following, where precision is critical. 

### How to Run
1. To launch the iLQR controller, run the following command:
   ```bash
   roslaunch ilqr nav.launch
    ```

This will start the iLQR controller, which uses the generated waypoints to control the robot’s navigation along the crop rows.