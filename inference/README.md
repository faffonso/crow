# Inference

The **Inference** folder contains the perception system for the CROW project, which plays a crucial role in enabling the robot to navigate autonomously through agricultural fields. This system is responsible for processing sensor data, specifically from LiDAR, to detect crop rows in real-time. The detected rows are then used to generate waypoints, which guide the robot’s navigation through the field.

## Overview

The perception system within the **Inference** folder is designed to analyze complex agricultural environments using advanced machine learning models. It processes incoming data, detects important features like crop rows, and feeds this information to the navigation and control systems. The accurate detection of crop rows is essential for the robot to follow the correct path and avoid obstacles within the field.

### Key Concepts
- **Real-Time Processing**: The inference system operates in real-time, constantly analyzing LiDAR data to provide up-to-date information about the crop rows in the environment.
- **Neural Network Models**: Pre-trained models are used to detect crop rows from the raw sensor data. These models are specifically trained to handle the variability and complexity of agricultural environments, ensuring reliable performance in diverse field conditions.
- **Waypoint Generation**: Based on the detected crop rows, waypoints are generated for the robot to follow. These waypoints are crucial for the control system to navigate the robot accurately along the rows.

> For more information about the Waypoint Generation system, please refer to the `wp_gen` folder.

## Usability

In the CROW project, the **Inference** system serves as the perception layer that interprets the environment and provides the necessary data for navigation. It directly interacts with the control systems through the Waypoint Generator framework, enabling the robot to adjust its path and maintain the correct trajectory through the field.

Without the accurate detection of crop rows provided by the inference system, the robot would be unable to follow the desired path, making this component essential for autonomous navigation in agricultural settings.

### General Workflow
1. **LiDAR Data Input**: The system receives raw data from the LiDAR sensors mounted on the robot.
2. **Data Processing and Detection**: Neural network models analyze the data, detecting the crop rows in real-time.
3. **Waypoint Generation**: The detected rows are used to generate waypoints, which the control system uses to guide the robot through the field.
4. **Navigation**: The waypoints are sent to the robot’s waypoint generator and control systems, enabling smooth and efficient navigation along the crop rows.

By continuously processing data and generating waypoints, the inference system ensures that the robot remains on course, avoiding obstacles and adapting to the dynamic environment of the agricultural field.
