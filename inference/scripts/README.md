# Inference Scripts

This folder contains the core scripts used for real-time inference in the CROW project, enabling the perception system to detect crop rows in agricultural fields using LiDAR data. The perception system processes the incoming data and generates meaningful crop row information that is then used by the controllers to guide the robot through the field.

## Overview

### RTInference.py

The main script in this folder is **RTInference.py** (Real-Time Inference). It runs the perception system responsible for identifying crop rows in real-time based on LiDAR data. This script interfaces with trained models to process the sensor data and extract the necessary information to generate waypoints for navigation.

### Key Functions
- **Real-Time Data Processing**: The script captures data from the LiDAR sensor and feeds it through a trained neural network to detect crop rows.
- **Model Integration**: It loads pre-trained models stored in the `models` folder, which are used to interpret the incoming data.
- **Waypoint Generation**: Based on the detected crop rows, it generates waypoints that guide the robot's navigation along the field.


### How to Run
To run the real-time inference script, execute the following command:

```bash
rosrun inference RTInference.py
```

This will start the perception system and begin processing LiDAR data to detect crop rows and generate waypoints for navigation.