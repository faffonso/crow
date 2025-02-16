# Waypoint Generator (wp_gen)

The **wp_gen** folder contains the waypoint generation system for the CROW project, which acts as the link between the perception (inference) and control (iLQR/NMPC) modules. The waypoint generator takes the crop row data detected by the perception system and translates it into navigable waypoints, allowing the control system to guide the robot through the field accurately.

## Overview

The waypoint generation system processes the real-time data from the inference module, which detects crop rows using LiDAR. It then generates a series of waypoints that represent the optimal path for the robot to follow. These waypoints are passed to the control system, ensuring that the robot remains aligned with the crop rows and avoids any obstacles.

### Key Concepts
- **Waypoint Generation**: The core functionality of this module is to generate waypoints based on the perception system’s output. These waypoints serve as targets for the control algorithms, guiding the robot through the field in a straight and efficient manner.
- **Adaptability**: The waypoint generator runs in real-time, meaning it can adjust the robot's path dynamically as new crop row data is processed by the perception system. This ensures the robot can adapt to changes in the environment, such as variations in crop row alignment or obstacles.
- **Path Optimization**: The waypoints are generated with the goal of keeping the robot aligned with the crop rows, avoiding unnecessary deviations, and optimizing the navigation path.

## Usability

The waypoint generation module is critical in translating perception data into actionable control inputs. Without the waypoints, the control system would not have the precise path targets needed to navigate through the field. This module ensures that the robot maintains a consistent trajectory along the crop rows by continuously providing updated waypoints based on real-time perception data.

The **wp_gen** system is the "middleman" in the CROW navigation pipeline, making sure that the data flowing from the perception module is converted into a navigable path that the control system can act on.