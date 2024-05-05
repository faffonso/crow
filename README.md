# Terrasentia Navigation

### Installing iLQR C++ depencies

* Eigen3: Install with `sudo apt install libeigen3-dev`.
* CasADi: Follow the instructions in this [tutorial](https://github.com/zehuilu/Tutorial-on-CasADi-with-CPP).


### Testing Navigation System

Open the world in Gazebo with the Terrasentia robot using the [terrasentia_description](https://github.com/Felipe-Tommaselli/terrasentia_description) package:

```bash
roslaunch virtual_maize_field simulation.launch
```
```bash
roslaunch terrasentia_description launch_default_ts3.launch
```

In the `catkin_ws` directory, run the Inference server (Perception):

```bash
cd src/terrasentia_navigation/inference/scripts
python3 RTInference.py    
```

Again in the catkin_ws directory, launch the controller which initializes the waypoint generator:

```bash
roslaunch nmpc nav_crop.launch     
```

This launch will start the entire system and open a Rviz visualization of all important topics:

* /tf: Global position of the robot
* /lidar_plot: Image to visualize LiDAR readings
* /terrasentia/goal: Last generated waypoint
* /path: Control trajectory

After this, each package has a .yaml file to configure some parameters:

Waypoint Generator (wp_gen_params.yaml):
```yaml
wp_gen/img:
  row_height: 7.5   # [m]
  row_width: 1.35   # [m]
  img_height: 244   # [px]
  img_width: 244    # [px]
  D: 2.5            # Waypoint Distance [m]

wp_gen/odom:
  topic: "/terrasentia/will/odom"
  frame_id: "terrasentia/will/odom"

```

NMPC Controller (nmpc_params.yaml)
```yaml
controller/NMPC: 
# Optimization Setup
  dt: 0.1 # Sampling time [s]
  N:  15  # Prediction Horizon

  # Weight Matrices
  Q_x:      1.0 # X cost
  Q_y:      1.0 # Y cost
  Q_theta:  1.0 # Theta cost

  R_v:      1.0 # Linear speed cost
  R_omega:  1.0 # Angular speed cost

  # Boundary Action Control
  v_max:      0.8 # [m/s]
  omega_max:  1.5 # [rad/s]

## Odometry Params
odom:
  topic: "/terrasentia/will/odom"
  frame_id: "/terrasentia/will/odom"

```

**Observation**: If you need to change the frequencies of each node, you can modify the corresponding parameters within the `<package>_node.py` files.