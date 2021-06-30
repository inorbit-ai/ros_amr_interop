# massrobotics_amr_sender

Configuration-based ROS2 package for sending MassRobotics [AMR Interop Standard messages](https://github.com/MassRobotics-AMR/AMR_Interop_Standard) to complaint receivers.

# Installing

## From binary packages

The node is available as a released package and can be added manually to your ROS2 build installation:

```sudo apt-get install ros-foxy-massrobotics-amr-sender```

Or listed as a rosdep dependency and installed with `rosdep update`

## Building from source

Make sure `ros2` is installed properly. Clone this repository inside your `src` folder on your local workspace and build the package:

```bash
# Create a ROS2 workspace and go into it - if you don't have one already
mkdir -p ~/ros2_ws/src && cd ros2_ws/
# Clone the repo inside the workspace
git clone https://github.com/inorbit-ai/ros_amr_interop.git ./src
# Install dependencies
rosdep install --ignore-src --from-paths src/
# Run the build
colcon build --packages-select massrobotics_amr_sender
```
# Configuring

A configuration file must be provided to define how ROS2 messages are mapped to different AMR Interop Standard messages. A [sample_config.yaml](https://github.com/inorbit-ai/ros_amr_interop/blob/foxy-devel/massrobotics_amr_sender_py/sample_config.yaml) is provided for reference.

# Running

The node takes the MassRobotics AMR config file path as parameter. If not provided, it is assumed the file is on the current directory.

```bash
# Remember to source the ROS2 environment from the binary installation or your workspace overlay
source install/setup.bash
# Run the node pointing to your configuration file
ros2 run massrobotics_amr_sender massrobotics_amr_node \
    --ros-args -p config_file:=/path/to/config.yaml --log-level debug
```


# Running tests

On you local workspace:

```bash
colcon test --packages-select massrobotics_amr_sender
colcon test-result --verbose
```
