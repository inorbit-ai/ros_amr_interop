# massrobotics_amr_sender

Configuration-based ROS package for sending MassRobotics [AMR Interop Standard messages](https://github.com/MassRobotics-AMR/AMR_Interop_Standard) to compliant receivers.

# Package installation

## From binary packages

Coming soon.

## Building from source

Make sure `ros` is installed properly. Then clone this repository inside your `src` folder on your local workspace and build the package executing the following commands:

```bash
# Create a ROS workspace and go into it - if you don't have one already
mkdir -p ~/ros_ws/src && cd ros_ws/
# Clone the repo inside the workspace
git clone --branch noetic-devel https://github.com/inorbit-ai/ros_amr_interop.git ./src
# Install dependencies
rosdep update && rosdep install --ignore-src --from-paths src/
# Run the build
catkin config --install massrobotics_amr_sender
catkin build
colcon build --packages-select massrobotics_amr_sender
```
# Node configuration

A configuration file must be provided to define how ROS1 messages are mapped to different AMR Interop Standard messages. A [sample_config.yaml](https://github.com/inorbit-ai/ros_amr_interop/blob/foxy-devel/massrobotics_amr_sender_py/sample_config.yaml) is provided for reference.

# Running the sender node

The node takes the MassRobotics AMR config file path as parameter. If not provided, it is assumed the file is on the current directory.

```bash
# Remember to source the ROS2 environment from the binary installation or your workspace overlay
source devel/setup.bash
# Launch the node pointing to your configuration file
roslaunch massrobotics_amr_sender massrobotics_amr_sender.launch config_file:=/path/to/config.yaml
```


# Tests

TODO.