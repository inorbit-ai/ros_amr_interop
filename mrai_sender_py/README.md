# mrai_sender

Configuration-based ROS2 package for sending MassRobotics [AMR Interop Standard messages](https://github.com/MassRobotics-AMR/AMR_Interop_Standard) to complaint receivers.

## Running from source

Make sure `ros2` is installed properly. Clone this repository inside your `src` folder on your local workspace and build the package:

```bash
mkdir -p ~/ros2_ws/src && cd ros2_ws/
git clone https://github.com/inorbit-ai/ros-amr-interop.git ./src

colcon build --packages-select mrai_sender
```

## Running the node

The node takes the Mass AMR config file path as parameter. If not provided, it is assumed the file is on the current directory.

```bash
# Source the local overlay by running `. install/setup.sh` if
# using bash or `. install/setup.zsh` if using zsh.
# Also, install dependencies by running `rosdep install --ignore-src --from-paths src/`
ros2 run mrai_sender mrai_sender \
    --ros-args -p config_file:=/path/to/config.yaml --log-level debug
```

## Running tests

On you local workspace:

```bash
colcon test --packages-select mrai_sender
colcon test-result --verbose
```
